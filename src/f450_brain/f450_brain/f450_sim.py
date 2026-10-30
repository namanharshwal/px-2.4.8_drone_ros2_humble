#!/usr/bin/env python3
# Copyright (c) 2025-2026 Naman Harshwal. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
F450 Hybrid Simulation & Hardware Synchronization Engine
Author: Naman Harshwal

Bridges keyboard teleop commands to both the physical Pixhawk 2.4.8 hardware
and a 6-DOF Real-Time Digital Twin Simulation with 3D RViz2 visualization:
- Integrates 50Hz Offboard Trajectory Setpoints & Heartbeats to PX4
- Broadcasts dynamic TF tree: 'map' -> 'odom' -> 'base_link'
- Publishes /joint_states for spinning 3D propellers
- Records flight path history on /f450/path
"""

import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from rclpy.executors import ExternalShutdownException

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleStatus,
    VehicleCommand
)

def euler_to_quaternion(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]

class F450SimEngine(Node):
    def __init__(self):
        super().__init__('f450_sim_engine')

        # PX4 QoS
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Standard ROS 2 QoS
        std_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, std_qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.hw_attitude_callback, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.hw_status_callback, px4_qos)

        # Publishers to PX4 Hardware
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        # Simulation & Visualization Publishers
        self.odom_pub = self.create_publisher(Odometry, '/f450/odom', std_qos)
        self.path_pub = self.create_publisher(Path, '/f450/path', std_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/f450/pose', std_qos)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', std_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Drone State Variables
        self.pos = [0.0, 0.0, 0.0]     # X, Y, Z (meters)
        self.vel = [0.0, 0.0, 0.0]     # Vx, Vy, Vz (m/s)
        self.euler = [0.0, 0.0, 0.0]   # Roll, Pitch, Yaw (radians)
        self.quat = [1.0, 0.0, 0.0, 0.0] # w, x, y, z

        # Desired Velocities from Teleop
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_yaw_rate = 0.0

        # Hardware Status
        self.hw_attitude_available = False
        self.hw_quat = [1.0, 0.0, 0.0, 0.0]
        self.hw_armed = False

        # Propeller visual rotation angle
        self.prop_angle = 0.0
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # Main 50Hz Loop (20ms)
        self.timer = self.create_timer(0.02, self.sim_loop)
        self.get_logger().info("F450 Hybrid Simulation & Hardware Engine running at 50Hz.")

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_vz = msg.linear.z
        self.cmd_yaw_rate = msg.angular.z

    def hw_attitude_callback(self, msg: VehicleAttitude):
        self.hw_attitude_available = True
        # PX4 quaternion is [w, x, y, z] in NED frame.
        # Convert to ENU for ROS: w=w, x=x, y=-y, z=-z
        self.hw_quat = [float(msg.q[0]), float(msg.q[1]), float(-msg.q[2]), float(-msg.q[3])]

    def hw_status_callback(self, msg: VehicleStatus):
        self.hw_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def sim_loop(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        dt = 0.02

        # 1. Dispatch 50Hz Offboard Stream to Real PX4 Hardware
        self.publish_px4_offboard_heartbeat()
        self.publish_px4_trajectory()

        # 2. Physics & Kinematics Update
        # Update Yaw
        self.euler[2] += self.cmd_yaw_rate * dt
        yaw = self.euler[2]

        # Rotate body-frame command velocities into world frame (ENU)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        world_vx = self.cmd_vx * cos_yaw - self.cmd_vy * sin_yaw
        world_vy = self.cmd_vx * sin_yaw + self.cmd_vy * cos_yaw
        world_vz = self.cmd_vz

        # Smooth velocity integration
        alpha = 0.25
        self.vel[0] = (1.0 - alpha) * self.vel[0] + alpha * world_vx
        self.vel[1] = (1.0 - alpha) * self.vel[1] + alpha * world_vy
        self.vel[2] = (1.0 - alpha) * self.vel[2] + alpha * world_vz

        # Position integration
        self.pos[0] += self.vel[0] * dt
        self.pos[1] += self.vel[1] * dt
        self.pos[2] = max(0.0, self.pos[2] + self.vel[2] * dt)

        # Dynamic Bank / Tilt Angles based on acceleration & velocity
        desired_pitch = -self.cmd_vx * 0.15   # Forward velocity tilts pitch down
        desired_roll = self.cmd_vy * 0.15    # Right velocity tilts roll right
        self.euler[0] = (1.0 - alpha) * self.euler[0] + alpha * desired_roll
        self.euler[1] = (1.0 - alpha) * self.euler[1] + alpha * desired_pitch

        # If real hardware telemetry is active, blend attitude with real hardware IMU
        if self.hw_attitude_available:
            self.quat = self.hw_quat
        else:
            self.quat = euler_to_quaternion(self.euler[0], self.euler[1], self.euler[2])

        # 3. Broadcast TF: map -> odom and odom -> base_footprint
        t_odom = TransformStamped()
        t_odom.header.stamp = stamp
        t_odom.header.frame_id = 'map'
        t_odom.child_frame_id = 'odom'
        t_odom.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_odom)

        t_base = TransformStamped()
        t_base.header.stamp = stamp
        t_base.header.frame_id = 'odom'
        t_base.child_frame_id = 'base_footprint'
        t_base.transform.translation.x = float(self.pos[0])
        t_base.transform.translation.y = float(self.pos[1])
        t_base.transform.translation.z = float(self.pos[2])
        t_base.transform.rotation.w = float(self.quat[0])
        t_base.transform.rotation.x = float(self.quat[1])
        t_base.transform.rotation.y = float(self.quat[2])
        t_base.transform.rotation.z = float(self.quat[3])
        self.tf_broadcaster.sendTransform(t_base)

        # 4. Publish Odometry & Pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = 'map'
        pose_stamped.pose.position.x = float(self.pos[0])
        pose_stamped.pose.position.y = float(self.pos[1])
        pose_stamped.pose.position.z = float(self.pos[2])
        pose_stamped.pose.orientation = t_base.transform.rotation
        self.pose_pub.publish(pose_stamped)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose = pose_stamped.pose
        odom_msg.twist.twist.linear.x = float(self.vel[0])
        odom_msg.twist.twist.linear.y = float(self.vel[1])
        odom_msg.twist.twist.linear.z = float(self.vel[2])
        self.odom_pub.publish(odom_msg)

        # 5. Append to Flight Path History
        if len(self.path_msg.poses) == 0 or (
            abs(self.pos[0] - self.path_msg.poses[-1].pose.position.x) > 0.05 or
            abs(self.pos[1] - self.path_msg.poses[-1].pose.position.y) > 0.05 or
            abs(self.pos[2] - self.path_msg.poses[-1].pose.position.z) > 0.05
        ):
            self.path_msg.header.stamp = stamp
            self.path_msg.poses.append(pose_stamped)
            if len(self.path_msg.poses) > 500:
                self.path_msg.poses.pop(0)
            self.path_pub.publish(self.path_msg)

        # 6. Spin 3D Propellers (/joint_states)
        is_spinning = (self.pos[2] > 0.01 or abs(self.cmd_vx) > 0.01 or abs(self.cmd_vy) > 0.01 or
                       abs(self.cmd_vz) > 0.01 or self.hw_armed)
        if is_spinning:
            self.prop_angle = (self.prop_angle + 0.8) % (2.0 * math.pi)

        js = JointState()
        js.header.stamp = stamp
        js.name = ['prop_1_joint', 'prop_2_joint', 'prop_3_joint', 'prop_4_joint']
        js.position = [
            float(self.prop_angle),
            float(-self.prop_angle),
            float(-self.prop_angle),
            float(self.prop_angle)
        ]
        self.joint_pub.publish(js)

    def publish_px4_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mode_pub.publish(msg)

    def publish_px4_trajectory(self):
        # In PX4 NED frame:
        # X is North (Forward), Y is East (Right), Z is Down
        # cmd_vx -> North (+X)
        # cmd_vy -> East (+Y)
        # cmd_vz -> Down (-Z)
        msg = TrajectorySetpoint()
        msg.velocity = [
            float(self.cmd_vx),
            float(self.cmd_vy),
            float(-self.cmd_vz)
        ]
        msg.yaw = float(-self.euler[2])
        msg.yawspeed = float(-self.cmd_yaw_rate)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.traj_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = F450SimEngine()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()
