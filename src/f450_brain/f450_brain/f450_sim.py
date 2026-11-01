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
F450 3D Digital Twin Simulation & Avionics Visualization Engine
Author: Naman Harshwal

Provides a responsive, realistic 6-DOF Digital Twin Simulation with 3D RViz2 visualization:
- Subscribes to /cmd_vel from the keyboard teleop flight controller
- Strict ground clamp: locks horizontal drift to 0 while on the ground (no sliding on the floor!)
- Direct upward lift: throttle > 25% produces clean, immediate vertical climb
- Smooth 6-DOF airborne flight: forward (W), backward (S), left (A), right (D), and yaw turn (J/L)
- Subscribes to live Pixhawk IMU attitude (/fmu/out/vehicle_attitude) and status (/fmu/out/vehicle_status)
- Broadcasts dynamic TF tree: 'map' -> 'odom' -> 'base_footprint'
- Publishes /joint_states for continuous 3D propeller rotation proportional to motor thrust
- Records and displays flight trajectory on /f450/path
- Highly optimized at 20Hz to preserve CPU headroom on Raspberry Pi 4
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

from px4_msgs.msg import VehicleAttitude, VehicleStatus

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (rad) to quaternion [w, x, y, z] in ROS ENU frame."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    w = cr * cp * cy + sr * cp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]

class F450SimEngine(Node):
    def __init__(self):
        super().__init__('f450_sim_engine')

        # PX4 QoS Profile
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Standard ROS 2 QoS Profile
        std_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, std_qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.hw_attitude_callback, px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.hw_status_callback, px4_qos)

        # Simulation & Visualization Publishers
        self.odom_pub = self.create_publisher(Odometry, '/f450/odom', std_qos)
        self.path_pub = self.create_publisher(Path, '/f450/path', std_qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/f450/pose', std_qos)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', std_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Drone State Variables (Simulation)
        self.pos = [0.0, 0.0, 0.0]     # X, Y, Z (meters)
        self.vel = [0.0, 0.0, 0.0]     # Vx, Vy, Vz (m/s)
        self.euler = [0.0, 0.0, 0.0]   # Roll, Pitch, Yaw (radians)
        self.quat = [1.0, 0.0, 0.0, 0.0]

        # Commanded setpoints from keyboard teleop
        self.cmd_pitch = 0.0       # Pitch angle in radians (linear.x)
        self.cmd_roll = 0.0        # Roll angle in radians (linear.y)
        self.cmd_throttle = 0.0    # Direct throttle 0.0 to 1.0 (linear.z)
        self.cmd_yaw_rate = 0.0    # Yaw rate in rad/s (angular.z)

        # Hardware Avionics Sync
        self.hw_attitude_available = False
        self.hw_quat = [1.0, 0.0, 0.0, 0.0]
        self.hw_armed = False

        # Propeller visual rotation animation
        self.prop_angle = 0.0
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # Main 20Hz Simulation Loop (50ms interval) - low CPU on Pi 4
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.sim_loop)
        self.get_logger().info("F450 3D Digital Twin Simulation Engine active at 20Hz.")

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_pitch = msg.linear.x
        self.cmd_roll = msg.linear.y
        self.cmd_throttle = msg.linear.z
        self.cmd_yaw_rate = msg.angular.z

    def hw_attitude_callback(self, msg: VehicleAttitude):
        self.hw_attitude_available = True
        # PX4 quaternion is [w, x, y, z] in NED frame.
        # Convert to ENU for ROS visualization: w=w, x=x, y=-y, z=-z
        self.hw_quat = [float(msg.q[0]), float(msg.q[1]), float(-msg.q[2]), float(-msg.q[3])]

    def hw_status_callback(self, msg: VehicleStatus):
        self.hw_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def sim_loop(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        dt = self.dt

        # 1. Update Heading (Yaw)
        self.euler[2] += self.cmd_yaw_rate * dt
        yaw = self.euler[2]

        # 2. Flight Kinematics & Strict Ground Clamp Logic
        is_armed = (self.hw_armed or self.cmd_throttle >= 0.20)

        if not is_armed or self.cmd_throttle < 0.15:
            # Drone is DISARMED / EMERGENCY STOP
            if self.pos[2] > 0.02:
                # Descending / falling smoothly to ground
                self.vel[2] = max(-2.5, self.vel[2] - 9.81 * dt)
                self.pos[2] = max(0.0, self.pos[2] + self.vel[2] * dt)
            else:
                # Resting firmly on landing gear - zero horizontal drift!
                self.pos[2] = 0.0
                self.vel = [0.0, 0.0, 0.0]
                self.euler[0] = 0.0
                self.euler[1] = 0.0

        elif self.cmd_throttle <= 0.30:
            # Idle throttle on the ground (armed at 30% power)
            if self.pos[2] > 0.02:
                # Smooth landing descent
                self.vel[2] = -0.6
                self.pos[2] = max(0.0, self.pos[2] + self.vel[2] * dt)
            else:
                # Firmly locked on the ground - NO sideways sliding!
                self.pos[2] = 0.0
                self.vel = [0.0, 0.0, 0.0]
                self.euler[0] = 0.0
                self.euler[1] = 0.0

        else:
            # Throttle > 30% (AIRBORNE 3D FLIGHT!)
            # Hover deadband (33% to 37%): holds altitude steadily
            if 0.33 <= self.cmd_throttle <= 0.37:
                self.vel[2] = 0.85 * self.vel[2]
            elif self.cmd_throttle > 0.37:
                # Climb upwards
                target_climb = (self.cmd_throttle - 0.37) * 8.0
                self.vel[2] = 0.80 * self.vel[2] + 0.20 * target_climb
            else:
                # Gentle descent (30% to 33%)
                target_climb = (self.cmd_throttle - 0.33) * 4.0
                self.vel[2] = 0.80 * self.vel[2] + 0.20 * target_climb

            self.pos[2] = max(0.0, self.pos[2] + self.vel[2] * dt)

            # Horizontal velocity is only active when airborne:
            if self.pos[2] > 0.02:
                # In body frame:
                # Pitch forward (cmd_pitch < 0) produces positive forward body velocity
                # Roll right (cmd_roll > 0) produces positive rightward body velocity
                v_body_forward = - (self.cmd_pitch / 0.35) * 2.8
                v_body_right = (self.cmd_roll / 0.35) * 2.8

                # Transform body velocity to world coordinates using yaw heading:
                cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
                v_world_x = v_body_forward * cos_yaw - v_body_right * sin_yaw
                v_world_y = v_body_forward * sin_yaw + v_body_right * cos_yaw

                self.vel[0] = 0.80 * self.vel[0] + 0.20 * v_world_x
                self.vel[1] = 0.80 * self.vel[1] + 0.20 * v_world_y

                self.pos[0] += self.vel[0] * dt
                self.pos[1] += self.vel[1] * dt

                self.euler[0] = self.cmd_roll
                self.euler[1] = self.cmd_pitch
            else:
                self.vel[0] = 0.0
                self.vel[1] = 0.0

        # 3. Attitude Orientation Quaternion
        if self.pos[2] <= 0.02 and self.hw_attitude_available:
            # On bench, mirror the physical Pixhawk IMU
            self.quat = self.hw_quat
        else:
            # In flight, reflect commanded 3D attitude tilt and heading
            self.quat = euler_to_quaternion(self.euler[0], self.euler[1], self.euler[2])

        # 4. Broadcast Dynamic TF tree: map -> odom -> base_footprint
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

        # 5. Publish Odometry & Pose
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

        # 6. Flight Path History
        if len(self.path_msg.poses) == 0 or (
            abs(self.pos[0] - self.path_msg.poses[-1].pose.position.x) > 0.05 or
            abs(self.pos[1] - self.path_msg.poses[-1].pose.position.y) > 0.05 or
            abs(self.pos[2] - self.path_msg.poses[-1].pose.position.z) > 0.05
        ):
            self.path_msg.header.stamp = stamp
            self.path_msg.poses.append(pose_stamped)
            if len(self.path_msg.poses) > 200:
                self.path_msg.poses.pop(0)
            self.path_pub.publish(self.path_msg)

        # 7. Spin 3D Propellers proportional to motor throttle
        is_prop_active = (self.cmd_throttle >= 0.20 or self.hw_armed or self.pos[2] > 0.01)
        if is_prop_active:
            spin_speed = 0.5 + 1.8 * max(0.25, self.cmd_throttle)
            self.prop_angle = (self.prop_angle + spin_speed) % (2.0 * math.pi)

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
