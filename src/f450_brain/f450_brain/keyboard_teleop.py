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
F450 Full Keyboard Teleoperation Node
Author: Naman Harshwal

Provides direct, high-frequency (50Hz) Offboard Attitude & Thrust control for both
the physical Pixhawk 2.4.8 drone hardware and the 3D Digital Twin Simulation simultaneously:
- Independent background ROS 2 executor thread ensures rock-solid 50Hz offboard heartbeat
- Direct OffboardControlMode (attitude=True) and VehicleAttitudeSetpoint streaming to PX4
- Direct Force-Arming and Emergency Disarm commands
- Auto-Arm on flight keys: pressing [W], [UP ARROW], [I], [SPACE], or [P] immediately starts motors!
- Universal intuitive 6-DOF controls:
    [W] / [S]          : Fly Forward / Backward
    [A] / [D]          : Fly Left / Right
    [I] / [UP ARROW]   : Climb Upwards (Throttle +5%)
    [K] / [DOWN ARROW] : Descend Downwards (Throttle -5%)
    [J] / [L]          : Yaw Turn Left / Right
    [SPACE]            : Auto-Takeoff (to 1.5m) / Level Hover
    [P]                : ARM Drone (25% Idle Motors)
    [M]                : Quick ESC Pulse Test
    [X]                : EMERGENCY DISARM (Motors Stop)
- Real-time terminal HUD showing live avionics telemetry
"""

import math
import os
import sys
import select
import termios
import tty
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleAttitudeSetpoint,
    VehicleCommand,
    VehicleStatus
)

BANNER = """
╔══════════════════════════════════════════════════════════════════════════╗
║              F450 DRONE HYBRID KEYBOARD TELEOP CONTROLLER                ║
║           Author: Naman Harshwal | ROS 2 Humble & PX4 2.4.8              ║
╠══════════════════════════════════════════════════════════════════════════╣
║                                                                          ║
║   [SPACE] : AUTO-TAKEOFF (to 1.5m) / HOVER LEVEL                         ║
║   [I] or [UP ARROW]   : CLIMB UPWARDS (Throttle +5% Power)               ║
║   [K] or [DOWN ARROW] : DESCEND DOWNWARDS (Throttle -5% Power)           ║
║   [W] : Fly Forward   [S] : Fly Backward                                 ║
║   [A] : Fly Left      [D] : Fly Right                                    ║
║   [J] : Turn Left     [L] : Turn Right (Yaw Heading)                     ║
║                                                                          ║
║   [P] : ARM Drone (30% Idle Motors)   [M] : Motor Pulse Test (4 ESCs)    ║
║   [X] : EMERGENCY DISARM (STOP)       [Q] : Land & Exit Teleop           ║
║                                                                          ║
╚══════════════════════════════════════════════════════════════════════════╝
"""

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (rad) to quaternion [w, x, y, z]."""
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
    return [float(w), float(x), float(y), float(z)]

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        std_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers directly driving Pixhawk Hardware mixer
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.att_pub = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', px4_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos)

        # Publisher for RViz Simulation Engine
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', std_qos)

        # Telemetry Subscribers
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, px4_qos)
        self.create_subscription(Odometry, '/f450/odom', self.odom_cb, std_qos)

        # Control States
        self.pitch = 0.0          # Radians (-0.35 to +0.35 rad ~ +/- 20 deg)
        self.roll = 0.0           # Radians (-0.35 to +0.35 rad)
        self.throttle = 0.0       # 0.0 to 0.60
        self.yaw_rate = 0.0       # rad/s

        # Flight State
        self.is_armed_commanded = False
        self.hw_armed = False
        self.nav_state = 0
        self.mode_str = "STANDBY"
        self.sim_altitude = 0.0

        # Dedicated 50Hz Offboard & Attitude Stream (20ms interval)
        self.stream_timer = self.create_timer(0.02, self.offboard_stream_loop)
        self.get_logger().info("Keyboard Teleop ready. 50Hz Offboard Engine active.")

    def status_cb(self, msg: VehicleStatus):
        self.hw_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.mode_str = "OFFBOARD"
        elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
            self.mode_str = "POSCTL"
        elif msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            self.mode_str = "HOLD"
        else:
            self.mode_str = f"MODE_{msg.nav_state}"

    def odom_cb(self, msg: Odometry):
        self.sim_altitude = float(msg.pose.pose.position.z)

    def send_cmd(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0):
        """Send MAVLink / PX4 Vehicle Command with high-priority external source headers."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.param3 = float(p3)
        msg.param4 = float(p4)
        msg.param5 = float(p5)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 191
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def trigger_arm(self, initial_throttle=0.30):
        """Engage Offboard mode and Force-Arm drone so all 4 motors start spinning reliably."""
        self.is_armed_commanded = True
        self.throttle = max(initial_throttle, self.throttle)
        for _ in range(5):
            self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            time.sleep(0.01)
        for _ in range(5):
            self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
            time.sleep(0.01)

    def trigger_disarm(self):
        """Safe disarm and motor stop."""
        self.is_armed_commanded = False
        self.throttle = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw_rate = 0.0
        for _ in range(5):
            self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
            time.sleep(0.01)

    def offboard_stream_loop(self):
        """50Hz continuous offboard heartbeat and attitude setpoint generator."""
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        # 1. Offboard Control Mode (Attitude Mode: works indoors without GPS lock!)
        mode_msg = OffboardControlMode()
        mode_msg.position = False
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = True
        mode_msg.body_rate = False
        mode_msg.timestamp = now_us
        self.mode_pub.publish(mode_msg)

        # 2. Attitude & Motor Thrust Setpoint to PX4
        att_msg = VehicleAttitudeSetpoint()
        q = euler_to_quaternion(self.roll, -self.pitch, 0.0)
        att_msg.q_d = q

        active_thrust = (self.is_armed_commanded or self.hw_armed or self.throttle > 0.05)
        if active_thrust and self.throttle >= 0.20:
            eff_throttle = max(0.30, min(0.60, self.throttle))
            att_msg.thrust_body = [0.0, 0.0, -float(eff_throttle)]
        else:
            att_msg.thrust_body = [0.0, 0.0, 0.0]

        att_msg.yaw_sp_move_rate = float(self.yaw_rate)
        att_msg.timestamp = now_us
        self.att_pub.publish(att_msg)

        # 3. Publish /cmd_vel for RViz Simulation Visualizer
        vel_msg = Twist()
        vel_msg.linear.x = float(self.pitch)
        vel_msg.linear.y = float(self.roll)
        vel_msg.linear.z = float(self.throttle)
        vel_msg.angular.z = float(self.yaw_rate)
        self.cmd_vel_pub.publish(vel_msg)

    def print_hud(self):
        pitch_deg = self.pitch * 57.2958
        roll_deg = self.roll * 57.2958
        thrust_pct = int(self.throttle * 100) if (self.is_armed_commanded or self.hw_armed or self.throttle > 0.05) else 0
        arm_status = "ARMED (SPINNING)" if (self.is_armed_commanded or self.hw_armed or self.throttle >= 0.20) else "DISARMED (IDLE)"
        alt_str = f"{self.sim_altitude:4.1f}m"
        status_line = (
            f" [FLIGHT] Alt: {alt_str} | Throttle: {thrust_pct:2d}% | Pitch: {pitch_deg:+5.1f}° | Roll: {roll_deg:+5.1f}° | "
            f"Motors: {arm_status:<17} | Mode: {self.mode_str:<8}"
        )
        sys.stdout.write(f"\r\033[K{status_line}")
        sys.stdout.flush()

def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':  # Escape sequence (e.g. arrow keys)
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardTeleop()

    # Run the 50Hz ROS 2 stream in an independent background thread
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(BANNER)
    print(" >>> Interactive Teleop Active. Press any flight key or [SPACE] / [P] to FLY! <<<\n")

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.04)

            if not key:
                node.print_hud()
                continue

            k = key.lower()

            # AUTO-TAKEOFF / HOVER / LEVEL : [SPACE]
            if key == ' ':
                if not node.is_armed_commanded and not node.hw_armed:
                    print("\n[ACTION] AUTO-TAKEOFF: Arming Drone & Climbing to 1.5m Hover...")
                    node.trigger_arm(initial_throttle=0.38)
                else:
                    print("\n[ACTION] LEVEL & HOVER: Maintaining Altitude...")
                    node.pitch = 0.0
                    node.roll = 0.0
                    node.yaw_rate = 0.0
                    node.throttle = 0.35  # Stable hover throttle

            # CLIMB UPWARDS (Throttle UP) : [I] or [UP ARROW]
            elif k == 'i' or key == '\x1b[A':
                if not node.is_armed_commanded and not node.hw_armed:
                    print("\n[ACTION] Climbing Upwards -> Auto-Arming Drone...")
                    node.trigger_arm(initial_throttle=0.35)
                else:
                    node.throttle = min(0.60, max(0.25, node.throttle + 0.05))

            # DESCEND DOWNWARDS (Throttle DOWN) : [K] or [DOWN ARROW]
            elif k == 'k' or key == '\x1b[B':
                if node.is_armed_commanded or node.hw_armed:
                    node.throttle = max(0.30, node.throttle - 0.05)
                else:
                    node.throttle = 0.0

            # FLY FORWARD : [W]
            elif k == 'w':
                if not node.is_armed_commanded and not node.hw_armed:
                    print("\n[ACTION] Flying Forward -> Auto-Arming Drone...")
                    node.trigger_arm(initial_throttle=0.35)
                node.pitch = max(-0.35, node.pitch - 0.07)

            # FLY BACKWARD : [S]
            elif k == 's':
                if not node.is_armed_commanded and not node.hw_armed:
                    print("\n[ACTION] Flying Backward -> Auto-Arming Drone...")
                    node.trigger_arm(initial_throttle=0.35)
                node.pitch = min(0.35, node.pitch + 0.07)

            # FLY LEFT : [A]
            elif k == 'a':
                if not node.is_armed_commanded and not node.hw_armed:
                    print("\n[ACTION] Flying Left -> Auto-Arming Drone...")
                    node.trigger_arm(initial_throttle=0.35)
                node.roll = max(-0.35, node.roll - 0.07)

            # FLY RIGHT : [D]
            elif k == 'd':
                if not node.is_armed_commanded and not node.hw_armed:
                    print("\n[ACTION] Flying Right -> Auto-Arming Drone...")
                    node.trigger_arm(initial_throttle=0.35)
                node.roll = min(0.35, node.roll + 0.07)

            # TURN LEFT (Yaw CCW) : [J] or [LEFT ARROW]
            elif k == 'j' or key == '\x1b[D':
                node.yaw_rate = min(1.5, node.yaw_rate + 0.3)

            # TURN RIGHT (Yaw CW) : [L] or [RIGHT ARROW]
            elif k == 'l' or key == '\x1b[C':
                node.yaw_rate = max(-1.5, node.yaw_rate - 0.3)

            # MANUAL ARM (All 4 Motors start spinning at 30% Idle) : [P]
            elif k == 'p':
                print("\n[ACTION] Engaging OFFBOARD Mode & ARMING Drone (All 4 Motors Spinning at 30% Power)...")
                node.trigger_arm(initial_throttle=0.30)

            # OFFBOARD MODE TOGGLE : [O]
            elif k == 'o':
                print("\n[ACTION] Requesting PX4 OFFBOARD Mode...")
                for _ in range(3):
                    node.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    time.sleep(0.01)

            # DIRECT MOTOR PULSE TEST (All 4 Motors spin for 2 seconds) : [M]
            elif k == 'm':
                print("\n[ACTION] Triggering Direct Motor Test on ESCs 1, 2, 3, 4 (25% for 2s)...")
                for motor_idx in [1, 2, 3, 4]:
                    for _ in range(3):
                        node.send_cmd(
                            VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST,
                            p1=float(motor_idx),
                            p2=0.0,   # Throttle type 0: percentage
                            p3=25.0,  # 25% throttle (reliable spin threshold for all ESCs)
                            p4=2.0    # 2 seconds duration
                        )
                        time.sleep(0.01)

            # EMERGENCY DISARM (Motors STOP Immediately) : [X]
            elif k == 'x':
                print("\n[ACTION] EMERGENCY DISARM (Motors STOP)...")
                node.trigger_disarm()

            # QUIT : [Q] or Ctrl+C
            elif k == 'q' or key == '\x03':
                print("\n\nStopping teleop & disarming safely...")
                node.trigger_disarm()
                break

            node.print_hud()

    except Exception as e:
        print(f"\nError in teleop loop: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

if __name__ == '__main__':
    main()
