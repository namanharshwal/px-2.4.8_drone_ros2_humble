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
- Keystroke-driven 6-DOF flight: Pitch, Roll, Throttle (Motor RPM), Yaw Rate
- Quick Motor Test key [M] to pulse physical ESCs directly
- Real-time terminal HUD showing live Pixhawk avionics telemetry
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
║       [W] : Pitch Forward (Tilt Nose Down) [I] : Throttle UP (+5% Power) ║
║  [A]  [S]  [D] : Roll L / Back / Roll R    [K] : Throttle DOWN (-5% Power║
║       [J] : Yaw Turn Left (CCW)            [L] : Yaw Turn Right (CW)     ║
║                                                                          ║
║       [P] : ARM Drone (25% Idle Motors)    [O] : Switch to OFFBOARD Mode ║
║       [M] : Motor Pulse Test (All 4 ESCs)  [SPACE] : Level & Idle (25%)  ║
║       [X] : EMERGENCY DISARM (Motors STOP) [Q] : Exit Teleop             ║
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

        # Telemetry Subscriber from Pixhawk
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, px4_qos)

        # Control States
        self.pitch = 0.0          # Radians (-0.35 to +0.35 rad ~ +/- 20 deg)
        self.roll = 0.0           # Radians (-0.35 to +0.35 rad)
        self.throttle = 0.0       # 0.0 to 0.60 (25% default on arm)
        self.yaw_rate = 0.0       # rad/s

        # Flight State
        self.is_armed_commanded = False
        self.hw_armed = False
        self.nav_state = 0
        self.mode_str = "STANDBY"

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

        # 2. Attitude & Motor Thrust Setpoint
        att_msg = VehicleAttitudeSetpoint()
        # In PX4 NED: roll is roll, nose-down (forward) is negative pitch, yaw is CW
        q = euler_to_quaternion(self.roll, -self.pitch, 0.0)
        att_msg.q_d = q

        # Apply upward motor thrust if armed:
        if self.hw_armed or self.is_armed_commanded or self.throttle > 0.05:
            eff_throttle = max(0.20, min(0.60, self.throttle))
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
        vel_msg.linear.z = float(self.throttle if (self.hw_armed or self.is_armed_commanded) else 0.0)
        vel_msg.angular.z = float(self.yaw_rate)
        self.cmd_vel_pub.publish(vel_msg)

    def print_hud(self):
        pitch_deg = self.pitch * 57.2958
        roll_deg = self.roll * 57.2958
        thrust_pct = int(self.throttle * 100) if (self.hw_armed or self.is_armed_commanded) else 0
        arm_status = "YES (ARMED & SPINNING)" if (self.hw_armed or self.is_armed_commanded) else "NO (DISARMED)"
        status_line = (
            f" [STATUS] Motors: {thrust_pct:2d}% | Pitch: {pitch_deg:+5.1f}° | Roll: {roll_deg:+5.1f}° | "
            f"YawRate: {self.yaw_rate:+4.1f}r/s | Armed: {arm_status:<22} | Mode: {self.mode_str:<8}"
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
    print(" >>> Interactive Teleop Active. Press keys to control drone. [P] to ARM & SPIN MOTORS! <<<\n")

    try:
        while rclpy.ok():
            key = get_key(settings, timeout=0.04)

            if not key:
                node.print_hud()
                continue

            k = key.lower()

            # Pitch Tilt (Forward / Backward)
            if k == 'w' or key == '\x1b[A':  # Forward (Nose Down)
                node.pitch = max(-0.35, node.pitch - 0.07)
            elif k == 's' or key == '\x1b[B':  # Backward (Nose Up)
                node.pitch = min(0.35, node.pitch + 0.07)

            # Roll Tilt (Left / Right)
            elif k == 'a' or key == '\x1b[D':  # Roll Left
                node.roll = max(-0.35, node.roll - 0.07)
            elif k == 'd' or key == '\x1b[C':  # Roll Right
                node.roll = min(0.35, node.roll + 0.07)

            # Throttle (Real Motor RPM)
            elif k == 'i':  # Throttle UP
                node.throttle = min(0.60, max(0.20, node.throttle + 0.05))
            elif k == 'k':  # Throttle DOWN
                if node.is_armed_commanded or node.hw_armed:
                    node.throttle = max(0.20, node.throttle - 0.05)
                else:
                    node.throttle = 0.0

            # Yaw Rotation
            elif k == 'j':  # Turn CCW
                node.yaw_rate = min(1.5, node.yaw_rate + 0.3)
            elif k == 'l':  # Turn CW
                node.yaw_rate = max(-1.5, node.yaw_rate - 0.3)

            # Level & Idle
            elif key == ' ':
                node.pitch = 0.0
                node.roll = 0.0
                node.yaw_rate = 0.0
                if node.is_armed_commanded or node.hw_armed:
                    node.throttle = 0.25  # 25% Idle spin when armed
                else:
                    node.throttle = 0.0

            # Offboard Request
            elif k == 'o':
                print("\n[ACTION] Requesting PX4 OFFBOARD Mode...")
                for _ in range(3):
                    node.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    time.sleep(0.01)

            # Arming Request (ARM + OFFBOARD + 25% Power Motors Spin)
            elif k == 'p':
                print("\n[ACTION] Engaging OFFBOARD Mode & ARMING Drone (Real Motors Spinning at 25% Power)...")
                node.is_armed_commanded = True
                node.throttle = 0.25
                node.pitch = 0.0
                node.roll = 0.0
                node.yaw_rate = 0.0
                for _ in range(5):
                    node.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    time.sleep(0.01)
                for _ in range(5):
                    node.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
                    time.sleep(0.01)

            # Direct Motor Pulse Test (All 4 Motors spin for 2 seconds)
            elif k == 'm':
                print("\n[ACTION] Triggering Direct Motor Test on ESCs 1, 2, 3, 4 (20% for 2s)...")
                for motor_idx in [1, 2, 3, 4]:
                    for _ in range(3):
                        node.send_cmd(
                            VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST,
                            p1=float(motor_idx),
                            p2=0.0,   # Throttle type 0: percentage
                            p3=20.0,  # 20% throttle
                            p4=2.0    # 2 seconds duration
                        )
                        time.sleep(0.01)

            # Emergency Disarm
            elif k == 'x':
                print("\n[ACTION] EMERGENCY DISARM (Motors STOP)...")
                node.is_armed_commanded = False
                node.throttle = 0.0
                node.pitch = 0.0
                node.roll = 0.0
                node.yaw_rate = 0.0
                for _ in range(5):
                    node.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                    time.sleep(0.01)

            # Quit
            elif k == 'q' or key == '\x03':  # 'q' or Ctrl+C
                print("\n\nStopping teleop & disarming safely...")
                node.is_armed_commanded = False
                node.throttle = 0.0
                for _ in range(5):
                    node.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                    time.sleep(0.01)
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
