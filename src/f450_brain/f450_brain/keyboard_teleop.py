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

Provides interactive keyboard flight control for both the 3D Digital Twin Simulation
and the Physical Pixhawk 2.4.8 Hardware simultaneously:
- Real-time terminal HUD with live velocity and flight state telemetry
- Full 6-DOF controls: Pitch, Roll, Throttle/Altitude, Yaw Rate
- Direct Offboard Toggle, Force Arm, and Emergency Disarm keys
"""

import os
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleCommand, VehicleStatus

BANNER = """
╔══════════════════════════════════════════════════════════════════════════╗
║              F450 DRONE HYBRID KEYBOARD TELEOP CONTROLLER                ║
║           Author: Naman Harshwal | ROS 2 Humble & PX4 2.4.8              ║
╠══════════════════════════════════════════════════════════════════════════╣
║                                                                          ║
║       [W] : Pitch Forward (+Vx)         [I] : Climb / Altitude Up (+Vz)  ║
║  [A]  [S]  [D] : Roll L / Back / Roll R [K] : Descend (-Vz)              ║
║       [J] : Yaw Turn Left (CCW)         [L] : Yaw Turn Right (CW)        ║
║                                                                          ║
║       [SPACE] : Hover / Brake to 0.0    [O] : Request OFFBOARD Mode      ║
║       [P]     : ARM Vehicle             [X] : EMERGENCY DISARM           ║
║       [+] / [-]: Increase/Decrease Max Speed   [Q] : Exit Teleop         ║
║                                                                          ║
╚══════════════════════════════════════════════════════════════════════════╝
"""

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

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', std_qos)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, px4_qos)

        # Velocities
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.yaw_rate = 0.0

        # Step Sizes
        self.speed_step = 0.5    # m/s
        self.yaw_step = 0.5      # rad/s
        self.max_speed = 3.0

        # Flight State
        self.hw_armed = False
        self.nav_state = 0
        self.mode_str = "STANDBY"

        self.timer = self.create_timer(0.05, self.publish_loop) # 20 Hz
        self.get_logger().info("Keyboard Teleop ready. Listening to keystrokes...")

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

    def publish_loop(self):
        msg = Twist()
        msg.linear.x = float(self.vx)
        msg.linear.y = float(self.vy)
        msg.linear.z = float(self.vz)
        msg.angular.z = float(self.yaw_rate)
        self.cmd_vel_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 191
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(msg)

    def print_hud(self):
        status_line = (
            f" [STATUS] Vx: {self.vx:+4.1f} m/s | Vy: {self.vy:+4.1f} m/s | Vz: {self.vz:+4.1f} m/s | "
            f"YawRate: {self.yaw_rate:+4.1f} rad/s | Armed: {'YES' if self.hw_armed else 'NO '} | Mode: {self.mode_str:<8}"
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

    print(BANNER)
    print(" Press keys to command the drone. Terminal HUD updates in real-time.\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings, timeout=0.04)

            if not key:
                node.print_hud()
                continue

            # Key Handling
            k = key.lower()

            # Linear Motion
            if k == 'w' or key == '\x1b[A':  # Forward / Up Arrow
                node.vx = min(node.max_speed, node.vx + node.speed_step)
            elif k == 's' or key == '\x1b[B':  # Backward / Down Arrow
                node.vx = max(-node.max_speed, node.vx - node.speed_step)
            elif k == 'a' or key == '\x1b[D':  # Roll Left / Left Arrow
                node.vy = max(-node.max_speed, node.vy - node.speed_step)
            elif k == 'd' or key == '\x1b[C':  # Roll Right / Right Arrow
                node.vy = min(node.max_speed, node.vy + node.speed_step)

            # Altitude / Throttle
            elif k == 'i':  # Climb Up
                node.vz = min(node.max_speed, node.vz + node.speed_step)
            elif k == 'k':  # Descend
                node.vz = max(-node.max_speed, node.vz - node.speed_step)

            # Yaw Rotation
            elif k == 'j':  # Turn CCW
                node.yaw_rate = min(2.0, node.yaw_rate + node.yaw_step)
            elif k == 'l':  # Turn CW
                node.yaw_rate = max(-2.0, node.yaw_rate - node.yaw_step)

            # Hover / Brake
            elif key == ' ':
                node.vx = 0.0
                node.vy = 0.0
                node.vz = 0.0
                node.yaw_rate = 0.0

            # Offboard Request
            elif k == 'o':
                print("\n[ACTION] Requesting PX4 OFFBOARD Mode...")
                node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

            # Arming Request
            elif k == 'p':
                print("\n[ACTION] Sending ARM command (Force Arm 21196)...")
                node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)

            # Emergency Disarm
            elif k == 'x':
                print("\n[ACTION] Sending EMERGENCY DISARM command (Force Disarm 21196)...")
                node.vx = 0.0
                node.vy = 0.0
                node.vz = 0.0
                node.yaw_rate = 0.0
                for _ in range(5):
                    node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)

            # Speed Tuning
            elif key in ['+', '=']:
                node.max_speed = min(8.0, node.max_speed + 0.5)
                print(f"\n[CONFIG] Max speed increased to {node.max_speed:.1f} m/s")
            elif key in ['-', '_']:
                node.max_speed = max(0.5, node.max_speed - 0.5)
                print(f"\n[CONFIG] Max speed decreased to {node.max_speed:.1f} m/s")

            # Quit
            elif k == 'q' or key == '\x03':  # 'q' or Ctrl+C
                print("\n\nExiting keyboard teleop...")
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
