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
- Real-time terminal HUD with live velocity, throttle, and flight state telemetry
- Full 6-DOF controls: Pitch, Roll, Throttle/Thrust (Motor RPM), Yaw Rate
- Direct Offboard Toggle, Force Arm, and Emergency Disarm keys
- Quick Motor Test key [M] to pulse real ESCs
"""

import os
import sys
import select
import termios
import tty
import time
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
║       [W] : Pitch Forward (Tilt Nose Down) [I] : Throttle UP (+5% Power) ║
║  [A]  [S]  [D] : Roll L / Back / Roll R    [K] : Throttle DOWN (-5% Power║
║       [J] : Yaw Turn Left (CCW)            [L] : Yaw Turn Right (CW)     ║
║                                                                          ║
║       [P] : ARM Drone (15% Idle Motors)    [O] : Switch to OFFBOARD Mode ║
║       [M] : Motor Pulse Test (All 4 ESCs)  [SPACE] : Level & Idle (15%)  ║
║       [X] : EMERGENCY DISARM (Motors STOP) [Q] : Exit Teleop             ║
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

        # Control States
        self.pitch = 0.0        # Radians (+/- 0.40 rad ~ 23 deg)
        self.roll = 0.0         # Radians (+/- 0.40 rad)
        self.throttle = 0.0     # 0.0 to 0.60
        self.yaw_rate = 0.0     # rad/s

        # Flight State from Pixhawk
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
        msg.linear.x = float(self.pitch)
        msg.linear.y = float(self.roll)
        msg.linear.z = float(self.throttle)
        msg.angular.z = float(self.yaw_rate)
        self.cmd_vel_pub.publish(msg)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 191
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(msg)

    def print_hud(self):
        pitch_deg = self.pitch * 57.2958
        roll_deg = self.roll * 57.2958
        thrust_pct = int(self.throttle * 100)
        status_line = (
            f" [STATUS] Throttle: {thrust_pct:2d}% | Pitch: {pitch_deg:+5.1f}° | Roll: {roll_deg:+5.1f}° | "
            f"YawRate: {self.yaw_rate:+4.1f}r/s | Armed: {'YES (SPINNING)' if self.hw_armed else 'NO (IDLE)'} | Mode: {self.mode_str:<8}"
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
    print(" Press keys to fly the drone. Real motors will spin when armed [P]!\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings, timeout=0.04)

            if not key:
                node.print_hud()
                continue

            k = key.lower()

            # Pitch Tilt (Forward / Backward)
            if k == 'w' or key == '\x1b[A':  # Forward (Nose Down)
                node.pitch = max(-0.40, node.pitch - 0.08)
            elif k == 's' or key == '\x1b[B':  # Backward (Nose Up)
                node.pitch = min(0.40, node.pitch + 0.08)

            # Roll Tilt (Left / Right)
            elif k == 'a' or key == '\x1b[D':  # Roll Left
                node.roll = max(-0.40, node.roll - 0.08)
            elif k == 'd' or key == '\x1b[C':  # Roll Right
                node.roll = min(0.40, node.roll + 0.08)

            # Throttle (Real Motor RPM)
            elif k == 'i':  # Throttle UP
                node.throttle = min(0.60, max(0.15, node.throttle + 0.05))
            elif k == 'k':  # Throttle DOWN
                node.throttle = max(0.0, node.throttle - 0.05)

            # Yaw Rotation
            elif k == 'j':  # Turn CCW
                node.yaw_rate = min(2.0, node.yaw_rate + 0.5)
            elif k == 'l':  # Turn CW
                node.yaw_rate = max(-2.0, node.yaw_rate - 0.5)

            # Hover / Brake
            elif key == ' ':
                node.pitch = 0.0
                node.roll = 0.0
                node.yaw_rate = 0.0
                if node.hw_armed:
                    node.throttle = 0.15  # Idle spin when armed
                else:
                    node.throttle = 0.0

            # Offboard Request
            elif k == 'o':
                print("\n[ACTION] Requesting PX4 OFFBOARD Mode...")
                node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

            # Arming Request (ARM + OFFBOARD + 15% Idle Throttle)
            elif k == 'p':
                print("\n[ACTION] Engaging OFFBOARD Mode & ARMING Drone (Motors Start at 15% Idle)...")
                node.throttle = 0.15
                node.pitch = 0.0
                node.roll = 0.0
                node.yaw_rate = 0.0
                # Stream Offboard Mode command then Force Arm command:
                for _ in range(3):
                    node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    time.sleep(0.02)
                for _ in range(3):
                    node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
                    time.sleep(0.02)

            # Direct Motor Pulse Test (All 4 Motors spin for 2 seconds)
            elif k == 'm':
                print("\n[ACTION] Firing Direct Motor Test on all 4 ESCs at 15% throttle for 2 seconds...")
                for motor_idx in [1, 2, 3, 4]:
                    node.send_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST,
                        float(motor_idx),
                        0.0,   # Throttle type 0: percentage
                        15.0,  # 15% throttle
                        2.0    # 2 seconds duration
                    )

            # Emergency Disarm
            elif k == 'x':
                print("\n[ACTION] Sending EMERGENCY DISARM (Motors STOP)...")
                node.pitch = 0.0
                node.roll = 0.0
                node.throttle = 0.0
                node.yaw_rate = 0.0
                for _ in range(5):
                    node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
                    time.sleep(0.02)

            # Quit
            elif k == 'q' or key == '\x03':  # 'q' or Ctrl+C
                print("\n\nStopping teleop & disarming...")
                node.throttle = 0.0
                for _ in range(5):
                    node.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
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
