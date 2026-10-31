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
F450 Quadcopter Individual Motor Diagnostic & Isolation Tool
Author: Naman Harshwal

Tests physical ESCs/Motors 1, 2, 3, and 4 individually and together via PX4
MAV_CMD_DO_MOTOR_TEST (Command 209) to pinpoint:
  1. Which specific motor is not spinning (Motor 1, 2, 3, or 4)
  2. Whether the issue is ESC calibration / minimum throttle threshold
  3. Whether the issue is bench IMU attitude mixer bias
  4. Whether there is a wiring / hardware fault on that channel

⚠️ SAFETY NOTICE: PROPELLERS MUST BE REMOVED!
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus, OffboardControlMode, VehicleAttitudeSetpoint

MOTOR_NAMES = {
    1: "Motor 1 (Front-Right, CCW) -> Pixhawk MAIN 1",
    2: "Motor 2 (Rear-Left, CCW)   -> Pixhawk MAIN 2",
    3: "Motor 3 (Front-Left, CW)   -> Pixhawk MAIN 3",
    4: "Motor 4 (Rear-Right, CW)  -> Pixhawk MAIN 4"
}

class MotorDiagnostic(Node):
    def __init__(self):
        super().__init__('motor_diagnostic')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.att_pub = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', px4_qos)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, px4_qos)

        self.status = None

    def status_cb(self, msg):
        self.status = msg

    def send_cmd(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0):
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

    def test_single_motor(self, motor_idx, throttle_pct=25.0, duration_sec=2.5):
        """Pulse a single motor channel directly via MAV_CMD_DO_MOTOR_TEST."""
        print(f"\n >>> Testing {MOTOR_NAMES.get(motor_idx, f'Motor {motor_idx}')} at {throttle_pct:.0f}% for {duration_sec}s...")
        for _ in range(5):
            self.send_cmd(
                VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST,
                p1=float(motor_idx),
                p2=0.0,               # 0: percentage throttle
                p3=float(throttle_pct),
                p4=float(duration_sec)
            )
            time.sleep(0.02)

    def test_all_motors_simultaneous(self, throttle_pct=25.0, duration_sec=2.5):
        """Pulse all 4 motors simultaneously via direct motor test."""
        print(f"\n >>> Pulsing ALL 4 Motors Simultaneously at {throttle_pct:.0f}% for {duration_sec}s...")
        for motor_idx in [1, 2, 3, 4]:
            for _ in range(3):
                self.send_cmd(
                    VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST,
                    p1=float(motor_idx),
                    p2=0.0,
                    p3=float(throttle_pct),
                    p4=float(duration_sec)
                )
                time.sleep(0.01)

def run_diagnostic():
    rclpy.init()
    node = MotorDiagnostic()

    print("==================================================================")
    print("      F450 QUADCOPTER MOTOR DIAGNOSTIC & ISOLATION SUITE          ")
    print("      Author: Naman Harshwal | PX4 2.4.8 & ROS 2 Humble          ")
    print("==================================================================")
    print("⚠️  VERIFY PROPELLERS ARE DETACHED BEFORE PROCEEDING!\n")

    # 1. Check PX4 Connection
    print("[1/3] Checking Pixhawk telemetry connection...")
    for _ in range(25):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.status is not None:
            break

    if node.status is None:
        print("[WARN] Pixhawk vehicle_status not received yet. Testing commands anyway...")
    else:
        print(f"  -> Arming State : {node.status.arming_state} (1=STANDBY, 2=ARMED)")
        print(f"  -> Safety Switch: {'OFF / UNLOCKED' if node.status.safety_off else 'LOCKED'}")

    # 2. Sequential Motor Isolation Test
    print("\n[2/3] Sequential Motor Isolation (Each motor will spin one-by-one):")
    for m in [1, 2, 3, 4]:
        node.test_single_motor(m, throttle_pct=25.0, duration_sec=2.0)
        time.sleep(2.8)

    # 3. Higher Throttle Threshold Test (to detect uncalibrated ESC min PWM)
    print("\n[3/3] Testing All 4 Motors Together at 30% Power (ESC Threshold Check):")
    node.test_all_motors_simultaneous(throttle_pct=30.0, duration_sec=3.0)
    time.sleep(3.5)

    print("\n==================================================================")
    print("DIAGNOSTIC COMPLETE")
    print("Please observe:")
    print(" 1. Which motor did NOT spin? (Motor 1, 2, 3, or 4?)")
    print(" 2. Did that motor spin during the individual 25% pulse or 30% pulse?")
    print("==================================================================")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    run_diagnostic()
