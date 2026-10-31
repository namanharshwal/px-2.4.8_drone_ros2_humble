#!/usr/bin/env python3
# Copyright (c) 2025-2026 Naman Harshwal. All Rights Reserved.
# Licensed under Apache 2.0
"""
Hardware Motor Diagnostic & Activation Test
Author: Naman Harshwal
Tests multiple PX4 motor activation pathways:
1. Percentage-based MAV_CMD_DO_MOTOR_TEST (20% throttle)
2. Direct PWM-based MAV_CMD_DO_MOTOR_TEST (1200 us)
3. Offboard Att/Thrust streaming with Force Arm
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus, OffboardControlMode, VehicleAttitudeSetpoint

class MotorHwTest(Node):
    def __init__(self):
        super().__init__('motor_hw_test')

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

    def publish_attitude(self, thrust_val):
        mode_msg = OffboardControlMode()
        mode_msg.position = False
        mode_msg.velocity = False
        mode_msg.acceleration = False
        mode_msg.attitude = True
        mode_msg.body_rate = False
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mode_pub.publish(mode_msg)

        att_msg = VehicleAttitudeSetpoint()
        att_msg.q_d = [1.0, 0.0, 0.0, 0.0]
        att_msg.thrust_body = [0.0, 0.0, -float(thrust_val)]
        att_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.att_pub.publish(att_msg)

def main():
    rclpy.init()
    node = MotorHwTest()

    print("==================================================================")
    print("      PIXHAWK 2.4.8 PHYSICAL MOTOR ACTIVATION DIAGNOSTIC")
    print("==================================================================")

    # 1. Wait for status
    print("[1/4] Waiting for Pixhawk vehicle_status...")
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.status is not None:
            break

    if node.status is None:
        print("[FAIL] No vehicle_status received! Check MicroXRCEAgent.")
        sys.exit(1)

    print(f"  -> Arming State : {node.status.arming_state} (1=STANDBY, 2=ARMED)")
    print(f"  -> Safety Switch: {'OFF / UNLOCKED' if node.status.safety_off else 'LOCKED / BUTTON NOT PRESSED'}")
    print(f"  -> Power Rail   : {'VALID (Flight Battery Detected)' if node.status.power_input_valid else 'INVALID'}")

    # 2. Test Direct Motor Spin via Percentage (Method A)
    print("\n[2/4] Testing MAV_CMD_DO_MOTOR_TEST (Percentage: 20% on Motor 1 for 2s)...")
    for _ in range(5):
        node.send_cmd(
            VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST,
            p1=1.0,  # Motor 1
            p2=0.0,  # Throttle type 0: percentage
            p3=20.0, # 20% throttle
            p4=2.0   # 2 seconds
        )
        time.sleep(0.05)
    print("  -> Sent percentage motor test command.")

    time.sleep(2.5)

    # 3. Test Direct Motor Spin via PWM Microseconds (Method B)
    print("\n[3/4] Testing MAV_CMD_DO_MOTOR_TEST (Direct PWM: 1250us on Motor 1 for 2s)...")
    for _ in range(5):
        node.send_cmd(
            VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST,
            p1=1.0,    # Motor 1
            p2=1.0,    # Throttle type 1: PWM microseconds
            p3=1250.0, # 1250 microseconds PWM
            p4=2.0     # 2 seconds
        )
        time.sleep(0.05)
    print("  -> Sent direct PWM (1250us) motor test command.")

    time.sleep(2.5)

    # 4. Test Offboard Stream + Force Arm with 25% Thrust (Method C)
    print("\n[4/4] Testing 50Hz Offboard Stream + Force Arm with 25% Thrust...")
    print("  -> Streaming setpoints for 1 second...")
    for _ in range(50):
        node.publish_attitude(0.25)
        rclpy.spin_once(node, timeout_sec=0.02)

    print("  -> Engaging OFFBOARD mode and FORCE ARM...")
    node.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
    node.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)

    for i in range(150): # 3 seconds at 25% thrust
        node.publish_attitude(0.25)
        rclpy.spin_once(node, timeout_sec=0.02)
        if i % 50 == 0:
            print(f"     Holding 25% thrust... (arming_state={node.status.arming_state if node.status else '?'})")

    print("  -> Disarming safely...")
    for _ in range(10):
        node.publish_attitude(0.0)
        node.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        rclpy.spin_once(node, timeout_sec=0.02)

    print("\nDiagnostic complete. Check if physical Motor 1 spun during any of the tests.")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
