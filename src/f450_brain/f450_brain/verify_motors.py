#!/usr/bin/env python3
"""
F450 Motor Verification Test Node
Sequentially tilts attitude to isolate and verify each quadcopter motor:
1. Front Right
2. Front Left
3. Rear Left
4. Rear Right

⚠️ SAFETY NOTICE:
Always REMOVE PROPELLERS before bench testing!
"""

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleAttitudeSetpoint

def get_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*cy - sr*sp*cy
    ]

class MotorVerificationTest(Node):
    def __init__(self):
        super().__init__('verify_motors')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.att_pub = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.counter = 0
        self.target_q = [1.0, 0.0, 0.0, 0.0]
        self.get_logger().info("Motor Verification Test Node started.")

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_attitude_setpoint()
        if self.counter == 50:
            self.get_logger().info("Requesting OFFBOARD Mode...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif self.counter == 100:
            self.get_logger().info("ARMED: Holding Level (All motors idle)")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        elif self.counter == 200:
            self.get_logger().info("1. ISOLATING: [FRONT RIGHT] motor!")
            self.target_q = get_quaternion(-30.0, 30.0, 30.0)
        elif self.counter == 450:
            self.get_logger().info("2. ISOLATING: [FRONT LEFT] motor!")
            self.target_q = get_quaternion(30.0, 30.0, -30.0)
        elif self.counter == 700:
            self.get_logger().info("3. ISOLATING: [REAR LEFT] motor!")
            self.target_q = get_quaternion(30.0, -30.0, 30.0)
        elif self.counter == 950:
            self.get_logger().info("4. ISOLATING: [REAR RIGHT] motor!")
            self.target_q = get_quaternion(-30.0, -30.0, -30.0)
        elif self.counter == 1200:
            self.get_logger().info("TEST COMPLETE. Disarming...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            raise SystemExit
        self.counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position, msg.velocity, msg.acceleration = False, False, False
        msg.attitude, msg.body_rate = True, False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mode_pub.publish(msg)

    def publish_attitude_setpoint(self):
        msg = VehicleAttitudeSetpoint()
        msg.q_d = self.target_q
        msg.thrust_body = [0.0, 0.0, -0.25]
        msg.yaw_sp_move_rate = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.att_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1, msg.param2 = float(param1), float(param2)
        msg.command = command
        msg.target_system, msg.target_component = 1, 1
        msg.source_system, msg.source_component = 255, 191
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotorVerificationTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        try:
            node.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            time.sleep(0.1)
        except Exception:
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
