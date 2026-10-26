#!/usr/bin/env python3
"""
F450 Quadcopter Drone Commander Node
Publishes offboard control mode and attitude setpoint to PX4 via Micro-XRCE-DDS Agent.

⚠️ SAFETY NOTICE:
Always REMOVE PROPELLERS before bench testing!
"""

import rclpy
import sys
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleAttitudeSetpoint

class DroneCommander(Node):
    def __init__(self):
        super().__init__('drone_commander')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.att_pub = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.counter = 0
        self.get_logger().info("Drone Commander initialized. Starting 50Hz setpoint stream...")

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_attitude_setpoint()
        if self.counter == 50:
            self.get_logger().info("1. Requesting OFFBOARD Mode...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif self.counter == 100:
            self.get_logger().info("2. Firing FORCE ARM Command (30% Thrust)...")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        elif self.counter == 500:
            self.get_logger().info("3. Disarming and shutting down...")
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
        msg.q_d = [1.0, 0.0, 0.0, 0.0]
        msg.thrust_body = [0.0, 0.0, -0.30]
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
        self.vehicle_command_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneCommander()
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
