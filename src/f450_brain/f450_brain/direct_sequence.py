#!/usr/bin/env python3
"""
F450 Direct Sequence Motor Test Node
Sends VEHICLE_CMD_DO_MOTOR_TEST commands directly to PX4 to spin each individual ESC/motor at 15% throttle for 3 seconds.

⚠️ SAFETY NOTICE:
Always REMOVE PROPELLERS before bench testing!
"""

import rclpy
import time
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class DirectSequenceTest(Node):
    def __init__(self):
        super().__init__('direct_sequence_test')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.seconds = 0
        self.get_logger().info("Direct Sequence Test initialized. Testing motors 1 to 4...")

    def timer_callback(self):
        self.seconds += 1
        if self.seconds == 2:
            self.get_logger().info("1. Testing FRONT RIGHT (Motor 1) at 15%...")
            self.send_motor_test_cmd(1, 15.0)
        elif self.seconds == 6:
            self.get_logger().info("2. Testing FRONT LEFT (Motor 3) at 15%...")
            self.send_motor_test_cmd(3, 15.0)
        elif self.seconds == 10:
            self.get_logger().info("3. Testing REAR LEFT (Motor 2) at 15%...")
            self.send_motor_test_cmd(2, 15.0)
        elif self.seconds == 14:
            self.get_logger().info("4. Testing REAR RIGHT (Motor 4) at 15%...")
            self.send_motor_test_cmd(4, 15.0)
        elif self.seconds == 18:
            self.get_logger().info("TEST COMPLETE. Shutting down.")
            raise SystemExit

    def send_motor_test_cmd(self, motor_idx, throttle_percent):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_MOTOR_TEST
        msg.param1 = float(motor_idx)
        msg.param2 = 0.0
        msg.param3 = float(throttle_percent)
        msg.param4 = 3.0  # Duration in seconds
        msg.param5 = 1.0  # Motor count
        msg.param6 = 0.0  # Motor order
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 191
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DirectSequenceTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
