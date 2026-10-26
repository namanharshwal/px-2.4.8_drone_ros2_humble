#!/usr/bin/env python3
"""
F450 Drone Visualizer Node
Subscribes to PX4 VehicleAttitude, converts NED frame to ENU frame, and broadcasts TF transform 'map' -> 'base_link' as well as publishes '/drone/pose' for real-time RViz2 visualization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleAttitude
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class DroneVisualizer(Node):
    def __init__(self):
        super().__init__('drone_visualizer')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos)
        self.get_logger().info("DroneVisualizer running. Publishing TF 'map' -> 'base_link' and '/drone/pose' for RViz2.")

    def attitude_callback(self, msg):
        now = self.get_clock().now().to_msg()
        # PX4 quaternion is [w, x, y, z] in NED frame. Convert to ENU for RViz:
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = float(msg.q[0])
        t.transform.rotation.x = float(msg.q[1])
        t.transform.rotation.y = float(-msg.q[2])
        t.transform.rotation.z = float(-msg.q[3])
        self.tf_broadcaster.sendTransform(t)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.orientation = t.transform.rotation
        self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = DroneVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
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
