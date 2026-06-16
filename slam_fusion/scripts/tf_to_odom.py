#!/usr/bin/env python3
"""
=============================================================================
@Project : Semantic SLAM Evaluation Framework
@Desc    : Evaluation framework for comparing Visual vs LIDAR SLAM 
           algorithms (ORB-SLAM3, RTAB-Map, Cartographer) augmented 
           with zero-shot semantic segmentation (SAM2 / DeepLabV3).
=============================================================================
"""
"""
TF to Odometry Converter Node — ROS 2 Humble
────────────────────────────────────────────────────────────────────────────
Converts Map to Base Link transforms into standard Odometry messages.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformException, Buffer, TransformListener

class TfToOdom(Node):
    def __init__(self):
        super().__init__('tf_to_odom')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Odometry, '/slam/odom', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("tf_to_odom ready — publishing map→base_link as /slam/odom")

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

        except TransformException:
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z
        msg.pose.pose.orientation = t.transform.rotation

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TfToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
