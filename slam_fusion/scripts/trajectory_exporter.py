#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import os

class TrajectoryExporterNode(Node):
    def __init__(self):
        super().__init__('trajectory_exporter_node')
        
        self.declare_parameter('output_file', 'slam_trajectory.txt')
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        
        # Clear existing file without comment header to strictly match evo's TUM format expectations
        with open(self.output_file, 'w') as f:
            pass
            
        self.sub_odom = self.create_subscription(
            Odometry,
            '/slam/odom',
            self.odom_callback,
            10
        )
        self.get_logger().info(f"Trajectory Exporter started. Writing to {self.output_file}")

    def odom_callback(self, msg: Odometry):
        # Convert ROS timestamp to seconds (float)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        line = f"{t:.6f} {pos.x:.6f} {pos.y:.6f} {pos.z:.6f} {ori.x:.6f} {ori.y:.6f} {ori.z:.6f} {ori.w:.6f}\n"
        
        with open(self.output_file, 'a') as f:
            f.write(line)
            f.flush()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExporterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
