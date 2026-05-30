#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import message_filters
from cv_bridge import CvBridge
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class SemanticEvaluatorNode(Node):
    def __init__(self):
        super().__init__('semantic_evaluator_node')
        
        self.bridge = CvBridge()
        self.latest_info = None
        self.prev_data = None  # Stores (mask, depth, pose) from t-1
        
        # Performance metric publisher
        self.consistency_pub = self.create_publisher(Float32, '/metrics/semantic_consistency', 10)
        
        # Subscribers
        self.info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_cb, 10)
        
        self.sub_mask = message_filters.Subscriber(self, Image, '/sam2/semantic_mask')
        self.sub_depth = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        self.sub_odom = message_filters.Subscriber(self, Odometry, '/slam/odom')
        
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_mask, self.sub_depth, self.sub_odom], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info("Semantic Evaluator Node initialized.")

    def info_cb(self, msg):
        self.latest_info = msg

    def pose_msg_to_matrix(self, odom_msg):
        """Converts nav_msgs/Odometry to 4x4 transformation matrix."""
        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        
        T = np.eye(4)
        T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [p.x, p.y, p.z]
        return T

    def sync_callback(self, mask_msg, depth_msg, odom_msg):
        if self.latest_info is None:
            return

        # Convert to numpy
        mask = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        T_world_curr = self.pose_msg_to_matrix(odom_msg)

        if self.prev_data is not None:
            prev_mask, prev_depth, T_world_prev = self.prev_data
            
            # Compute relative transform from prev camera to current camera
            # T_curr_prev = (T_world_curr)^-1 * T_world_prev
            T_curr_prev = np.linalg.inv(T_world_curr) @ T_world_prev
            
            # Calculate temporal consistency
            consistency_score = self.compute_temporal_consistency(
                prev_mask, prev_depth, mask, T_curr_prev, self.latest_info
            )
            
            self.consistency_pub.publish(Float32(data=float(consistency_score)))
            self.get_logger().info(f"Frame-to-Frame Label Consistency: {consistency_score*100:.1f}%")

        # Store current data for next iteration
        self.prev_data = (mask, depth, T_world_curr)

    def compute_temporal_consistency(self, mask1, depth1, mask2, T_2_1, cam_info):
        """Projects pixels from frame 1 to frame 2 and compares semantic labels."""
        fx, fy = cam_info.k[0], cam_info.k[4]
        cx, cy = cam_info.k[2], cam_info.k[5]
        
        # Get valid depth pixels from frame 1
        v, u = np.where((depth1 > 0.1) & (depth1 < 10.0) & (mask1 > 0))
        z1 = depth1[v, u]
        
        # Backproject to 3D
        x1 = (u - cx) * z1 / fx
        y1 = (v - cy) * z1 / fy
        points_3d_f1 = np.vstack((x1, y1, z1, np.ones_like(z1)))
        
        # Transform to Frame 2
        points_3d_f2 = T_2_1 @ points_3d_f1
        
        # Project back to 2D image plane of Frame 2
        x2, y2, z2 = points_3d_f2[0, :], points_3d_f2[1, :], points_3d_f2[2, :]
        valid_z = z2 > 0.1
        
        u2 = np.round((x2[valid_z] * fx / z2[valid_z]) + cx).astype(int)
        v2 = np.round((y2[valid_z] * fy / z2[valid_z]) + cy).astype(int)
        
        # Find pixels that fall within the image bounds of Frame 2
        h, w = mask2.shape
        valid_bounds = (u2 >= 0) & (u2 < w) & (v2 >= 0) & (v2 < h)
        
        u2_valid = u2[valid_bounds]
        v2_valid = v2[valid_bounds]
        
        # Compare labels
        labels_f1 = mask1[v[valid_z][valid_bounds], u[valid_z][valid_bounds]]
        labels_f2 = mask2[v2_valid, u2_valid]
        
        # Ignore background in frame 2
        valid_comparisons = labels_f2 > 0
        if not np.any(valid_comparisons):
            return 0.0
            
        matches = (labels_f1[valid_comparisons] == labels_f2[valid_comparisons])
        consistency = np.sum(matches) / len(matches)
        
        return consistency

def main(args=None):
    rclpy.init(args=args)
    node = SemanticEvaluatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
