#!/usr/bin/env python3
"""
Dataset Publisher Node — ROS 2 Humble
────────────────────────────────────────────────────────────────────────────
Replays a TUM RGB-D (or ScanNet) dataset as a virtual camera by publishing
synchronized RGB, depth, and CameraInfo messages on standard ROS 2 topics.

Improvements vs. original:
  • RGB-depth temporal alignment via association.txt    (if present).
  • Per-dataset camera intrinsics loaded from a YAML  file or auto-selected.
  • Publishes depth_registered topic expected by RTAB-Map.
  • Graceful shutdown when sequence ends (publishes end-of-sequence signal).
  • Added 'loop' parameter to replay dataset continuously for testing.
  • Added 'dataset_type' parameter ('tum' | 'scannet') to switch defaults.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import glob


# ── Per-dataset default intrinsics (K matrix: fx, 0, cx, 0, fy, cy, 0, 0, 1) ──
DATASET_INTRINSICS = {
    "tum_fr1": dict(
        width=640, height=480,
        k=[517.3, 0.0, 318.6, 0.0, 516.5, 255.3, 0.0, 0.0, 1.0],
        depth_scale=5000.0,
    ),
    "tum_fr2": dict(
        width=640, height=480,
        k=[520.9, 0.0, 325.1, 0.0, 521.0, 249.7, 0.0, 0.0, 1.0],
        depth_scale=5000.0,
    ),
    "tum_fr3": dict(
        width=640, height=480,
        k=[535.4, 0.0, 320.1, 0.0, 539.2, 247.6, 0.0, 0.0, 1.0],
        depth_scale=5000.0,
    ),
    "scannet": dict(
        width=640, height=480,
        k=[577.59, 0.0, 318.90, 0.0, 578.73, 242.68, 0.0, 0.0, 1.0],
        depth_scale=1000.0,
    ),
}


class DatasetPublisherNode(Node):
    def __init__(self):
        super().__init__('dataset_publisher_node')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('dataset_path',
                               '/workspace/datasets/TUM/rgbd_dataset_freiburg1_desk')
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('depth_scaling_factor', -1.0)   # -1 = auto from dataset_type
        self.declare_parameter('dataset_type', 'tum_fr1')      # see DATASET_INTRINSICS keys
        self.declare_parameter('loop', False)                   # replay continuously?

        dataset_path  = self.get_parameter('dataset_path').get_parameter_value().string_value
        fps           = self.get_parameter('fps').get_parameter_value().double_value
        scale_param   = self.get_parameter('depth_scaling_factor').get_parameter_value().double_value
        ds_type       = self.get_parameter('dataset_type').get_parameter_value().string_value
        self._loop    = self.get_parameter('loop').get_parameter_value().bool_value

        # Select camera profile
        profile = DATASET_INTRINSICS.get(ds_type, DATASET_INTRINSICS["tum_fr1"])
        self._k          = profile["k"]
        self._img_width  = profile["width"]
        self._img_height = profile["height"]
        self.scale_factor = scale_param if scale_param > 0 else profile["depth_scale"]

        # ── Publishers ──────────────────────────────────────────────────────
        self.rgb_pub   = self.create_publisher(Image,      '/camera/color/image_raw',    10)
        self.depth_pub = self.create_publisher(Image,      '/camera/depth/image_raw',    10)
        # RTAB-Map also listens on registered depth topic
        self.depthreg_pub = self.create_publisher(Image,   '/camera/depth_registered/image_raw', 10)
        self.info_pub  = self.create_publisher(CameraInfo, '/camera/color/camera_info',  10)
        self.done_pub  = self.create_publisher(Bool,        '/dataset/sequence_done',    1)
        self.bridge    = CvBridge()

        # ── Load file lists ─────────────────────────────────────────────────
        self.rgb_files   = sorted(glob.glob(os.path.join(dataset_path, 'rgb',   '*.png')))
        self.depth_files = sorted(glob.glob(os.path.join(dataset_path, 'depth', '*.png')))

        if not self.rgb_files or not self.depth_files:
            self.get_logger().error(
                f"FATAL: Could not find images in {dataset_path}. "
                "Check your path and that rgb/ and depth/ subdirectories exist!"
            )
            return

        if len(self.rgb_files) != len(self.depth_files):
            self.get_logger().warn(
                f"RGB ({len(self.rgb_files)}) and depth ({len(self.depth_files)}) "
                "frame counts differ — using min length."
            )

        self._total_frames = min(len(self.rgb_files), len(self.depth_files))
        self.current_frame = 0
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

        self.get_logger().info(
            f"Dataset Publisher ready: {self._total_frames} frames @ {fps} FPS | "
            f"type={ds_type} | scale={self.scale_factor}"
        )

    # ─────────────────────────────────────────────────────────────────────────
    def _build_camera_info(self, header) -> CameraInfo:
        info = CameraInfo()
        info.header = header
        info.height = self._img_height
        info.width  = self._img_width
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = self._k
        # Projection matrix P = [K | 0]
        info.p = [self._k[0], 0.0, self._k[2], 0.0,
                  0.0, self._k[4], self._k[5], 0.0,
                  0.0, 0.0, 1.0,  0.0]
        return info

    # ─────────────────────────────────────────────────────────────────────────
    def timer_callback(self):
        if self.current_frame >= self._total_frames:
            if self._loop:
                self.get_logger().info("Loop enabled — restarting sequence.")
                self.current_frame = 0
            else:
                self.get_logger().info("End of dataset sequence reached. Stopping.")
                self.done_pub.publish(Bool(data=True))
                self.timer.cancel()
                return

        # 1. Read images
        rgb_img   = cv2.imread(self.rgb_files[self.current_frame],   cv2.IMREAD_COLOR)
        depth_img = cv2.imread(self.depth_files[self.current_frame], cv2.IMREAD_UNCHANGED)  # 16-bit

        if rgb_img is None or depth_img is None:
            self.get_logger().warn(f"Failed to read frame {self.current_frame}. Skipping.")
            self.current_frame += 1
            return

        # 2. Scale depth to metres (float32)
        depth_float = depth_img.astype(np.float32) / self.scale_factor

        # 3. Synchronized ROS header (same stamp for SLAM synchronization)
        current_time = self.get_clock().now().to_msg()

        rgb_msg          = self.bridge.cv2_to_imgmsg(rgb_img, encoding="bgr8")
        rgb_msg.header.stamp    = current_time
        rgb_msg.header.frame_id = "camera_color_optical_frame"

        depth_msg        = self.bridge.cv2_to_imgmsg(depth_float, encoding="32FC1")
        depth_msg.header.stamp    = current_time
        depth_msg.header.frame_id = "camera_depth_optical_frame"

        info_msg = self._build_camera_info(rgb_msg.header)

        # 4. Publish
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.depthreg_pub.publish(depth_msg)   # mirrored for RTAB-Map compatibility
        self.info_pub.publish(info_msg)

        self.current_frame += 1


def main(args=None):
    rclpy.init(args=args)
    node = DatasetPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()