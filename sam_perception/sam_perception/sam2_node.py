#!/usr/bin/env python3
"""
SAM2 Perception Node — ROS 2 Humble
────────────────────────────────────────────────────────────────────────────
Subscribes to an RGB image stream, generates semantic masks via SAM2 at 
configured keyframe intervals, and publishes the instance ID label map.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch

from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator


class Sam2PerceptionNode(Node):
    def __init__(self):
        super().__init__("sam2_perception_node")

        # ROS Parameters
        self.declare_parameter("checkpoint",
                               "/opt/sam2/checkpoints/sam2.1_hiera_small.pt")
        self.declare_parameter("model_cfg",
                               "configs/sam2.1/sam2.1_hiera_s.yaml")
        self.declare_parameter("keyframe_interval", 3)   # process every 3rd frame
        self.declare_parameter("use_half_precision", False)

        checkpoint = self.get_parameter("checkpoint").get_parameter_value().string_value
        model_cfg = self.get_parameter("model_cfg").get_parameter_value().string_value
        kf_interval = self.get_parameter("keyframe_interval").get_parameter_value().integer_value
        use_half = self.get_parameter("use_half_precision").get_parameter_value().bool_value

        # Communication Setup
        self.bridge = CvBridge()
        self._frame_counter = 0
        self._keyframe_interval = max(1, kf_interval)

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # Publish the resulting 2D Semantic Mask
        self.publisher_ = self.create_publisher(Image, '/sam2/semantic_mask', 10)

        # Publish latency diagnostics as a Float32
        self.latency_pub_ = self.create_publisher(Float32, '/sam2/inference_latency_ms', 10)

        # Initialize Model
        self.get_logger().info("Initializing SAM2 Model ...")

        device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {device}")

        try:
            sam2_model = build_sam2(model_cfg, checkpoint, device=device)
        except Exception as e:
            self.get_logger().fatal(f"Failed to load SAM2 model: {e}")
            raise

        # Utilize half-precision for optimized GPU inference
        if use_half and device == "cuda":
            sam2_model = sam2_model.half()
            self.get_logger().info("Using FP16 (half-precision) for faster inference.")

        self.mask_generator = SAM2AutomaticMaskGenerator(
            model=sam2_model,
            points_per_side=16,           # 32 is default; 16 gives ~4× speedup
            pred_iou_thresh=0.86,
            stability_score_thresh=0.92,
        )

        self.get_logger().info("SAM2 Node ready — listening for images.")


    def image_callback(self, msg: Image):
        # Throttle processing based on keyframe interval
        self._frame_counter += 1
        if self._frame_counter % self._keyframe_interval != 0:
            return

        start_time = self.get_clock().now()

        # Convert ROS message to NumPy RGB array
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Execute SAM2 Inference
        label_map = np.zeros(image_rgb.shape[:2], dtype=np.uint8)

        try:
            with torch.no_grad():
                masks = self.mask_generator.generate(image_rgb)
        except Exception as e:
            self.get_logger().error(f"SAM2 inference failed: {e}")
            return

        # Prioritize smaller objects by rendering them over larger masks
        masks_sorted = sorted(masks, key=lambda m: m['area'], reverse=True)

        for i, mask_data in enumerate(masks_sorted):
            object_id = (i % 254) + 1
            boolean_mask = mask_data['segmentation']
            label_map[boolean_mask] = object_id

        # Publish resulting label map
        mask_msg = self.bridge.cv2_to_imgmsg(label_map, encoding="mono8")  # typo fixed
        mask_msg.header = msg.header   # propagate original timestamp for sync
        self.publisher_.publish(mask_msg)

        # Latency diagnostics
        elapsed_ms = (self.get_clock().now() - start_time).nanoseconds / 1e6
        self.latency_pub_.publish(Float32(data=float(elapsed_ms)))
        # Log diagnostic metrics intermittently
        if self._frame_counter % (self._keyframe_interval * 10) == 0:
            self.get_logger().info(
                f"Published mask: {len(masks)} objects | inference {elapsed_ms:.1f} ms "
                f"(frame #{self._frame_counter})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = Sam2PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
