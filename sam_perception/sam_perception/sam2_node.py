#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch

from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

class Sam2PerceptionNode(Node):
    def __init__(self):
        super().__init__("sam2_perception_node")

        #Communication Setup
        self.bridge = CvBridge()

        #Subscribe to your dataset or camera RGB topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        #Publish the resulting 2D Semantic Mask
        self.publisher_ = self.create_publisher(Image, '/sam2/semantic_mask', 10)

        #Model Initialization
        self.get_logger().info("Initializing SAM2 Model ...")

        checkpoint = "./checkpoints/sam2.1_hiera_samll.pt"
        model_cfg = "configs/sam2.1/sam2.1_hiera_s.yaml"

        sam2_model = build_sam2(model_cfg, checkpoint, device="cuda")

        self.mask_generator = SAM2AutomaticMaskGenerator(model=sam2_model)

        self.get_logger().info("SAM2 Node is ready and listening for images!")

    def image_callback(self, msg):

            start_time = self.get_clock().now()

            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            masks = self.mask_generator.generate(image_rgb)

            for i, mask_data in enumerate(masks):

                object_id = (i + 1) % 255
                if object_id == 0: object_id = 1

                boolean_mask = mask_data['segmentation']

                label_map[boolean_mask] = object_id

                mask_msg = self.bridge.cv2_to_imgms(label_map, encoding="mono8")

                self.publisher_.publish(mask_msg)

                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                self.get_logger().info(f"Published mask with {len(masks)} objects. Inference took {elapsed:.2f}s")


def main(args=None):
    rclpy.init(args=args)
    node = Sam2PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
