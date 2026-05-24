#!/usr/bin/env python3
"""
DeepLabV3 Perception Node — ROS 2 Humble
────────────────────────────────────────────────────────────────────────────
Subscribes to a raw RGB image stream, runs DeepLabV3 segmentation on
every N-th frame, and publishes a uint8 label map on /sam2/semantic_mask.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from torchvision.models.segmentation import deeplabv3_resnet50, DeepLabV3_ResNet50_Weights
from torchvision import transforms

class DeepLabV3Node(Node):
    def __init__(self):
        super().__init__("deeplabv3_node")

        # Parameters
        self.declare_parameter("keyframe_interval", 3)
        self.declare_parameter("use_half_precision", False)

        # Get Parameter Values
        kf_interval = self.get_parameter("keyframe_interval").get_parameter_value().integer_value
        self.use_half = self.get_parameter("use_half_precision").get_parameter_value().bool_value

        self.bridge = CvBridge()
        self._frame_counter = 0
        self._keyframe_interval = max(1, kf_interval)

        # Publishers & Subscribers
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

        self.publisher_ = self.create_publisher(Image, '/sam2/semantic_mask', 10)
        self.latency_pub_ = self.create_publisher(Float32, '/sam2/inference_latency_ms', 10)

        # Initialize DeeplabV3
        self.get_logger().info("Initializing DeepLabV3 Model ...")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        try:
            # Using API Modern Weights
            weights = DeepLabV3_ResNet50_Weights.DEFAULT
            self.model = deeplabv3_resnet50(weights=weights)
            self.model.to(self.device)
            self.model.eval()

            if self.use_half and self.device == "cuda":
                self.model = self.model.half()
                self.get_logger().info("Using FP16 for faster inference.")

            self.preprocess = transforms.Compose([
                transforms.ToTensor(),
                transforms.Normalize(mean= [0.485, 0.456, 0.406], std = [0.229, 0.224, 0.225]),
            ])

            self.get_logger().info("DeepLabV3 Model Loaded Successfully.")

        except Exception as e:
            self.get_logger().fatal(f"Failed to load DeepLabV3: {e}")
            raise

    def image_callback(self, msg:Image):
        self._frame_counter += 1
        if self._frame_counter % self._keyframe_interval != 0:
            return

        start_time = self.get_clock().now()

        # Convert to Numpy/OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        # DeepLabV3 Inference
        try:
            input_tensor = self.preprocess(image_rgb).unsqueeze(0).to(self.device)

            if self.use_half and self.device == "cuda":
                input_tensor = input_tensor.half()

            with torch.no_grad():
                output = self.model(input_tensor)['out'][0]

            # Extract the class with the highest probability for each pixel
            label_map = output.argmax(0).byte().cpu().numpy()

        except Exception as e:
            self.get_logger().error(f"DeepLabV3 inference failed: {e}")
            return


        # Publish label map
        mask_msg = self.bridge.cv2_to_imgmsg(label_map, encoding="mono8")
        mask_msg.header = msg.header
        self.publisher_.publish(mask_msg)

        # Latency tracking
        elapsed_ms = (self.get_clock().now() - start_time).nanoseconds / 1e6
        self.latency_pub_.publish(Float32(data=float(elapsed_ms)))

        if self._frame_counter % (self._keyframe_interval * 10) == 0:
            self.get_logger().info(f"DeepLabV3 published map | inference {elapsed_ms:.1f} ms")


def main(args=None):
    rclpy.init(args=args)
    node = DeepLabV3Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


