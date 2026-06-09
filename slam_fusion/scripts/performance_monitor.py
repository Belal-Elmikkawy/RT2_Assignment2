#!/usr/bin/env python3
"""
Performance Monitor Node — ROS 2 Humble
────────────────────────────────────────────────────────────────────────────
Tracks and logs average system latency and inference frame rates.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import collections
import time

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        self.declare_parameter('output_csv', '')
        self.output_csv = self.get_parameter('output_csv').get_parameter_value().string_value
        
        if self.output_csv:
            with open(self.output_csv, 'w') as f:
                f.write("timestamp,avg_latency_ms,fps\n")

        # Subscribe to inference latency metrics
        self.latency_sub = self.create_subscription(
            Float32,
            '/sam2/inference_latency_ms',
            self.latency_callback,
            10
        )

        self.latencies = collections.deque(maxlen=10)
        self.frame_times = collections.deque(maxlen=30)
        self.last_frame_time = time.time()

        # Schedule periodic performance reporting
        self.timer = self.create_timer(1.0, self.report_callback)
        self.get_logger().info("Performance Monitor Started. Waiting for latency messages ...")

    def latency_callback(self, msg):
        self.latencies.append(msg.data)

        current_time = time.time()
        if self.last_frame_time is not None:
            self.frame_times.append(current_time - self.last_frame_time)

        self.last_frame_time = current_time

    def report_callback(self):
        if len(self.latencies) == 0:
            return

        avg_latency = sum(self.latencies) / len(self.latencies)

        if len(self.frame_times) > 0:
            avg_frame_time = sum(self.frame_times) / len(self.frame_times)
            fps = 1.0 / avg_frame_time if avg_frame_time > 0 else 0

        else:
            fps = 0.0

        if self.output_csv:
            with open(self.output_csv, 'a') as f:
                f.write(f"{time.time()},{avg_latency:.2f},{fps:.2f}\n")

        print(f"\r\033[K[Performance] Inference Latency: {avg_latency:6.1f} ms | Actual Rate: {fps:5.1f} FPS", end="", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()