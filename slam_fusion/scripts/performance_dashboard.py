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
Real-time SLAM Performance Dashboard — ROS 2 Humble
────────────────────────────────────────────────────────────────────────────
Subscribes to latency and consistency topics and renders a live interactive
dashboard using Matplotlib and PyQt6.
"""
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import collections
import time

class DashboardNode(Node):
    def __init__(self):
        super().__init__('performance_dashboard')
        
        self.latency_sub = self.create_subscription(
            Float32, '/sam2/inference_latency_ms', self.latency_cb, 10)
        self.consistency_sub = self.create_subscription(
            Float32, '/metrics/semantic_consistency', self.consistency_cb, 10)
        
        self.times = collections.deque([0.0]*100, maxlen=100)
        self.latencies = collections.deque([0.0]*100, maxlen=100)
        self.fps_data = collections.deque([0.0]*100, maxlen=100)
        
        self.cons_times = collections.deque([0.0]*100, maxlen=100)
        self.consistencies = collections.deque([0.0]*100, maxlen=100)
        
        self.last_frame_time = None
        self.start_time = time.time()

    def latency_cb(self, msg):
        current_time = time.time()
        t = current_time - self.start_time
        
        if self.last_frame_time is not None:
            dt = current_time - self.last_frame_time
            fps = 1.0 / dt if dt > 0 else 0.0
        else:
            fps = 0.0
            
        self.last_frame_time = current_time
        
        self.times.append(t)
        self.latencies.append(msg.data)
        self.fps_data.append(fps)

    def consistency_cb(self, msg):
        t = time.time() - self.start_time
        self.cons_times.append(t)
        self.consistencies.append(msg.data * 100.0)

def main():
    rclpy.init()
    node = DashboardNode()
    
    # Run ROS 2 spin in a background thread so it doesn't block the GUI
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    # Set up dark mode Matplotlib figure
    plt.style.use('dark_background')
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 10))
    fig.canvas.manager.set_window_title('Real-time SLAM Performance Dashboard')
    
    # 1. Latency Plot
    line_lat, = ax1.plot([], [], color='cyan', lw=2)
    ax1.set_title('Inference Latency (ms)', fontsize=12, fontweight='bold')
    ax1.set_xlim(0, 10)
    ax1.set_ylim(0, 500)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylabel('ms')
    
    # 2. FPS Plot
    line_fps, = ax2.plot([], [], color='lime', lw=2)
    ax2.set_title('Processing Frame Rate (FPS)', fontsize=12, fontweight='bold')
    ax2.set_xlim(0, 10)
    ax2.set_ylim(0, 30)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylabel('FPS')
    
    # 3. Consistency Plot
    line_cons, = ax3.plot([], [], color='magenta', lw=2)
    ax3.set_title('Semantic Consistency (%)', fontsize=12, fontweight='bold')
    ax3.set_xlim(0, 10)
    ax3.set_ylim(0, 105)
    ax3.grid(True, alpha=0.3)
    ax3.set_ylabel('%')
    ax3.set_xlabel('Time (s)')
    
    plt.tight_layout()
    
    def update(frame):
        # Update Latency and FPS
        if len(node.times) > 0 and node.times[-1] > 0.0:
            current_t = node.times[-1]
            ax1.set_xlim(max(0, current_t - 10), max(10, current_t))
            ax2.set_xlim(max(0, current_t - 10), max(10, current_t))
            
            line_lat.set_data(node.times, node.latencies)
            max_lat = max(node.latencies) if max(node.latencies) > 100 else 100
            ax1.set_ylim(0, max_lat * 1.2)
            
            line_fps.set_data(node.times, node.fps_data)
            max_fps = max(node.fps_data) if max(node.fps_data) > 10 else 10
            ax2.set_ylim(0, max_fps * 1.2)
            
        # Update Consistency
        if len(node.cons_times) > 0 and node.cons_times[-1] > 0.0:
            current_c = node.cons_times[-1]
            ax3.set_xlim(max(0, current_c - 10), max(10, current_c))
            line_cons.set_data(node.cons_times, node.consistencies)
            
        return line_lat, line_fps, line_cons
        
    ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
