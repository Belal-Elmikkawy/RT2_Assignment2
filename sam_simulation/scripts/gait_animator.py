#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class GaitAnimator(Node):
    def __init__(self):
        super().__init__('gait_animator')
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.joint_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10
        )
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.current_speed = 0.0
        self.phase = 0.0

        # Joints must match the exact order in g1_controllers.yaml:
        # left_hip_pitch, left_knee, left_ankle_pitch, right_hip_pitch, right_knee, right_ankle_pitch, left_shoulder_pitch, left_elbow, right_shoulder_pitch, right_elbow
        self.joints = [0.0] * 10

    def cmd_vel_callback(self, msg):
        # Animate walking when translating OR rotating in place
        linear_mag = abs(msg.linear.x)
        angular_mag = abs(msg.angular.z)
        
        # If rotating in place, simulate a virtual forward speed to trigger the walk cycle
        self.current_speed = msg.linear.x if linear_mag > 0.01 else angular_mag * 0.2
        if linear_mag == 0 and angular_mag == 0:
            self.current_speed = 0.0

    def timer_callback(self):
        if abs(self.current_speed) > 0.01:
            # Increment phase based on speed
            self.phase += self.current_speed * 0.15
        else:
            # Smoothly return to standing pose
            self.phase = 0.0
            for i in range(len(self.joints)):
                self.joints[i] *= 0.8  # Decay back to zero

        if abs(self.current_speed) > 0.01:
            # Calculate human-like sinusoidal gait
            hip_swing = math.sin(self.phase) * 0.5
            shoulder_swing = math.sin(self.phase) * 0.6
            knee_bend_l = abs(math.sin(self.phase)) * 0.5
            knee_bend_r = abs(math.cos(self.phase)) * 0.5

            # Left leg
            self.joints[0] = -hip_swing        # left_hip_pitch
            self.joints[1] = knee_bend_l       # left_knee
            self.joints[2] = hip_swing         # left_ankle_pitch

            # Right leg (opposite phase)
            self.joints[3] = hip_swing         # right_hip_pitch
            self.joints[4] = knee_bend_r       # right_knee
            self.joints[5] = -hip_swing        # right_ankle_pitch

            # Left arm (swings with right leg)
            self.joints[6] = hip_swing         # left_shoulder_pitch
            self.joints[7] = 0.3               # left_elbow (slightly bent)

            # Right arm (swings with left leg)
            self.joints[8] = -hip_swing        # right_shoulder_pitch
            self.joints[9] = 0.3               # right_elbow

        # Publish the joint commands
        msg = Float64MultiArray()
        msg.data = self.joints
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitAnimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
