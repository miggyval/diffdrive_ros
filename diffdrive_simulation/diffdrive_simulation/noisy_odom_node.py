#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class NoisyMeasurementPublisher(Node):
    def __init__(self):
        super().__init__('diffdrive_noisy_measurement')
        # Parameters
        self.pos_std = 0.02  # 2 cm noise
        self.yaw_std = 0.01  # 0.01 rad ~ 0.6°
        
        # Sub & Pub
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub_noisy = self.create_publisher(PoseStamped, '/noisy_pose', 10)
    
    def odom_callback(self, msg):
        # Extract true pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)) + np.pi / 2.0
        
        # Add Gaussian noise
        x_noisy = x + np.random.normal(0, self.pos_std)
        y_noisy = y + np.random.normal(0, self.pos_std)
        yaw_noisy = yaw + np.random.normal(0, self.yaw_std)
        
        # Normalize yaw
        yaw_noisy = math.atan2(math.sin(yaw_noisy), math.cos(yaw_noisy))
        
        # Convert yaw back to quaternion
        qz = math.sin(yaw_noisy / 2.0)
        qw = math.cos(yaw_noisy / 2.0)
        
        # Create PoseStamped message
        noisy_msg = PoseStamped()
        noisy_msg.header.stamp = self.get_clock().now().to_msg()
        noisy_msg.header.frame_id = 'odom'
        noisy_msg.pose.position.x = x_noisy
        noisy_msg.pose.position.y = y_noisy
        noisy_msg.pose.position.z = 0.0
        noisy_msg.pose.orientation.z = qz
        noisy_msg.pose.orientation.w = qw
        
        self.pub_noisy.publish(noisy_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NoisyMeasurementPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
