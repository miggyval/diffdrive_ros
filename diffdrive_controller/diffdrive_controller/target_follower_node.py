#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from diffdrive_interfaces.msg import WheelSpeeds
import numpy as np
import math

class TargetFollower(Node):
    def __init__(self):
        super().__init__('target_follower')
        
        # Parameters
        self.k_v = 1.0    # Linear gain
        self.k_w = 2.0    # Angular gain
        self.k_theta = 1.0    
        self.max_v = 1.0
        self.max_w = 2.0
        
        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_x = None
        self.target_y = None
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.target_callback, 10)
        
        # Publisher
        self.pub_wheels = self.create_publisher(WheelSpeeds, '/wheel_speeds', 10)
        
        # Timer
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.current_yaw = yaw

    def target_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        q = msg.pose.orientation
        self.target_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def control_loop(self):
        if self.target_x is None:
            return
        
        # Compute error
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance > 0.05:
            heading = -np.arctan2(dx, dy)
            heading_error = heading - self.current_yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
            
            # Control
            v = self.k_v * distance
            w = self.k_w * heading_error
        else:
            v = 0
            yaw_error = self.target_yaw - self.current_yaw - np.pi / 2
            yaw_error = (yaw_error + np.pi) % (2 * np.pi) - np.pi
            w = self.k_theta * yaw_error
        
        # Limit speeds
        v = max(-self.max_v, min(self.max_v, v))
        w = max(-self.max_w, min(self.max_w, w))
        
        # Convert to wheel speeds
        v_l = v - w
        v_r = v + w
        
        self.publish_speeds(v_l, v_r)

    def publish_speeds(self, left, right):
        msg = WheelSpeeds()
        msg.left = float(left)
        msg.right = float(right)
        self.pub_wheels.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TargetFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
