#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from diffdrive_interfaces.msg import WheelSpeeds
import math
import time
import numpy as np

class FeedforwardTargetFollower(Node):
    def __init__(self):
        super().__init__('target_follower_feedforward')
        
        self.wheel_base = 2.0

        # Motion parameters
        self.v_linear = 1.0    # m/s
        self.v_angular = 0.75   # rad/s

        # Robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Target state
        self.target_x = None
        self.target_y = None
        self.target_yaw = 0.0

        # Motion control
        self.motion_plan = []
        self.motion_state = "IDLE"
        self.step_start_time = None
        self.step_duration = 0.0
        self.direction = 1

        # ROS setup
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.target_callback, 10)
        self.pub_wheels = self.create_publisher(WheelSpeeds, '/wheel_speeds', 10)
        self.create_timer(0.05, self.control_loop)  # 20 Hz

        #self.get_logger().info("Feedforward controller ready.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

    def target_callback(self, msg):
        if self.motion_state != "IDLE":
            return

        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        q = msg.pose.orientation
        self.target_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        self.plan_motion()

    def plan_motion(self):
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        # For +Y forward convention
        angle_to_target = math.atan2(dx, dy)

        # First rotation (align to target direction)
        yaw = self.current_yaw
        rot1 = yaw - angle_to_target
        rot1 = math.atan2(math.sin(rot1), math.cos(rot1))

        # Distance to target
        distance = math.sqrt(dx**2 + dy**2)

        # Final rotation (match target yaw)
        target_yaw_adjusted = -(self.target_yaw - math.pi/2)
        rot2 = angle_to_target - target_yaw_adjusted


        # Create motion plan
        self.motion_plan = [
            ("ROTATE_1", rot1),
            ("TRANSLATE", distance),
            ("ROTATE_2", rot2)
        ]

        # Debug log
        #self.get_logger().info(
        #    f"Current: ({self.current_x:.2f}, {self.current_y:.2f}), "
        #    f"Target: ({self.target_x:.2f}, {self.target_y:.2f})"
        #)
        #self.get_logger().info(
        #    f"Motion plan: Rotate1={math.degrees(rot1):.1f}°, Move={distance:.2f}m, Rotate2={math.degrees(rot2):.1f}°"
        #)

        # Execute first step
        self.execute_next_step()


    def execute_next_step(self):
        if not self.motion_plan:
            self.motion_state = "IDLE"
            self.publish_speeds(0.0, 0.0)
            time.sleep(5.0)  # Pause for 5 seconds
            #self.get_logger().info("Motion complete.")
            return

        step, value = self.motion_plan.pop(0)
        self.motion_state = step

        if step.startswith("ROTATE"):
            self.step_duration = abs(value) / self.v_angular
            self.direction = 1 if value >= 0 else -1
        elif step == "TRANSLATE":
            self.step_duration = abs(value) / self.v_linear
            self.direction = 1 if value >= 0 else -1

        self.step_start_time = time.time()
        #self.get_logger().info(f"Executing {step} for {self.step_duration:.2f}s")

    def control_loop(self):
        if self.motion_state == "IDLE":
            return

        elapsed = time.time() - self.step_start_time

        if elapsed < self.step_duration:
            if self.motion_state.startswith("ROTATE"):
                # Use differential drive formula with wheel base
                v_left = -0.5 * self.wheel_base * self.v_angular * self.direction
                v_right = 0.5 * self.wheel_base * self.v_angular * self.direction
                self.publish_speeds(v_left, v_right)

            elif self.motion_state == "TRANSLATE":
                v = self.v_linear * self.direction
                self.publish_speeds(v, v)
        else:
            self.publish_speeds(0.0, 0.0)
            self.execute_next_step()

    def publish_speeds(self, left, right):
        msg = WheelSpeeds()
        msg.left = float(left)
        msg.right = float(right)
        self.pub_wheels.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FeedforwardTargetFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
