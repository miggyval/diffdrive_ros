import rclpy
from rclpy.node import Node
import pygame
import numpy as np
from diffdrive_interfaces.msg import WheelSpeeds
from std_msgs.msg import Bool


MAX_SPEED = 2.0 * 5
MAX_REVERSE = 0.5 * 5
MAX_ROTATE = 2.0 * 3
LENGTH = 2.0
LEFT_BUMPER, RIGHT_BUMPER, LEFT_TRIGGER, RIGHT_TRIGGER, BUTTON_X = 9, 10, 4, 5, 3

class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')
        self.pub_wheel = self.create_publisher(WheelSpeeds, '/wheel_speeds', 10)
        self.pub_reset = self.create_publisher(Bool, '/reset_sim', 10)

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick found!")
            raise RuntimeError("No joystick found")
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.get_logger().info(f"Using joystick: {self.js.get_name()}")
        self.create_timer(0.05, self.update)
        self.v_l = 0.0
        self.v_r = 0.0


    def get_joystick_drive(self):
        pygame.event.pump()
        axis_forward = -self.js.get_axis(1)
        axis_steer   = -self.js.get_axis(2)
        throttle = np.sign(axis_forward) * axis_forward ** 4
        steering = np.sign(axis_steer) * axis_steer ** 4
            
        weight_v = 0.7
        weight_omega = 2.0
        scale = max(1.0, weight_v * np.abs(throttle) + weight_omega * np.abs(steering))
        
        omega = MAX_ROTATE * steering / scale
        if axis_forward > 0:
            v = MAX_SPEED * throttle / scale
        else:
            v = MAX_REVERSE * throttle / scale
            
        v = v
        v_l = v - omega
        v_r = v + omega
        
        if self.js.get_button(LEFT_BUMPER):  # Left bumper
            v_l = 0.0
        if self.js.get_button(RIGHT_BUMPER):  # Right bumper
            v_r = 0.0

        return v_l, v_r

    def update(self):
        pygame.event.pump()

        if self.js.get_button(BUTTON_X):
            msg = Bool()
            msg.data = True
            self.pub_reset.publish(msg)
            
        v_l_cmd, v_r_cmd = self.get_joystick_drive()
        # Direct Drive
        for i in range(self.js.get_numaxes()):
            val = self.js.get_axis(i)
            if (val + 1.0) / 2.0 < 0.1:
                continue
            if i == LEFT_TRIGGER:
                v_l_cmd = MAX_SPEED * (val + 1.0) / 2.0
                if self.js.get_button(LEFT_BUMPER):
                    v_l_cmd *= -1
            if i == RIGHT_TRIGGER:
                v_r_cmd = MAX_SPEED * (val + 1.0) / 2.0
                if self.js.get_button(RIGHT_BUMPER):
                    v_r_cmd *= -1
                    
        
        dt = 0.1
        tau = 1.0
        self.v_l += (v_l_cmd - self.v_l) * dt / tau
        self.v_r += (v_r_cmd - self.v_r) * dt / tau

        msg = WheelSpeeds()
        msg.left = self.v_l
        msg.right = self.v_r
        self.pub_wheel.publish(msg)


def main():
    rclpy.init()
    node = JoystickController()
    rclpy.spin(node)
    rclpy.shutdown()
