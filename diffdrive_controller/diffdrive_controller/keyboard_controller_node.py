import rclpy
from rclpy.node import Node
import numpy as np
from diffdrive_interfaces.msg import WheelSpeeds
from std_msgs.msg import Bool
import cv2 as cv
import time

# Constants
MAX_SPEED = 2.0 * 5
MAX_REVERSE = 0.5 * 5
MAX_ROTATE = 2.0 * 3

# OpenCV window
WINDOW_NAME = "Keyboard Controller"


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.pub_wheel = self.create_publisher(WheelSpeeds, '/wheel_speeds', 10)
        self.pub_reset = self.create_publisher(Bool, '/reset_sim', 10)

        self.create_timer(0.05, self.update)
        self.v_l = 0.0
        self.v_r = 0.0

        # Create OpenCV window
        cv.namedWindow(WINDOW_NAME)
        self.display_info()

        # Key states
        self.key_pressed = set()

    def display_info(self):
        img = np.zeros((300, 500, 3), dtype=np.uint8)
        text = [
            "Controls:",
            "W/S: Forward / Reverse",
            "A/D: Turn Left / Right",
            "X: Reset Simulation",
            "< , > : Direct wheel drive",
            "Press Q to quit"
        ]
        y = 30
        for t in text:
            cv.putText(img, t, (20, y), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)
            y += 30
        cv.imshow(WINDOW_NAME, img)

    def get_keyboard_drive(self):
        # Interpret key states
        axis_forward = (1 if ord('w') in self.key_pressed else 0) - (1 if ord('s') in self.key_pressed else 0)
        axis_steer = (1 if ord('a') in self.key_pressed else 0) - (1 if ord('d') in self.key_pressed else 0)

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

        v_l = v - omega
        v_r = v + omega
        return v_l, v_r

    def update(self):
        self.display_info()
        key = cv.waitKey(10) & 0xFF

        if key != 255:  # Key pressed
            if key == ord('q'):
                rclpy.shutdown()
                cv.destroyAllWindows()
                return
            if key == ord('x'):
                msg = Bool()
                msg.data = True
                self.pub_reset.publish(msg)
            self.key_pressed.add(key)
        else:
            # Clear pressed keys that are not being held
            self.key_pressed.clear()

        # Direct wheel control
        v_l_cmd, v_r_cmd = self.get_keyboard_drive()
        if ord(',') in self.key_pressed:  # LESS key
            v_l_cmd = MAX_SPEED
        if ord('.') in self.key_pressed:  # GREATER key
            v_r_cmd = MAX_SPEED

        # Smooth response
        dt = 0.1
        tau = 1.0
        self.v_l += (v_l_cmd - self.v_l) * dt / tau
        self.v_r += (v_r_cmd - self.v_r) * dt / tau

        # Publish wheel speeds
        msg = WheelSpeeds()
        msg.left = self.v_l
        msg.right = self.v_r
        self.pub_wheel.publish(msg)


def main():
    rclpy.init()
    node = KeyboardController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
