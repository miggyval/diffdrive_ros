import rclpy
import rclpy.logging
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from diffdrive_interfaces.msg import WheelSpeeds
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
import cv2 as cv
import numpy as np

SIZE = 600
frame = np.ones((SIZE, SIZE, 3), dtype=np.uint8) * 200

DIAM = 3
LENGTH = 2.0
MAX_SPEED = 2.0
MAX_LENGTH = 100
BORDER_MARGIN = 0.5

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('diffdrive_visualizer')
        self.pub_pose = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.sub_reset = self.create_subscription(Bool, '/reset_sim', self.callback_reset, 10)
        self.create_subscription(WheelSpeeds, '/wheel_speeds', self.wheel_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_l = 0.0
        self.v_r = 0.0
        self.trail = []
        self.xmin, self.xmax = -10, 10
        self.ymin, self.ymax = -10, 10
        
        self.last_motion_time = self.get_clock().now().nanoseconds * 1e-9  # current time in seconds
        self.recenter_delay = 0.5  # seconds after stillness before recenter
        self.motion_threshold = 0.05  # speed threshold to detect movement
        
        self.target_zoom_factor = 1.0
        self.current_zoom_factor = 1.0
        self.base_view_size = self.xmax - self.xmin  # initial world width



        self.start_point, self.end_point = None, None
        cv.namedWindow("Simulation")
        cv.setMouseCallback("Simulation", self.mouse_callback)

        self.get_logger().info("Visualizer started")
        self.run()

    def callback_reset(self, msg):
        if msg.data:
            self.xmin, self.xmax = -10, 10
            self.ymin, self.ymax = -10, 10
            self.trail = []
            self.x, self.y, self.theta = 0.0, 0.0, 0.0
            self.v_l, self.v_r = 0.0, 0.0
            
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.theta = np.arctan2(siny_cosp, cosy_cosp)

    def cmd_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        self.v_l = v - omega
        self.v_r = v + omega
        
        
    def wheel_callback(self, msg):
        v = (msg.left + msg.right) / 2
        omega = (msg.left - msg.right) / LENGTH
        self.v_l = msg.left
        self.v_r = msg.right


    def mouse_callback(self, event, x_pixel, y_pixel, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.start_point = self.p2c(x_pixel, y_pixel)
            self.end_point = None
        elif event == cv.EVENT_LBUTTONUP and self.start_point is not None:
            self.end_point = self.p2c(x_pixel, y_pixel)

    def c2p(self, x, y=None):
        if y is None:
            return int(SIZE * x / (self.xmax - self.xmin))
        return (
            int(SIZE * (x - self.xmin) / (self.xmax - self.xmin)),
            int(SIZE * (1 - (y - self.ymin) / (self.ymax - self.ymin)))
        )

    def p2c(self, px, py):
        return (
            px / SIZE * (self.xmax - self.xmin) + self.xmin,
            (SIZE - 1 - py) / SIZE * (self.ymax - self.ymin) + self.ymin
        )
        
    def recenter_view(self, Kp=0.01):
        """
        Smoothly recenter the view toward the robot position using proportional control.
        Kp: proportional gain (0.01 = slow, 0.1 = fast).
        """
        # Robot position (target)
        target_cx, target_cy = self.x, self.y

        # Current viewport center
        current_cx = (self.xmin + self.xmax) / 2.0
        current_cy = (self.ymin + self.ymax) / 2.0

        # Compute error
        ex = target_cx - current_cx
        ey = target_cy - current_cy

        # Apply proportional control
        new_cx = current_cx + Kp * ex
        new_cy = current_cy + Kp * ey

        # Keep square zoom size
        half_size = (self.xmax - self.xmin) / 2.0
        self.xmin = new_cx - half_size
        self.xmax = new_cx + half_size
        self.ymin = new_cy - half_size
        self.ymax = new_cy + half_size

    def set_target_zoom(self, factor):
        """Adjust the target zoom factor by multiplying it with 'factor'."""
        self.target_zoom_factor *= factor
        # Clamp zoom so it doesn't go too far
        self.target_zoom_factor = np.clip(self.target_zoom_factor, 0.2, 5.0)

        
    def apply_zoom(self, alpha=0.2):
        """
        Smoothly interpolate from current zoom to target zoom.
        alpha: smoothing factor (0.1 slow, 0.3 fast)
        """
        # Interpolate zoom factor
        self.current_zoom_factor += alpha * (self.target_zoom_factor - self.current_zoom_factor)

        # Compute new viewport size based on interpolated zoom
        center_x = (self.xmin + self.xmax) / 2.0
        center_y = (self.ymin + self.ymax) / 2.0
        half_size = (self.base_view_size * self.current_zoom_factor) / 2.0

        self.xmin = center_x - half_size
        self.xmax = center_x + half_size
        self.ymin = center_y - half_size
        self.ymax = center_y + half_size


    def draw_barrier(self, img):
        outer_rect = [self.c2p(self.xmin, self.ymax), self.c2p(self.xmax, self.ymin)]
        inner_rect = [
            self.c2p(self.xmin + BORDER_MARGIN, self.ymax - BORDER_MARGIN),
            self.c2p(self.xmax - BORDER_MARGIN, self.ymin + BORDER_MARGIN)
        ]
        cv.rectangle(img, outer_rect[0], outer_rect[1], (255, 255, 255), -1)
        cv.rectangle(img, inner_rect[0], inner_rect[1], (200, 200, 200), -1)
        return img
    
    def draw_grid(self, img, spacing=1.0):
        # compute first/last grid lines at multiples of 'spacing'
        x_start = np.floor(self.xmin / spacing) * spacing
        x_end   = np.ceil (self.xmax / spacing) * spacing
        y_start = np.floor(self.ymin / spacing) * spacing
        y_end   = np.ceil (self.ymax / spacing) * spacing

        # vertical lines
        x = x_start
        while x <= x_end:
            pt1 = self.c2p(x, self.ymin)
            pt2 = self.c2p(x, self.ymax)
            cv.line(img, pt1, pt2, (220, 220, 220), 1, lineType=cv.LINE_AA)
            x += spacing

        # horizontal lines
        y = y_start
        while y <= y_end:
            pt1 = self.c2p(self.xmin, y)
            pt2 = self.c2p(self.xmax, y)
            cv.line(img, pt1, pt2, (220, 220, 220), 1, lineType=cv.LINE_AA)
            y += spacing

        # axes
        if self.ymin <= 0 <= self.ymax:
            cv.line(img, self.c2p(self.xmin, 0), self.c2p(self.xmax, 0),
                    (0, 0, 0), 2, lineType=cv.LINE_AA)
        if self.xmin <= 0 <= self.xmax:
            cv.line(img, self.c2p(0, self.ymin), self.c2p(0, self.ymax),
                    (0, 0, 0), 2, lineType=cv.LINE_AA)

        return img


    def draw_trail(self, img):
        if len(self.trail) > 1:
            for i in range(1, len(self.trail)):
                pt1 = self.c2p(*self.trail[i - 1])
                pt2 = self.c2p(*self.trail[i])
                cv.line(img, pt1, pt2, (255, 255, 0), 2, lineType=cv.LINE_AA)

    def draw_robot(self, img):
        x, y, theta = self.x, self.y, self.theta
        body = 1.5
        wheel_radius = 0.3

        cv.circle(img, self.c2p(x, y), self.c2p(DIAM / 2), (80, 80, 80), -1, lineType=cv.LINE_AA)
        cv.circle(img, self.c2p(x, y), self.c2p(DIAM / 2), (0, 0, 0), 1, lineType=cv.LINE_AA)

        # Body rectangle
        pts = np.array([
            self.c2p(x - body / 2 * np.cos(theta) + body / 2 * np.sin(theta), y - body / 2 * np.sin(theta) - body / 2 * np.cos(theta)),
            self.c2p(x - body / 2 * np.cos(theta) - body / 2 * np.sin(theta), y - body / 2 * np.sin(theta) + body / 2 * np.cos(theta)),
            self.c2p(x + body / 2 * np.cos(theta) - body / 2 * np.sin(theta), y + body / 2 * np.sin(theta) + body / 2 * np.cos(theta)),
            self.c2p(x + body / 2 * np.cos(theta) + body / 2 * np.sin(theta), y + body / 2 * np.sin(theta) - body / 2 * np.cos(theta))
        ], dtype=np.int32)
        cv.fillPoly(img, [pts], (23, 23, 23), lineType=cv.LINE_AA)
        cv.polylines(img, [pts], True, (120, 120, 120), 1, lineType=cv.LINE_AA)

        # Heading indicator
        front_dir = self.c2p(x - DIAM / 3 * np.sin(theta), y + DIAM / 3 * np.cos(theta))
        front_ind = self.c2p(x - DIAM / 2 * np.sin(theta), y + DIAM / 2 * np.cos(theta))
        cv.line(img, front_ind, front_dir, (0, 255, 255), 1, lineType=cv.LINE_AA)

        # Velocity arrows for wheels
        vel_scale = 1.0
        def get_color(v):
            norm = min(abs(v) / MAX_SPEED, 1.0)
            return (0, int(255 * (1 - norm)), int(255 * norm))

        for (cx, cy, v) in [
            (x + 0.5 * LENGTH * np.cos(theta), y + 0.5 * LENGTH * np.sin(theta), self.v_r),
            (x - 0.5 * LENGTH * np.cos(theta), y - 0.5 * LENGTH * np.sin(theta), self.v_l)
        ]:
            dx = -vel_scale * v * np.sin(theta)
            dy = vel_scale * v * np.cos(theta)
            cv.arrowedLine(img, self.c2p(cx, cy), self.c2p(cx + dx, cy + dy), get_color(v), 2, line_type=cv.LINE_AA, tipLength=0.3)

    def draw_unit_arrow(self, img, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        length = np.hypot(dx, dy)
        if length < 1e-6:
            return img  # Ignore tiny drags
        dx /= length
        dy /= length
        arrow_end = (x1 + dx, y1 + dy)  # Unit length
        img = cv.circle(img, self.c2p(x1, y1), 10, (0, 0, 255), -1, lineType=cv.LINE_AA)
        img = cv.arrowedLine(
            img,
            self.c2p(x1, y1),
            self.c2p(*arrow_end),
            (255, 255, 255),
            8,
            tipLength=0.2,
            line_type=cv.LINE_AA
        )
        img = cv.arrowedLine(
            img,
            self.c2p(x1, y1),
            self.c2p(*arrow_end),
            (0, 20, 0),
            3,
            tipLength=0.2,
            line_type=cv.LINE_AA
        )
        
        return img
    def compute_dynamic_gain(self, still_time, max_gain=0.05, ramp_time=2.0):
        """
        Smoothly ramp Kp from 0 to max_gain using a sigmoid after stillness begins.
        still_time: time since last motion (in seconds)
        max_gain: target proportional gain
        ramp_time: duration to reach ~90% of max_gain
        """
        # Time after delay
        t = still_time - self.recenter_delay
        if t <= 0:
            return 0.0
        # Sigmoid parameters
        a = 6 / ramp_time  # Steepness (6 gives ~90% in ramp_time)
        # Sigmoid from 0 to 1
        s = 1 / (1 + np.exp(-a * (t - ramp_time / 2)))
        return max_gain * s


    def run(self):
        self.zoom_step = 0.2
        rate = self.create_rate(20)
        while rclpy.ok():
            view_size = (self.xmax - self.xmin)  # same as y-range since we enforce square
            buffer = 0.2 * view_size  # 20% of view size
            
            k = 0.05
            if self.x > self.xmax - buffer:
                self.xmax += k * (self.x - self.xmax + buffer)
                self.xmin += k * (self.x - self.xmax + buffer)
            if self.x < self.xmin + buffer:
                self.xmax += k * (self.x - self.xmin - buffer)
                self.xmin += k * (self.x - self.xmin - buffer)
            if self.y > self.ymax - buffer:
                self.ymax += k * (self.y - self.ymax + buffer)
                self.ymin += k * (self.y - self.ymax + buffer)
            if self.y < self.ymin + buffer:
                self.ymax += k * (self.y - self.ymin - buffer)
                self.ymin += k * (self.y - self.ymin - buffer)
                
                # Detect movement
            speed = abs((self.v_r + self.v_l) / 2.0)  # linear approx
            omega = abs((self.v_r - self.v_l) / LENGTH)
            if speed > self.motion_threshold or omega > 0.02:  # moving
                self.last_motion_time = self.get_clock().now().nanoseconds * 1e-9
                
            current_time = self.get_clock().now().nanoseconds * 1e-9
            still_time = current_time - self.last_motion_time

            if still_time > self.recenter_delay:
                Kp = self.compute_dynamic_gain(still_time, max_gain=0.05, ramp_time=1.5)
                self.recenter_view(Kp)


                
            img = frame.copy()
            #img = self.draw_barrier(img)
            
            self.apply_zoom(alpha=0.1)
            self.draw_grid(img, spacing=1.0)
            self.draw_trail(img)
            self.draw_robot(img)
            
            if self.start_point and self.end_point:
                self.draw_unit_arrow(img, self.start_point, self.end_point)
                
                # Compute pose and publish
                x1, y1 = self.start_point
                x2, y2 = self.end_point
                dx = x2 - x1
                dy = y2 - y1
                yaw = np.arctan2(dy, dx)  # Note: forward direction is along +Y in our sim
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "odom"
                pose_msg.pose.position.x = x1
                pose_msg.pose.position.y = y1
                pose_msg.pose.position.z = 0.0
                # Convert yaw to quaternion
                qz = np.sin(yaw / 2.0)
                qw = np.cos(yaw / 2.0)
                pose_msg.pose.orientation.z = qz
                pose_msg.pose.orientation.w = qw
                self.pub_pose.publish(pose_msg)

            # Predict dashed trajectory
            x_pred, y_pred, theta_pred = self.x, self.y, self.theta
            points = []
            for _ in range(100):
                v = (self.v_r + self.v_l) / 2.0
                omega = (self.v_r - self.v_l) / LENGTH
                x_pred += -v * np.sin(theta_pred) * 0.05
                y_pred += v * np.cos(theta_pred) * 0.05
                theta_pred += omega * 0.05
                points.append(self.c2p(x_pred, y_pred))
            for i in range(0, len(points) - 1, 5):
                cv.line(img, points[i], points[i + 1], (255, 0, 255), 2, lineType=cv.LINE_AA)

            self.trail.append((self.x, self.y))
            if len(self.trail) > MAX_LENGTH:
                self.trail.pop(0)

            cv.imshow("Simulation", img)
            key = cv.waitKey(1)
            if key == 27 or key == ord('q'):
                break
            if key == ord('+') or key == ord('='):  # Zoom in
                self.set_target_zoom(1 - self.zoom_step)
            elif key == ord('-') or key == ord('_'):  # Zoom out
                self.set_target_zoom(1 + self.zoom_step)

            rclpy.spin_once(self, timeout_sec=0.01)

def main():
    rclpy.init()
    node = VisualizerNode()
    node.destroy_node()
    rclpy.shutdown()
