import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import tf_transformations
import tf2_ros
import numpy as np

class Lidar(Node):
    def __init__(self):
        super().__init__('lidar')
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.callback_map, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.callback_odom, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.map = None
        self.resolution = None
        self.origin = None
        self.width = None
        self.height = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # LiDAR settings
        self.angle_min = -np.pi  # full 360°
        self.angle_max = np.pi
        self.angle_increment = np.radians(1.0)  # 1° per step
        self.range_max = 50.0  # meters
        self.range_min = 0.05  # meters
        self.timer = self.create_timer(0.001, self.publish_scan)  # 10 Hz
        
        
    def cast_ray(self, angle):
        """
        Raycast from robot position in direction 'angle' (world frame)
        """
        step = self.resolution / 2.0  # step size (half a cell)
        total_dist = 0.0
        x0, y0 = self.x, self.y
        theta = self.yaw + angle

        while total_dist < self.range_max:
            # Compute next point
            px = x0 + total_dist * np.cos(theta)
            py = y0 + total_dist * np.sin(theta)

            # Convert to map cell
            mx = int((px - self.origin_x) / self.resolution)
            my = int((py - self.origin_y) / self.resolution)

            if mx < 0 or mx >= self.width or my < 0 or my >= self.height:
                return self.range_max  # out of map bounds

            if self.map[my, mx] > 50:  # occupied cell
                return total_dist + np.random.normal(0.0, 0.05)

            total_dist += step

        return self.range_max
    
    def publish_scan(self):
        if self.map is None:
            return

        ranges = []
        angle = self.angle_min
        while angle <= self.angle_max:
            dist = self.cast_ray(angle)
            ranges.append(dist)
            angle += self.angle_increment

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_link"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges

        self.scan_pub.publish(scan)
            
        if self.map is None:
            return

        ranges = []
        angle = self.angle_min
        while angle <= self.angle_max:
            dist = self.cast_ray(angle)
            ranges.append(dist)
            angle += self.angle_increment

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_link"
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges

        self.scan_pub.publish(scan)
        
    def callback_map(self, msg: OccupancyGrid):
        self.width = msg.info.width
        self.height = msg.info.height
        self.map = np.reshape(msg.data, (self.height, self.width))
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
    
    
    def callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        
        
        

def main():
    rclpy.init()
    node = Lidar()
    rclpy.spin(node)
    rclpy.shutdown()
