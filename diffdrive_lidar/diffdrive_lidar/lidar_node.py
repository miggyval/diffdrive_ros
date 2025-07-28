import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import tf_transformations
import tf2_ros
import numpy as np

'''
# This represents a 2-D grid map

std_msgs/Header header

# MetaData for the map

MapMetaData info

# The map data, in row-major order, starting with (0,0).

# Cell (1, 0) will be listed second, representing the next cell in the x direction.

# Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).

# The values inside are application dependent, but frequently,

# 0 represents unoccupied, 1 represents definitely occupied, and

# -1 represents unknown.

int8[] data'''


class Lidar(Node):
    def __init__(self):
        super().__init__('lidar')
        self.map_subscriber = self.create_subscription('/map', OccupancyGrid, self.callback_map, 10)
        self.odom_subscriber = self.create_subscription('/odom', Odometry, self.callback_odom, 10)
        
    def callback_map(self, msg):
        self.width = msg.info.width
        self.height = msg.info.height
        self.map = np.reshape(msg.data, (self.height, self.width))
    
    def callback_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.x
        q = msg.pose.pose.orientation
        self.yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) + np.pi / 2.0
        self.get_logger().log(f"x: {self.x}, y: {self.y}, yaw: {self.yaw}")
        pass
        
        
        

def main():
    rclpy.init()
    node = Lidar()
    rclpy.spin(node)
    rclpy.shutdown()
