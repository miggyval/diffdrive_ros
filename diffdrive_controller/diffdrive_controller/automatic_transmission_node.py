import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from diffdrive_interfaces.msg import WheelSpeeds
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import tf_transformations
import tf2_ros
import numpy as np

class AutomaticTransmission(Node):
    def __init__(self):
        super().__init__('diffdrive_auto')
    
    

def main():
    rclpy.init()
    node = AutomaticTransmission()
    rclpy.spin(node)
    rclpy.shutdown()