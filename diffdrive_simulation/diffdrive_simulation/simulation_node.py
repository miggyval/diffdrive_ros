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

L = 2.0  # Wheel base

class DiffDriveSim(Node):
    def __init__(self):
        super().__init__('diffdrive_simulation')
        self.sub_wheel = self.create_subscription(WheelSpeeds, '/wheel_speeds', self.callback_wheel, 10)
        self.sub_reset = self.create_subscription(Bool, '/reset_sim', self.callback_reset, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_map = self.create_publisher(OccupancyGrid, '/map', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.v_l, self.v_r = 0.0, 0.0
        self.last_time = self.get_clock().now()
        self.create_timer(0.001, self.update)
        self.reset()
        
    def callback_wheel(self, msg):
        self.v_l = msg.left
        self.v_r = msg.right
        
    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v_l = 0
        self.v_r = 0
        
    def callback_reset(self, msg):
        if msg.data:
            self.reset()
        
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        v = (self.v_l + self.v_r) / 2.0
        omega = (self.v_r - self.v_l) / L
        self.x += -v * np.sin(self.theta) * dt + np.random.normal(0.0, 0.1) * dt
        self.y += v * np.cos(self.theta) * dt + np.random.normal(0.0, 0.1) * dt
        self.theta += omega * dt

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        self.pub_odom.publish(odom)
        
        
        GRID_SIZE = 150
        GRID_BORDER_UNK = 20
        GRID_BORDER_OCC = 35
        occ = OccupancyGrid()
        occ.header.stamp = now.to_msg()
        occ.header.frame_id = 'odom'
        occ.info.width = GRID_SIZE
        occ.info.height = GRID_SIZE
        occ.info.resolution = 0.25
        occ.info.origin.position.x = -occ.info.resolution * GRID_SIZE / 2
        occ.info.origin.position.y = -occ.info.resolution * GRID_SIZE / 2
        occ_grid = -np.ones((GRID_SIZE, GRID_SIZE), dtype=np.int8)
        occ_grid[GRID_BORDER_UNK:-GRID_BORDER_UNK, GRID_BORDER_UNK + 10:-GRID_BORDER_UNK - 10] = 100 * np.ones((GRID_SIZE - 2 * GRID_BORDER_UNK, GRID_SIZE - 2 * (GRID_BORDER_UNK + 10)), dtype=np.int8)
        occ_grid[GRID_BORDER_OCC:-GRID_BORDER_OCC, GRID_BORDER_OCC + 10:-GRID_BORDER_OCC - 10] = np.zeros((GRID_SIZE - 2 * GRID_BORDER_OCC, GRID_SIZE - 2 * (GRID_BORDER_OCC + 10)), dtype=np.int8)
        occ_reshape = np.reshape(occ_grid, (GRID_SIZE * GRID_SIZE,)).astype(np.int8).tolist()
        occ.data = occ_reshape
        self.pub_map.publish(occ)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)
        
        


def main():
    rclpy.init()
    node = DiffDriveSim()
    rclpy.spin(node)
    rclpy.shutdown()
