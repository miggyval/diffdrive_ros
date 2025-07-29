from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='diffdrive_controller', executable='joystick_controller_node', output='screen'),
        Node(package='diffdrive_lidar', executable='lidar_node', output='screen'),
        Node(package='diffdrive_simulation', executable='simulation_node', output='screen'),
        Node(package='diffdrive_simulation', executable='noisy_odom_node', output='screen'),
        Node(package='diffdrive_visualizer', executable='visualizer_node', output='screen')
    ])