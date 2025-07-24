from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='diffdrive_controller', executable='target_follower_node', output='screen'),
        Node(package='diffdrive_simulation', executable='simulation_node', output='screen'),
        Node(package='diffdrive_simulation', executable='noisy_odom_node', output='screen'),
        Node(package='diffdrive_visualizer', executable='visualizer_node', output='screen')
    ])