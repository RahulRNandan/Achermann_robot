from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen'),
        Node(package='gazebo_ros', executable='gzserver', output='screen'),
        Node(package='gazebo_ros', executable='gzclient', output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen')
    ])
