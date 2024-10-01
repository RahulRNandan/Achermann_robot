from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the robot URDF and RViz config file
    urdf_file = os.path.join(get_package_share_directory('my_robot'), 'urdf', 'my_robot.urdf')
    rviz_config_file = os.path.join(get_package_share_directory('my_robot'), 'rviz', 'my_robot_config.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
