import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the parameters file
    params_file = os.path.join(get_package_share_directory('my_robot'), 'params', 'nav2_params.yaml')

    # Create the launch description
    ld = LaunchDescription()

    # Add the navigation node with parameters
    ld.add_action(Node(
        package='nav2_bringup',
        executable='bringup_launch',
        name='nav2_bringup',
        parameters=[params_file],
        remappings=[
            # Add any topic remappings here if necessary
        ]
    ))

    # Add the Dijkstra node
    ld.add_action(Node(
        package='my_robot',
        executable='dijkstra_node',
        output='screen'
    ))

    # Add the SMPC node
    ld.add_action(Node(
        package='my_robot',
        executable='smpc_node',
        output='screen'
    ))

    return ld
