from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to your behavior tree XML file
    bt_xml = os.path.join(get_package_share_directory('my_robot'), 'bt_trees', 'navigate_bt.xml')

    # Node to run the behavior tree
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{'bt_xml_filename': bt_xml}]
    )

    return LaunchDescription([
        bt_navigator
    ])
