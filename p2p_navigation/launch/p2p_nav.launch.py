from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nav2_bringup",
            executable="nav2_navigation_launch.py",
            name="nav2_navigation",
            output="screen",
        ),
        Node(
            package="p2p_navigation",
            executable="p2p_nav_bt_node",
            name="p2p_nav_bt_node",
            output="screen",
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[{"default_nav_to_pose_bt_xml": "/path/to/custom_p2p_bt.xml"}],
        ),
    ])
