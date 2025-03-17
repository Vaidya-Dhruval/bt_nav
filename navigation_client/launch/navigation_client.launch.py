import launch
from launch import LaunchDescription
from launch.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_client',
            executable='navigation_client',
            name='navigation_client_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])
