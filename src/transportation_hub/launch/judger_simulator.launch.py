import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='judger',
            executable='judger_node',
            name='judger_node',
            output='screen',
        ),
    ])
