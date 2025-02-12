import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='transportation_hub',
            executable='transportation_hub_node',
            name='transportation_hub_node',
            output='screen',
        ),
    ])