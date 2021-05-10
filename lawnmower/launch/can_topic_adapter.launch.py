
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2socketcan_bridge', executable='ros2can_bridge', output='screen',
            arguments=['can0']
        ),
        Node(
            package='lawnmower', executable='can_topic_adapter', output='screen'
        )
    ])
