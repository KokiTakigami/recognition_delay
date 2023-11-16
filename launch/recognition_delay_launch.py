from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='recognition_delay',
            namespace='recognition_delay',
            executable='recognition_delay',
            name='recognition_delay'
        ),
    ])