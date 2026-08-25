from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tetra_gpio_controller',
            executable='gpio_controller_node',
            name='gpio_controller_node',
            output='screen',
            parameters=[],
        ),
    ])
