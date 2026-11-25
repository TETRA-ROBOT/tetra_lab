import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = os.path.join(
        get_package_share_directory('tetra_mmwave_radar'), 'config', 'mmwave_radar.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('port', default_value='/dev/serial/by-path/pci-0000:00:14.0-usb-0:10.3:1.0-port0'),
        Node(
            package='tetra_mmwave_radar',
            executable='mmwave_radar_node',
            name='mmwave_radar',
            output='screen',
            parameters=[LaunchConfiguration('params_file'),
                        {'port': LaunchConfiguration('port')}],
        ),
    ])
