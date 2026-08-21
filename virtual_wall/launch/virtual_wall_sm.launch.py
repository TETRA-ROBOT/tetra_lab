#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    virtual_wall_node = Node(
        package='virtual_wall', 
        executable='virtual_wall_node',
        output='screen'
    )

    return LaunchDescription([
        virtual_wall_node
    ])
