#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    station_name_arg = DeclareLaunchArgument('station_name', default_value='charging_station_lidar_v')
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='3.0', description='X position of charging station')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0', description='Y position of charging station')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.0', description='Z position (0 = on ground)')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='3.14159', description='Yaw angle (default facing robot origin)')

    station_name = LaunchConfiguration('station_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Spawning LiDAR V-Shape Charging Station using SDF model file
    spawn_station = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', station_name,
            '-file', PathJoinSubstitution([
                FindPackageShare('robot_gazebo_sim'),
                'models',
                'charging_station_lidar_v',
                'model.sdf'
            ]),
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
            '-R', LaunchConfiguration('roll'),
            '-P', LaunchConfiguration('pitch'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        station_name_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        spawn_station
    ])
