#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_gazebo_sim_path = get_package_share_directory('robot_gazebo_sim')
    try:
        tetra_description_path = get_package_share_directory('tetra_description')
    except Exception:
        tetra_description_path = ''

    # Gazebo resource paths
    install_base = os.path.dirname(os.path.dirname(robot_gazebo_sim_path))
    pkg_parent = os.path.dirname(robot_gazebo_sim_path)

    resource_paths = [
        install_base,
        pkg_parent,
        robot_gazebo_sim_path,
    ]
    if tetra_description_path:
        resource_paths.append(tetra_description_path)
        resource_paths.append(os.path.dirname(tetra_description_path))
    if os.environ.get('GZ_SIM_RESOURCE_PATH'):
        resource_paths.append(os.environ.get('GZ_SIM_RESOURCE_PATH'))

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(resource_paths)
    )

    # Common Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('robot_gazebo_sim'),
            'worlds',
            'tetra_office.sdf'
        ]),
        description='Path to Gazebo world file'
    )

    # Robot 1 Config
    robot1_name_arg = DeclareLaunchArgument('robot1_name', default_value='robot1')
    robot1_model_arg = DeclareLaunchArgument('robot1_model', default_value='m')
    robot1_x_arg = DeclareLaunchArgument('robot1_x', default_value='0.0')
    robot1_y_arg = DeclareLaunchArgument('robot1_y', default_value='0.0')
    robot1_yaw_arg = DeclareLaunchArgument('robot1_yaw', default_value='0.0')
    robot1_port_arg = DeclareLaunchArgument('robot1_port', default_value='9090')

    # Robot 2 Config
    robot2_name_arg = DeclareLaunchArgument('robot2_name', default_value='robot2')
    robot2_model_arg = DeclareLaunchArgument('robot2_model', default_value='edu')
    robot2_x_arg = DeclareLaunchArgument('robot2_x', default_value='2.0')
    robot2_y_arg = DeclareLaunchArgument('robot2_y', default_value='1.0')
    robot2_yaw_arg = DeclareLaunchArgument('robot2_yaw', default_value='0.0')
    robot2_port_arg = DeclareLaunchArgument('robot2_port', default_value='9091')

    # Launch Gazebo Simulator
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r -v 4 ', LaunchConfiguration('world_file')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn Robot 1
    spawn_robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_gazebo_sim'),
                'launch',
                'spawn_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_name': LaunchConfiguration('robot1_name'),
            'robot_model': LaunchConfiguration('robot1_model'),
            'x_pose': LaunchConfiguration('robot1_x'),
            'y_pose': LaunchConfiguration('robot1_y'),
            'yaw': LaunchConfiguration('robot1_yaw'),
            'bridge_port': LaunchConfiguration('robot1_port')
        }.items()
    )

    # Spawn Robot 2
    spawn_robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_gazebo_sim'),
                'launch',
                'spawn_robot.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_name': LaunchConfiguration('robot2_name'),
            'robot_model': LaunchConfiguration('robot2_model'),
            'x_pose': LaunchConfiguration('robot2_x'),
            'y_pose': LaunchConfiguration('robot2_y'),
            'yaw': LaunchConfiguration('robot2_yaw'),
            'bridge_port': LaunchConfiguration('robot2_port')
        }.items()
    )

    return LaunchDescription([
        gz_resource_path,
        use_sim_time_arg,
        world_file_arg,

        robot1_name_arg,
        robot1_model_arg,
        robot1_x_arg,
        robot1_y_arg,
        robot1_yaw_arg,
        robot1_port_arg,

        robot2_name_arg,
        robot2_model_arg,
        robot2_x_arg,
        robot2_y_arg,
        robot2_yaw_arg,
        robot2_port_arg,

        gazebo_launch,
        spawn_robot1,
        spawn_robot2
    ])
