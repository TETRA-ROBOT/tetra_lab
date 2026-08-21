#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_gazebo_sim_path = get_package_share_directory('robot_gazebo_sim')
    try:
        tetra_description_path = get_package_share_directory('tetra_description')
    except Exception:
        tetra_description_path = ''

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot1')
    robot_model_arg = DeclareLaunchArgument('robot_model', default_value='m')
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.0')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')
    bridge_port_arg = DeclareLaunchArgument('bridge_port', default_value='9090')

    robot_name = LaunchConfiguration('robot_name')
    robot_model = LaunchConfiguration('robot_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bridge_port = LaunchConfiguration('bridge_port')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('robot_gazebo_sim'),
                    'urdf',
                    'tetra_gazebo.xacro'
                ]),
                ' robot_model:=',
                robot_model,
                ' robot_name:=',
                robot_name,
            ])
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', [robot_name, '/robot_description'],
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

    # ROS-Gazebo Bridge for this robot
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=['ros_gz_bridge_', robot_name],
        arguments=[
            ['/model/', robot_name, '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
            ['/model/', robot_name, '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
            ['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            ['/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU'],
            ['/camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image'],
            ['/camera/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image'],
            ['/camera/depth/image_rect_raw@sensor_msgs/msg/Image@ignition.msgs.Image'],
            ['/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model'],
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
        ],
        remappings=[
            (['/model/', robot_name, '/odometry'], [robot_name, '/odom']),
            (['/model/', robot_name, '/cmd_vel'], [robot_name, '/cmd_vel']),
            ('/scan', [robot_name, '/scan']),
            ('/imu', [robot_name, '/imu']),
            ('/camera/image_raw', [robot_name, '/camera/image_raw']),
            ('/joint_states', [robot_name, '/joint_states'])
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Rosbridge server for this robot (WebSocket for single management)
    rosbridge_server = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name=['rosbridge_', robot_name],
        namespace=robot_name,
        output='screen',
        parameters=[
            {'port': bridge_port},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Power status service for single management
    power_data_read_service = Node(
        package='robot_gazebo_sim',
        executable='power_data_read_service.py',
        name=['power_service_', robot_name],
        namespace=robot_name,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        robot_model_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        bridge_port_arg,

        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        rosbridge_server,
        power_data_read_service
    ])
