#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction, LogInfo, GroupAction
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    pkg = 'auv_description'
    share = get_package_share_directory(pkg)
    prefix = get_package_prefix(pkg)

    world_path = os.path.join(share, 'worlds', 'water_world.sdf')
    xacro_file = os.path.join(share, 'urdf', 'cube', 'auv.urdf')  # Xacro file

    # Process Xacro at runtime
    robot_description_cmd = Command(['xacro ', xacro_file])

    plugin_search_path = os.path.join(prefix, 'lib')

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_path],
        output='screen'
    )

    spawn_auv = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_auv',
        output='screen',
        arguments=[
            '-name', 'auv',
            '-string', robot_description_cmd,
            '-allow_renaming', 'true',
            '-x', '0', '-y', '0', '-z', '0.5'
        ]
    )

    camera_bridge = GroupAction(actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='image_bridge',
            arguments=['/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            output='screen',
        )
    ])

    # LiDAR (PointCloud2) bridge: GZ -> ROS
    lidar_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/cube/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('force', default_value='100.0'),
        DeclareLaunchArgument('rate_hz', default_value='30.0'),

        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_search_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', share),

        LogInfo(msg=f'Loading world: {world_path}'),
        LogInfo(msg='Processing Xacro and launching AUV'),

        gz_server,
        TimerAction(period=2.0, actions=[spawn_auv]),
        TimerAction(period=4.0, actions=[camera_bridge])
    ])
