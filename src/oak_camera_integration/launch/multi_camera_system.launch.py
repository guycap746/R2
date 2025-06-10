#!/usr/bin/env python3
"""
Multi-Camera System Launch File

Launches multiple camera systems without conflicts:
- Intel RealSense D405 (wrist-mounted with IMU)
- OAK-D (stereo depth for workspace overview)
- OAK-1 (high-speed tracking)

Provides coordinated multi-camera setup for comprehensive robotic perception.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function for the launch"""
    
    # Get package directories
    realsense_launch_dir = get_package_share_directory('realsense_launch')
    wrist_imu_dir = get_package_share_directory('wrist_imu_integration')
    oak_camera_dir = get_package_share_directory('oak_camera_integration')
    
    actions = []
    
    # === Intel RealSense D405 with Wrist IMU ===
    if LaunchConfiguration('enable_realsense').perform(context) == 'true':
        d405_group = GroupAction([
            PushRosNamespace('realsense'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(wrist_imu_dir, 'launch', 'd405_with_imu.launch.py')
                ]),
                launch_arguments={
                    'camera_name': 'd405',
                    'camera_namespace': 'd405',
                    'imu_port': LaunchConfiguration('imu_port'),
                    'enable_imu': LaunchConfiguration('enable_imu'),
                }.items()
            )
        ])
        actions.append(d405_group)
    
    # === OAK-D Stereo Camera ===
    if LaunchConfiguration('enable_oak_d').perform(context) == 'true':
        oak_d_group = GroupAction([
            PushRosNamespace('oak_d'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(oak_camera_dir, 'launch', 'oak_d.launch.py')
                ]),
                launch_arguments={
                    'camera_name': 'oak_d_camera',
                    'camera_namespace': 'oak_d',
                    'enable_processing': 'true',
                }.items()
            )
        ])
        actions.append(oak_d_group)
    
    # === OAK-1 Mono Camera ===  
    if LaunchConfiguration('enable_oak_1').perform(context) == 'true':
        oak_1_group = GroupAction([
            PushRosNamespace('oak_1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(oak_camera_dir, 'launch', 'oak_1.launch.py')
                ]),
                launch_arguments={
                    'camera_name': 'oak_1_camera',
                    'camera_namespace': 'oak_1',
                    'enable_motion_detection': 'true',
                    'enable_tracking': LaunchConfiguration('enable_tracking'),
                }.items()
            )
        ])
        actions.append(oak_1_group)
    
    # === Multi-Camera Coordinator ===
    if LaunchConfiguration('enable_coordinator').perform(context) == 'true':
        coordinator_node = Node(
            package='oak_camera_integration',
            executable='multi_camera_coordinator.py',
            name='multi_camera_coordinator',
            parameters=[
                {
                    # Camera configurations
                    'realsense_enabled': LaunchConfiguration('enable_realsense'),
                    'oak_d_enabled': LaunchConfiguration('enable_oak_d'),
                    'oak_1_enabled': LaunchConfiguration('enable_oak_1'),
                    
                    # Topic remapping for coordination
                    'realsense_topics': {
                        'color': '/realsense/d405/color/image_raw',
                        'depth': '/realsense/d405/depth/image_rect_raw',
                        'points': '/realsense/d405/depth/color/points',
                        'camera_info': '/realsense/d405/color/camera_info'
                    },
                    'oak_d_topics': {
                        'color': '/oak_d/oak_d/color/image_raw',
                        'depth': '/oak_d/oak_d/depth/image_raw', 
                        'points': '/oak_d/oak_d/depth/color/points',
                        'camera_info': '/oak_d/oak_d/color/camera_info'
                    },
                    'oak_1_topics': {
                        'mono': '/oak_1/oak_1/image_raw',
                        'camera_info': '/oak_1/oak_1/camera_info'
                    },
                    
                    # Coordination settings
                    'synchronize_streams': True,
                    'max_sync_delay': 0.1,  # 100ms max delay
                    'publish_combined_pointcloud': True,
                    'combined_pointcloud_topic': '/cameras/combined_pointcloud',
                    
                    # Frame coordination
                    'base_frame': 'base_link',
                    'realsense_frame': 'd405_color_optical_frame',
                    'oak_d_frame': 'oak_d_color_optical_frame',
                    'oak_1_frame': 'oak_1_optical_frame',
                }
            ],
            output='screen'
        )
        actions.append(coordinator_node)
    
    # === TF Tree Setup ===
    # Base to camera mounts transforms
    base_to_realsense_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_realsense_mount_tf',
        arguments=[
            # Wrist-mounted RealSense position (end-effector area)
            '0.25', '0.0', '0.15',   # X: forward, Y: left, Z: up
            '0.0', '0.0', '0.0',     # Roll, pitch, yaw
            'base_link', 'realsense_d405_mount'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_realsense'))
    )
    actions.append(base_to_realsense_mount)
    
    base_to_oak_d_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_oak_d_mount_tf',
        arguments=[
            # OAK-D workspace overview position (elevated, angled)
            '0.0', '0.3', '0.4',     # X: center, Y: side, Z: elevated
            '0.0', '-0.785', '0.0',  # Angled down 45 degrees
            'base_link', 'oak_d_mount'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_oak_d'))
    )
    actions.append(base_to_oak_d_mount)
    
    base_to_oak_1_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_oak_1_mount_tf',
        arguments=[
            # OAK-1 side view position (for tracking)
            '-0.2', '0.0', '0.2',    # X: back, Y: center, Z: mid
            '0.0', '0.0', '1.57',    # Rotated 90 degrees (side view)
            'base_link', 'oak_1_mount'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_oak_1'))
    )
    actions.append(base_to_oak_1_mount)
    
    return actions


def generate_launch_description():
    """Generate launch description"""
    
    return LaunchDescription([
        # === Camera Enable Arguments ===
        DeclareLaunchArgument(
            'enable_realsense',
            default_value='true',
            description='Enable Intel RealSense D405 camera'
        ),
        DeclareLaunchArgument(
            'enable_oak_d',
            default_value='true',
            description='Enable OAK-D stereo camera'
        ),
        DeclareLaunchArgument(
            'enable_oak_1',
            default_value='false',
            description='Enable OAK-1 mono camera'
        ),
        
        # === Hardware Configuration ===
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/ttyUSB0',
            description='USB port for wrist IMU'
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='true',
            description='Enable wrist IMU integration'
        ),
        
        # === Processing Options ===
        DeclareLaunchArgument(
            'enable_tracking',
            default_value='true',
            description='Enable object tracking on OAK-1'
        ),
        DeclareLaunchArgument(
            'enable_coordinator',
            default_value='true',
            description='Enable multi-camera coordinator'
        ),
        
        # === Quality Settings ===
        DeclareLaunchArgument(
            'high_quality_mode',
            default_value='false',
            description='Enable high quality mode (lower FPS)'
        ),
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Enable debug visualizations'
        ),
        
        # Launch setup
        OpaqueFunction(function=launch_setup)
    ])