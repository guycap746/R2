#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for dual Intel RealSense D405 cameras:
    - Primary camera: Positioned above workspace for grasp detection
    - Side camera: Positioned to side of workspace for grasp verification
    """
    
    # Declare launch arguments
    enable_primary_camera_arg = DeclareLaunchArgument(
        'enable_primary_camera',
        default_value='true',
        description='Enable primary camera above workspace'
    )
    
    enable_side_camera_arg = DeclareLaunchArgument(
        'enable_side_camera',
        default_value='true', 
        description='Enable side camera for grasp verification'
    )
    
    primary_camera_serial_arg = DeclareLaunchArgument(
        'primary_camera_serial',
        default_value='',
        description='Serial number of primary camera (leave empty for auto-detect)'
    )
    
    side_camera_serial_arg = DeclareLaunchArgument(
        'side_camera_serial',
        default_value='',
        description='Serial number of side camera (leave empty for auto-detect)'
    )
    
    # Launch configurations
    enable_primary_camera = LaunchConfiguration('enable_primary_camera')
    enable_side_camera = LaunchConfiguration('enable_side_camera')
    primary_camera_serial = LaunchConfiguration('primary_camera_serial')
    side_camera_serial = LaunchConfiguration('side_camera_serial')
    
    # Get package directory
    realsense_launch_dir = get_package_share_directory('realsense_launch')
    
    # Primary camera (above workspace) - using existing configuration
    primary_camera_launch = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d405_primary',
        namespace='camera_primary',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('realsense_launch'),
                'config',
                'd405_config.yaml'
            ]),
            {
                'camera_name': 'd405_primary',
                'serial_no': primary_camera_serial,
                'camera_namespace': 'camera_primary',
                'tf_prefix': 'camera_primary',
            }
        ],
        condition=IfCondition(enable_primary_camera),
        remappings=[
            ('/camera/color/image_raw', '/camera_primary/color/image_raw'),
            ('/camera/depth/image_rect_raw', '/camera_primary/depth/image_rect_raw'),
            ('/camera/depth/color/points', '/camera_primary/depth/color/points'),
            ('/camera/color/camera_info', '/camera_primary/color/camera_info'),
            ('/camera/depth/camera_info', '/camera_primary/depth/camera_info'),
        ]
    )
    
    # Side camera (for grasp verification)
    side_camera_launch = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d405_side',
        namespace='camera_side',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('realsense_launch'),
                'config',
                'd405_side_config.yaml'
            ]),
            {
                'camera_name': 'd405_side',
                'serial_no': side_camera_serial,
                'camera_namespace': 'camera_side',
                'tf_prefix': 'camera_side',
            }
        ],
        condition=IfCondition(enable_side_camera),
        remappings=[
            ('/camera/color/image_raw', '/camera_side/color/image_raw'),
            ('/camera/depth/image_rect_raw', '/camera_side/depth/image_rect_raw'),
            ('/camera/depth/color/points', '/camera_side/depth/color/points'),
            ('/camera/color/camera_info', '/camera_side/color/camera_info'),
            ('/camera/depth/camera_info', '/camera_side/depth/camera_info'),
        ]
    )
    
    # Static transform publisher for primary camera (existing mount)
    primary_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='primary_camera_tf_publisher',
        arguments=[
            '0.2', '0.0', '0.4',        # x, y, z translation
            '0', '0.5', '0', '0.866',   # quaternion (30 deg pitch down)
            'base_link',
            'camera_primary_link'
        ],
        condition=IfCondition(enable_primary_camera)
    )
    
    # Static transform publisher for side camera 
    side_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='side_camera_tf_publisher',
        arguments=[
            '0.3', '-0.4', '0.2',       # x, y, z translation (to the left side)
            '0', '0', '0.707', '0.707', # quaternion (90 deg yaw to face robot)
            'base_link',
            'camera_side_link'
        ],
        condition=IfCondition(enable_side_camera)
    )
    
    return LaunchDescription([
        # Launch arguments
        enable_primary_camera_arg,
        enable_side_camera_arg, 
        primary_camera_serial_arg,
        side_camera_serial_arg,
        
        # Camera nodes
        primary_camera_launch,
        side_camera_launch,
        
        # Transform publishers
        primary_camera_tf,
        side_camera_tf,
    ])