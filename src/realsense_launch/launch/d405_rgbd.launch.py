#!/usr/bin/env python3
"""
Launch file for Intel RealSense D405 with RGBD processing pipeline
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description"""
    
    # Launch arguments
    camera_name = LaunchConfiguration('camera_name')
    
    # Include basic D405 launch
    d405_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense_launch'),
                'launch',
                'd405.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': camera_name,
        }.items()
    )
    
    # RGBD processing nodes
    depth_image_proc = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='point_cloud_xyz',
        namespace=camera_name,
        remappings=[
            ('depth/image_rect', '/d405/depth/image_rect'),
            ('depth/camera_info', '/d405/depth/camera_info'),
            ('points', '/d405/depth/points'),
        ],
        parameters=[{
            'queue_size': 10,
        }]
    )
    
    # RGB-D point cloud registration
    rgbd_launch = Node(
        package='depth_image_proc',
        executable='point_cloud_xyzrgb_node',
        name='point_cloud_xyzrgb',
        namespace=camera_name,
        remappings=[
            ('rgb/image_rect_color', '/d405/color/image_rect'),
            ('rgb/camera_info', '/d405/color/camera_info'),
            ('depth_registered/image_rect', '/d405/aligned_depth_to_color/image'),
            ('depth_registered/camera_info', '/d405/aligned_depth_to_color/camera_info'),
            ('points', '/d405/depth_registered/points'),
        ],
        parameters=[{
            'queue_size': 10,
            'use_exact_sync': False,
            'approximate_sync': True,
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='d405',
            description='Camera name'
        ),
        
        # Launch nodes
        d405_launch,
        depth_image_proc,
        rgbd_launch,
    ])