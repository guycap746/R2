#!/usr/bin/env python3
"""
OAK-D Stereo Camera Launch File

Launches Intel OAK-D stereo camera with optimized settings for robotic grasping.
Provides RGB + stereo depth with high quality point clouds.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function for the launch"""
    
    # Launch arguments
    camera_name = LaunchConfiguration('camera_name')
    camera_namespace = LaunchConfiguration('camera_namespace')
    config_file = LaunchConfiguration('config_file')
    
    # Get package share directory
    pkg_share = get_package_share_directory('oak_camera_integration')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'oak_d_config.yaml')
    config_path = config_file.perform(context) if config_file.perform(context) else default_config
    
    # OAK-D Camera Node
    oak_d_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name=camera_name,
        namespace=camera_namespace,
        parameters=[
            config_path,
            {
                # Override key parameters
                'camera.i_mx_id': '',  # Auto-detect
                'rgb.i_resolution': '1080P',
                'rgb.i_fps': 30,
                'mono.i_resolution': '720P', 
                'mono.i_fps': 30,
                'stereo.i_lr_check': True,
                'stereo.i_subpixel': False,
                'stereo.i_extended_disparity': False,
                'stereo.i_stereo_conf_threshold': 230,
                
                # Publishing settings
                'publishers.i_publish_topic': True,
                'publishers.i_tf_prefix': camera_namespace.perform(context),
                'publishers.i_parent_frame': f'{camera_namespace.perform(context)}_link',
                
                # Quality settings
                'quality.depth_quality': 'MEDIUM',
                'quality.confidence_threshold': 230,
            }
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            # Standardize topic names
            ('rgb/image_raw', f'/{camera_namespace.perform(context)}/color/image_raw'),
            ('rgb/camera_info', f'/{camera_namespace.perform(context)}/color/camera_info'),
            ('stereo/depth', f'/{camera_namespace.perform(context)}/depth/image_raw'),
            ('stereo/camera_info', f'/{camera_namespace.perform(context)}/depth/camera_info'),
            ('stereo/points', f'/{camera_namespace.perform(context)}/depth/color/points'),
            ('left/image_raw', f'/{camera_namespace.perform(context)}/infra1/image_rect_raw'),
            ('right/image_raw', f'/{camera_namespace.perform(context)}/infra2/image_rect_raw'),
        ]
    )
    
    # Static transform publisher for OAK-D mounting
    oak_d_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{camera_namespace.perform(context)}_tf_publisher',
        arguments=[
            # Translation (x, y, z) - adjust based on mounting
            '0.0', '0.0', '0.0',
            # Rotation (roll, pitch, yaw) - adjust based on orientation
            '0.0', '0.0', '0.0',
            # Parent and child frames
            f'{camera_namespace.perform(context)}_mount', f'{camera_namespace.perform(context)}_link'
        ],
        output='screen'
    )
    
    # Point cloud processing node (optional)
    pointcloud_processor = Node(
        package='oak_camera_integration',
        executable='oak_pointcloud_processor.py',
        name=f'{camera_namespace.perform(context)}_pointcloud_processor',
        namespace=camera_namespace,
        parameters=[
            {
                'input_cloud_topic': f'/{camera_namespace.perform(context)}/depth/color/points',
                'output_cloud_topic': f'/{camera_namespace.perform(context)}/depth/color/points_filtered',
                'enable_filtering': True,
                'voxel_size': 0.005,  # 5mm voxel size
                'statistical_outlier_removal': True,
                'sor_mean_k': 50,
                'sor_std_dev': 1.0,
            }
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_processing'))
    )
    
    return [
        oak_d_node,
        oak_d_tf,
        pointcloud_processor
    ]


def generate_launch_description():
    """Generate launch description"""
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='oak_d_camera',
            description='OAK-D camera node name'
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='oak_d',
            description='OAK-D camera namespace'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to OAK-D configuration file'
        ),
        DeclareLaunchArgument(
            'enable_processing',
            default_value='true',
            description='Enable point cloud processing'
        ),
        
        # Launch setup
        OpaqueFunction(function=launch_setup)
    ])