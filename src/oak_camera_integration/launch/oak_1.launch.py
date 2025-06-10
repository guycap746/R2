#!/usr/bin/env python3
"""
OAK-1 Mono Camera Launch File

Launches Intel OAK-1 mono camera with optimized settings for high-speed
object detection and tracking applications.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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
    default_config = os.path.join(pkg_share, 'config', 'oak_1_config.yaml')
    config_path = config_file.perform(context) if config_file.perform(context) else default_config
    
    # OAK-1 Camera Node
    oak_1_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name=camera_name,
        namespace=camera_namespace,
        parameters=[
            config_path,
            {
                # Override key parameters for OAK-1
                'camera.i_mx_id': '',  # Auto-detect
                'mono.i_resolution': '1080P',
                'mono.i_fps': 60,
                'mono.i_exposure': 8000,
                'mono.i_iso': 1600,
                
                # Disable stereo features (not available on OAK-1)
                'stereo.i_lr_check': False,
                'stereo.i_subpixel': False,
                'stereo.i_extended_disparity': False,
                
                # Publishing settings
                'publishers.i_publish_topic': True,
                'publishers.i_output_disparity': False,  # No stereo
                'publishers.i_tf_prefix': camera_namespace.perform(context),
                'publishers.i_parent_frame': f'{camera_namespace.perform(context)}_link',
                
                # Neural network disabled for basic operation
                'nn.i_nn_type': 'none',
                'nn.i_enable_passthrough': True,
            }
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            # Standardize topic names for mono camera
            ('mono/image_raw', f'/{camera_namespace.perform(context)}/image_raw'),
            ('mono/camera_info', f'/{camera_namespace.perform(context)}/camera_info'),
        ]
    )
    
    # Static transform publisher for OAK-1 mounting
    oak_1_tf = Node(
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
    
    # Motion detection node (optional)
    motion_detector = Node(
        package='oak_camera_integration',
        executable='oak_motion_detector.py',
        name=f'{camera_namespace.perform(context)}_motion_detector',
        namespace=camera_namespace,
        parameters=[
            {
                'input_image_topic': f'/{camera_namespace.perform(context)}/image_raw',
                'output_motion_topic': f'/{camera_namespace.perform(context)}/motion_detected',
                'motion_threshold': 10,
                'enable_visualization': True,
                'visualization_topic': f'/{camera_namespace.perform(context)}/motion_visualization',
            }
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_motion_detection'))
    )
    
    # Object tracking node (optional)
    object_tracker = Node(
        package='oak_camera_integration',
        executable='oak_object_tracker.py',
        name=f'{camera_namespace.perform(context)}_object_tracker',
        namespace=camera_namespace,
        parameters=[
            {
                'input_image_topic': f'/{camera_namespace.perform(context)}/image_raw',
                'output_tracking_topic': f'/{camera_namespace.perform(context)}/tracked_objects',
                'tracking_algorithm': 'CSRT',  # CSRT, KCF, MOSSE
                'max_objects': 5,
                'tracking_quality_threshold': 0.7,
            }
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_tracking'))
    )
    
    return [
        oak_1_node,
        oak_1_tf,
        motion_detector,
        object_tracker
    ]


def generate_launch_description():
    """Generate launch description"""
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='oak_1_camera',
            description='OAK-1 camera node name'
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='oak_1',
            description='OAK-1 camera namespace'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to OAK-1 configuration file'
        ),
        DeclareLaunchArgument(
            'enable_motion_detection',
            default_value='true',
            description='Enable motion detection'
        ),
        DeclareLaunchArgument(
            'enable_tracking',
            default_value='false',
            description='Enable object tracking'
        ),
        
        # Launch setup
        OpaqueFunction(function=launch_setup)
    ])