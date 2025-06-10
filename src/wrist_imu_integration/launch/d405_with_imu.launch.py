#!/usr/bin/env python3
"""
Combined D405 Camera + Wrist IMU Launch File

Launches both the Intel RealSense D405 camera and BNO055 wrist IMU
with proper TF tree integration for enhanced motion tracking.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function for the launch"""
    
    # Launch arguments
    camera_name = LaunchConfiguration('camera_name')
    camera_namespace = LaunchConfiguration('camera_namespace')
    imu_usb_port = LaunchConfiguration('imu_usb_port')
    enable_imu_calibration = LaunchConfiguration('enable_imu_calibration')
    enable_imu_fusion = LaunchConfiguration('enable_imu_fusion')
    
    # Package directories
    realsense_launch_dir = get_package_share_directory('realsense_launch')
    wrist_imu_dir = get_package_share_directory('wrist_imu_integration')
    
    # D405 Camera Launch
    d405_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(realsense_launch_dir, 'launch', 'd405.launch.py')
        ]),
        launch_arguments={
            'camera_name': camera_name,
            'camera_namespace': camera_namespace,
        }.items()
    )
    
    # Wrist IMU Launch
    wrist_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(wrist_imu_dir, 'launch', 'wrist_imu.launch.py')
        ]),
        launch_arguments={
            'usb_port': imu_usb_port,
            'enable_calibration': enable_imu_calibration,
            'enable_fusion': enable_imu_fusion,
        }.items()
    )
    
    # Enhanced static transform: base_link -> camera mount -> IMU
    # This represents the robot arm mounting position
    base_to_camera_mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_mount_tf',
        arguments=[
            # Translation to wrist camera mount position (adjust for your robot)
            '0.25', '0.0', '0.15',  # X: forward, Y: left, Z: up from base
            # Rotation (roll, pitch, yaw) - wrist orientation
            '0.0', '0.0', '0.0',  # Aligned with base frame initially
            # Parent and child frames
            'base_link', 'camera_mount_link'
        ],
        output='screen'
    )
    
    # Camera mount to D405 transform
    camera_mount_to_d405 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_mount_to_d405_tf', 
        arguments=[
            # D405 offset from mount center
            '0.0', '0.0', '0.0',  # Centered on mount
            # D405 orientation relative to mount
            '0.0', '0.0', '0.0',  # Aligned with mount
            # Parent and child frames
            'camera_mount_link', f'{camera_namespace.perform(context)}_link'
        ],
        output='screen'
    )
    
    # Motion analysis node - combines IMU + camera for motion detection
    motion_analysis_node = Node(
        package='wrist_imu_integration',
        executable='motion_analysis.py',
        name='motion_analysis',
        parameters=[
            {
                'camera_frame': f'{camera_namespace.perform(context)}_color_optical_frame',
                'imu_frame': 'wrist_imu_link',
                'base_frame': 'base_link',
                'motion_threshold': 0.05,  # rad/s
                'stabilization_enabled': True,
                'publish_motion_mask': True,
            }
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_motion_analysis'))
    )
    
    return [
        d405_launch,
        wrist_imu_launch,
        base_to_camera_mount,
        camera_mount_to_d405,
        motion_analysis_node
    ]


def generate_launch_description():
    """Generate launch description"""
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='d405',
            description='RealSense camera name'
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='d405',
            description='RealSense camera namespace'
        ),
        DeclareLaunchArgument(
            'imu_usb_port',
            default_value='/dev/ttyUSB0',
            description='USB port for BNO055 IMU'
        ),
        DeclareLaunchArgument(
            'enable_imu_calibration',
            default_value='true',
            description='Enable automatic IMU calibration'
        ),
        DeclareLaunchArgument(
            'enable_imu_fusion',
            default_value='true',
            description='Enable IMU-camera data fusion'
        ),
        DeclareLaunchArgument(
            'enable_motion_analysis',
            default_value='true',
            description='Enable motion analysis and stabilization'
        ),
        
        # Launch setup
        OpaqueFunction(function=launch_setup)
    ])