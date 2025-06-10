#!/usr/bin/env python3
"""
Enhanced D405 + IMU Launch File for realsense_launch package

Integrates the wrist IMU with existing D405 configurations
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for D405 with wrist IMU"""
    
    # Get package directories
    realsense_launch_dir = get_package_share_directory('realsense_launch')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='d405',
            description='Camera name'
        ),
        DeclareLaunchArgument(
            'camera_namespace', 
            default_value='d405',
            description='Camera namespace'
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/ttyUSB0',
            description='IMU USB port'
        ),
        DeclareLaunchArgument(
            'enable_imu',
            default_value='true',
            description='Enable wrist IMU'
        ),
        
        # Standard D405 launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(realsense_launch_dir, 'launch', 'd405.launch.py')
            ]),
            launch_arguments={
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_namespace': LaunchConfiguration('camera_namespace'),
            }.items()
        ),
        
        # Wrist IMU integration (if available)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join('/root/ros2_workspace/src/wrist_imu_integration/launch', 'wrist_imu.launch.py')
            ]),
            launch_arguments={
                'usb_port': LaunchConfiguration('imu_port'),
                'enable_calibration': 'true',
                'enable_fusion': 'true',
            }.items(),
            condition=IfCondition(LaunchConfiguration('enable_imu'))
        ),
    ])