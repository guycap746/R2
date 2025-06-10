#!/usr/bin/env python3
"""
Launch file for wrist-mounted BNO055 IMU with D405 camera integration
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
    usb_port = LaunchConfiguration('usb_port')
    frame_id = LaunchConfiguration('frame_id')
    frequency = LaunchConfiguration('frequency')
    enable_calibration = LaunchConfiguration('enable_calibration')
    config_file = LaunchConfiguration('config_file')
    
    # Get package share directory
    pkg_share = get_package_share_directory('wrist_imu_integration')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'bno055_wrist_config.yaml')
    
    # BNO055 IMU node
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='wrist_imu_driver',
        namespace='wrist_imu',
        parameters=[
            config_file.perform(context) if config_file.perform(context) else default_config,
            {
                'uart_port': usb_port,
                'frame_id': frame_id,
                'data_query_frequency': frequency,
            }
        ],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/bno055/imu', '/wrist_imu/imu'),
            ('/bno055/mag', '/wrist_imu/mag'),
            ('/bno055/temp', '/wrist_imu/temp'),
            ('/bno055/calib_status', '/wrist_imu/calib_status'),
        ]
    )
    
    # Static transform publisher: wrist_imu_link -> d405_link
    # This transform represents the physical mounting of IMU relative to camera
    imu_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wrist_imu_to_camera_tf',
        arguments=[
            # Translation (x, y, z) in meters - adjust based on physical mounting
            '0.02', '0.01', '0.005',  # IMU slightly offset from camera center
            # Rotation (roll, pitch, yaw) in radians - adjust based on orientation
            '0.0', '0.0', '0.0',  # Aligned with camera frame initially
            # Parent and child frames
            'wrist_imu_link', 'd405_color_optical_frame'
        ],
        output='screen'
    )
    
    # IMU data fusion and filtering node
    imu_fusion_node = Node(
        package='wrist_imu_integration',
        executable='imu_camera_fusion.py',
        name='imu_camera_fusion',
        parameters=[
            {
                'camera_frame': 'd405_color_optical_frame',
                'imu_frame': 'wrist_imu_link',
                'publish_filtered_imu': True,
                'publish_camera_stabilization': True,
                'fusion_frequency': 50.0,
            }
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_fusion'))
    )
    
    # IMU calibration helper node
    calibration_node = Node(
        package='wrist_imu_integration',
        executable='wrist_imu_calibration.py',
        name='wrist_imu_calibration',
        parameters=[
            {
                'auto_calibration': True,
                'calibration_timeout': 300.0,  # 5 minutes
                'save_calibration': True,
            }
        ],
        output='screen',
        condition=IfCondition(enable_calibration)
    )
    
    return [
        bno055_node,
        imu_to_camera_tf,
        imu_fusion_node,
        calibration_node
    ]


def generate_launch_description():
    """Generate launch description"""
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'usb_port',
            default_value='/dev/ttyUSB0',
            description='USB port for BNO055 IMU'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='wrist_imu_link',
            description='Frame ID for IMU data'
        ),
        DeclareLaunchArgument(
            'frequency',
            default_value='50',
            description='IMU data frequency in Hz'
        ),
        DeclareLaunchArgument(
            'enable_calibration',
            default_value='true',
            description='Enable automatic IMU calibration'
        ),
        DeclareLaunchArgument(
            'enable_fusion',
            default_value='true',
            description='Enable IMU-camera data fusion'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Path to BNO055 configuration file'
        ),
        
        # Launch setup
        OpaqueFunction(function=launch_setup)
    ])