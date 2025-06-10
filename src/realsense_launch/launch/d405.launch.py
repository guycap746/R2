#!/usr/bin/env python3
"""
Launch file for Intel RealSense D405 camera with optimized settings
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup function for the launch"""
    
    # Launch arguments
    camera_name = LaunchConfiguration('camera_name')
    camera_namespace = LaunchConfiguration('camera_namespace')
    serial_no = LaunchConfiguration('serial_no')
    usb_port_id = LaunchConfiguration('usb_port_id')
    device_type = LaunchConfiguration('device_type')
    config_file = LaunchConfiguration('config_file')
    
    # RealSense camera node
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name=camera_name,
        namespace=camera_namespace,
        parameters=[
            {
                'camera_name': camera_name,
                'camera_namespace': camera_namespace,
                'serial_no': serial_no,
                'usb_port_id': usb_port_id,
                'device_type': device_type,
                'config_file': config_file,
                
                # D405 optimized settings
                'rgb_camera.profile': '640x480x30',
                'depth_module.profile': '640x480x30',
                'pointcloud.enable': True,
                'pointcloud.stream_filter': 2,
                'pointcloud.stream_index_filter': 0,
                'align_depth.enable': True,
                'colorizer.enable': False,
                'decimation_filter.enable': True,
                'spatial_filter.enable': True,
                'temporal_filter.enable': True,
                'disparity_filter.enable': False,
                'hole_filling_filter.enable': False,
                'hdr_merge.enable': False,
                'sequence_id_filter.enable': False,
                'threshold_filter.enable': False,
                'units_transform.enable': False,
                
                # Quality settings for object detection
                'depth_module.exposure.1': 8500,
                'depth_module.gain.1': 16,
                'depth_module.enable_auto_exposure.1': True,
                'rgb_camera.exposure.1': 166,
                'rgb_camera.gain.1': 64,
                'rgb_camera.enable_auto_exposure.1': True,
                'rgb_camera.white_balance.1': 4600,
                'rgb_camera.enable_auto_white_balance.1': True,
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return [realsense_node]


def generate_launch_description():
    """Generate launch description"""
    
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
            'serial_no',
            default_value='',
            description='Camera serial number (empty for any)'
        ),
        DeclareLaunchArgument(
            'usb_port_id',
            default_value='',
            description='USB port ID (empty for any)'
        ),
        DeclareLaunchArgument(
            'device_type',
            default_value='d405',
            description='Device type'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value='',
            description='Configuration file path'
        ),
        
        # Launch setup
        OpaqueFunction(function=launch_setup)
    ])