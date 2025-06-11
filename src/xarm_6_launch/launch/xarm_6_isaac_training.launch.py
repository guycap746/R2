#!/usr/bin/env python3
"""
xArm 6 Isaac Sim Training Launch File

This launch file sets up xArm 6 in Isaac Sim for training workflows
including LeRobot integration and AnyGrasp compatibility.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    headless = LaunchConfiguration('headless')
    enable_cameras = LaunchConfiguration('enable_cameras')
    enable_training = LaunchConfiguration('enable_training')
    enable_anygrasp = LaunchConfiguration('enable_anygrasp')
    training_config = LaunchConfiguration('training_config')
    
    # Declare launch arguments
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Isaac Sim in headless mode'
    )
    
    declare_enable_cameras_cmd = DeclareLaunchArgument(
        'enable_cameras',
        default_value='true',
        description='Enable camera sensors in simulation'
    )
    
    declare_enable_training_cmd = DeclareLaunchArgument(
        'enable_training',
        default_value='true',
        description='Enable LeRobot training integration'
    )
    
    declare_enable_anygrasp_cmd = DeclareLaunchArgument(
        'enable_anygrasp',
        default_value='true',
        description='Enable AnyGrasp integration'
    )
    
    declare_training_config_cmd = DeclareLaunchArgument(
        'training_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('roarm_lerobot_integration'),
            'config',
            'roarm_lerobot_config.yaml'
        ]),
        description='Training configuration file'
    )
    
    # Isaac Sim launcher
    isaac_launcher = Node(
        package='xarm_6_isaac_sim',
        executable='xarm_6_isaac_launcher',
        name='xarm_6_isaac_launcher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'headless': headless,
            'enable_cameras': enable_cameras,
            'robot_model': 'xarm_6'
        }]
    )
    
    # Simulation controller
    sim_controller = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='xarm_6_isaac_sim',
                executable='xarm_6_simulation_controller',
                name='xarm_6_simulation_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # LeRobot training integration
    training_integration = TimerAction(
        period=5.0,
        actions=[
            Node(
                condition=IfCondition(enable_training),
                package='roarm_lerobot_integration',
                executable='lerobot_bridge.py',
                name='lerobot_bridge',
                output='screen',
                parameters=[
                    training_config,
                    {'use_sim_time': True, 'robot_type': 'xarm_6'}
                ]
            ),
            Node(
                condition=IfCondition(enable_training),
                package='roarm_lerobot_integration',
                executable='lerobot_data_collector.py',
                name='lerobot_data_collector',
                output='screen',
                parameters=[
                    training_config,
                    {'use_sim_time': True, 'robot_type': 'xarm_6'}
                ]
            )
        ]
    )
    
    # AnyGrasp integration
    anygrasp_integration = TimerAction(
        period=7.0,
        actions=[
            Node(
                condition=IfCondition(enable_anygrasp),
                package='roarm_anygrasp_integration',
                executable='anygrasp_interactive_node.py',
                name='anygrasp_interactive_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_type': 'xarm_6',
                    'simulation_mode': True
                }]
            )
        ]
    )
    
    # Camera integration for training
    camera_integration = TimerAction(
        period=4.0,
        actions=[
            Node(
                condition=IfCondition(enable_cameras),
                package='oak_camera_integration',
                executable='oak_camera_manager.py',
                name='oak_camera_manager_sim',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'simulation_mode': True,
                    'robot_type': 'xarm_6'
                }]
            )
        ]
    )
    
    # Training workflow manager
    training_workflow = TimerAction(
        period=8.0,
        actions=[
            Node(
                condition=IfCondition(enable_training),
                package='roarm_lerobot_integration',
                executable='training_workflow_manager.py',
                name='training_workflow_manager',
                output='screen',
                parameters=[
                    training_config,
                    {'use_sim_time': True, 'robot_type': 'xarm_6'}
                ]
            )
        ]
    )
    
    return LaunchDescription([
        declare_headless_cmd,
        declare_enable_cameras_cmd,
        declare_enable_training_cmd,
        declare_enable_anygrasp_cmd,
        declare_training_config_cmd,
        isaac_launcher,
        sim_controller,
        training_integration,
        anygrasp_integration,
        camera_integration,
        training_workflow
    ])