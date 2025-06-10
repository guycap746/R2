#!/usr/bin/env python3
"""
Complete Isaac Sim Launch Configuration for RoArm M3

This launch file starts the complete Isaac Sim integration including:
- Isaac Sim environment
- ROS2 bridge
- Synthetic data generation
- Integration with existing RoArm packages
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Isaac Sim in headless mode'
    )
    
    scene_arg = DeclareLaunchArgument(
        'scene',
        default_value='',
        description='Scene file to load'
    )
    
    enable_bridge_arg = DeclareLaunchArgument(
        'enable_bridge',
        default_value='true',
        description='Enable Isaac-ROS bridge'
    )
    
    enable_synthetic_data_arg = DeclareLaunchArgument(
        'enable_synthetic_data',
        default_value='false',
        description='Enable synthetic data generation'
    )
    
    enable_anygrasp_arg = DeclareLaunchArgument(
        'enable_anygrasp',
        default_value='true',
        description='Enable AnyGrasp integration'
    )
    
    enable_cameras_arg = DeclareLaunchArgument(
        'enable_cameras',
        default_value='true',
        description='Enable camera simulation'
    )
    
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='roarm_m3',
        description='Robot model to load'
    )
    
    # Isaac Sim Launcher
    isaac_launcher = Node(
        package='roarm_isaac_sim',
        executable='isaac_sim_launcher.py',
        name='isaac_sim_launcher',
        output='screen',
        parameters=[{
            'headless': LaunchConfiguration('headless'),
            'scene_path': LaunchConfiguration('scene'),
            'robot_model': LaunchConfiguration('robot_model'),
            'enable_cameras': LaunchConfiguration('enable_cameras'),
            'enable_physics': True,
            'real_time_factor': 1.0
        }]
    )
    
    # Isaac-ROS Bridge
    isaac_bridge = Node(
        package='roarm_isaac_sim',
        executable='isaac_ros_bridge.py',
        name='isaac_ros_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
        parameters=[{
            'update_frequency': 60.0,
            'publish_tf': True,
            'publish_camera_data': LaunchConfiguration('enable_cameras'),
            'publish_joint_states': True,
            'robot_name': LaunchConfiguration('robot_model')
        }]
    )
    
    # Synthetic Data Generator
    synthetic_data_generator = Node(
        package='roarm_isaac_sim',
        executable='synthetic_data_generator.py',
        name='synthetic_data_generator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_synthetic_data')),
        parameters=[{
            'output_directory': '/tmp/isaac_synthetic_data',
            'num_scenes_per_batch': 50,
            'objects_per_scene_min': 3,
            'objects_per_scene_max': 8,
            'enable_domain_randomization': True,
            'generate_annotations': True,
            'image_resolution': [1280, 720]
        }]
    )
    
    # Launch robot description
    robot_description_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('roarm_description'),
            'launch',
            'robot_description.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    
    # Launch MoveIt if available
    moveit_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('roarm_moveit'),
            'launch',
            'roarm_moveit.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_isaac_sim': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_anygrasp'))
    )
    
    # Launch AnyGrasp integration
    anygrasp_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('roarm_anygrasp_integration'),
            'launch',
            'anygrasp_complete.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'simulation_mode': 'true'
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_anygrasp'))
    )
    
    # Joint State Publisher for simulation
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'source_list': ['/isaac_sim/joint_states']
        }]
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': LaunchConfiguration('robot_description')
        }]
    )
    
    # TF Static transforms for Isaac Sim
    tf_static_transforms = [
        # Base to Isaac Sim world
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
        ),
        # Camera transforms for Isaac Sim
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.05', '0', '0.15', '0', '0', '0', 'tool0', 'd405_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0.3', '0.4', '-0.785', '0', '0', 'base_link', 'oak_d_link']
        )
    ]
    
    # RViz for visualization
    rviz_config = PathJoinSubstitution([
        FindPackageShare('roarm_isaac_sim'),
        'config',
        'isaac_sim_visualization.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )
    
    return LaunchDescription([
        # Launch arguments
        headless_arg,
        scene_arg,
        enable_bridge_arg,
        enable_synthetic_data_arg,
        enable_anygrasp_arg,
        enable_cameras_arg,
        robot_model_arg,
        
        # Core Isaac Sim nodes
        isaac_launcher,
        isaac_bridge,
        synthetic_data_generator,
        
        # Robot description and state
        robot_description_launch,
        joint_state_publisher,
        robot_state_publisher,
        
        # Integration launches
        moveit_launch,
        anygrasp_launch,
        
        # TF transforms
        *tf_static_transforms,
        
        # Visualization
        rviz
    ])