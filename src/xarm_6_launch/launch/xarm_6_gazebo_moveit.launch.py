#!/usr/bin/env python3
"""
xArm 6 Gazebo + MoveIt2 Integration Launch File

This launch file provides integrated Gazebo simulation with MoveIt2
motion planning for the xArm 6 robot.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('xarm_6_gazebo'),
            'worlds',
            'xarm_6_world.world'
        ]),
        description='Full path to world model file to load'
    )
    
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('xarm_6_moveit_config'),
            'config',
            'moveit.rviz'
        ]),
        description='RViz config file path'
    )
    
    # Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('xarm_6_gazebo'),
                'launch',
                'xarm_6_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
            'gui': gui,
            'use_sim_time': 'true'
        }.items()
    )
    
    # MoveIt2 planning - delay to ensure Gazebo is ready
    moveit_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('xarm_6_moveit_config'),
                        'launch',
                        'demo.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )
    
    # Trajectory execution bridge
    trajectory_bridge = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                name='trajectory_controller_spawner',
                arguments=['xarm_6_joint_trajectory_controller'],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    return LaunchDescription([
        declare_world_cmd,
        declare_gui_cmd,
        declare_rviz_config_cmd,
        gazebo_launch,
        moveit_launch,
        trajectory_bridge
    ])