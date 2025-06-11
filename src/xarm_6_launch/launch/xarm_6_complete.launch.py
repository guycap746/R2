#!/usr/bin/env python3
"""
Complete xArm 6 Launch File

This launch file provides a comprehensive setup for xArm 6 simulation
including Gazebo, MoveIt2, and Isaac Sim options.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration variables
    sim_mode = LaunchConfiguration('sim_mode')
    use_moveit = LaunchConfiguration('use_moveit')
    use_rviz = LaunchConfiguration('use_rviz')
    gui = LaunchConfiguration('gui')
    
    # Declare launch arguments
    declare_sim_mode_cmd = DeclareLaunchArgument(
        'sim_mode',
        default_value='gazebo',
        description='Simulation mode: gazebo, isaac, or none (real robot)'
    )
    
    declare_use_moveit_cmd = DeclareLaunchArgument(
        'use_moveit',
        default_value='true',
        description='Whether to launch MoveIt2'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch simulation GUI'
    )
    
    # Gazebo simulation
    gazebo_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('sim_mode').equals('gazebo')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('xarm_6_gazebo'),
                        'launch',
                        'xarm_6_gazebo.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'true',
                    'gui': gui
                }.items()
            )
        ]
    )
    
    # Isaac Sim simulation
    isaac_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('sim_mode').equals('isaac')),
        actions=[
            Node(
                package='xarm_6_isaac_sim',
                executable='xarm_6_isaac_launcher',
                name='xarm_6_isaac_launcher',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'headless': UnlessCondition(gui)
                }]
            ),
            Node(
                package='xarm_6_isaac_sim',
                executable='xarm_6_simulation_controller',
                name='xarm_6_simulation_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # MoveIt2 planning
    moveit_launch = GroupAction(
        condition=IfCondition(use_moveit),
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
                    'use_sim_time': IfCondition(LaunchConfiguration('sim_mode').not_equals('none'))
                }.items()
            )
        ]
    )
    
    # Display only (no simulation)
    display_launch = GroupAction(
        condition=IfCondition(LaunchConfiguration('sim_mode').equals('none')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('xarm_6_description'),
                        'launch',
                        'display.launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'false',
                    'gui': gui
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        declare_sim_mode_cmd,
        declare_use_moveit_cmd,
        declare_use_rviz_cmd,
        declare_gui_cmd,
        gazebo_launch,
        isaac_launch,
        moveit_launch,
        display_launch
    ])