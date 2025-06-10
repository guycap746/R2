#!/usr/bin/env python3
"""
Launch file for Virtual Object Creation System

This launch file starts all components needed for advanced object placement,
physics-based stacking, and intelligent scene generation in both Isaac Sim and Gazebo.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('roarm_isaac_sim')
    
    # Declare launch arguments
    simulator_arg = DeclareLaunchArgument(
        'simulator',
        default_value='isaac',
        choices=['isaac', 'gazebo', 'both'],
        description='Target simulator for object creation'
    )
    
    enable_stacking_arg = DeclareLaunchArgument(
        'enable_stacking',
        default_value='true',
        description='Enable physics-based object stacking'
    )
    
    enable_clustering_arg = DeclareLaunchArgument(
        'enable_clustering',
        default_value='true',
        description='Enable clustered object placement'
    )
    
    scene_complexity_arg = DeclareLaunchArgument(
        'scene_complexity',
        default_value='MODERATE',
        choices=['SIMPLE', 'MODERATE', 'COMPLEX', 'CLUTTERED'],
        description='Default scene complexity level'
    )
    
    workspace_min_arg = DeclareLaunchArgument(
        'workspace_min',
        default_value='[-0.1, -0.3, 0.0]',
        description='Minimum workspace bounds [x, y, z]'
    )
    
    workspace_max_arg = DeclareLaunchArgument(
        'workspace_max',
        default_value='[0.8, 0.3, 0.5]',
        description='Maximum workspace bounds [x, y, z]'
    )
    
    max_objects_arg = DeclareLaunchArgument(
        'max_objects',
        default_value='20',
        description='Maximum number of objects to place'
    )
    
    collision_margin_arg = DeclareLaunchArgument(
        'collision_margin',
        default_value='0.01',
        description='Safety margin for collision detection (meters)'
    )
    
    auto_generate_arg = DeclareLaunchArgument(
        'auto_generate',
        default_value='false',
        description='Automatically generate scenes on startup'
    )
    
    # Advanced Object Placer Node
    advanced_object_placer = Node(
        package='roarm_isaac_sim',
        executable='advanced_object_placer.py',
        name='advanced_object_placer',
        output='screen',
        parameters=[{
            'workspace_min': LaunchConfiguration('workspace_min'),
            'workspace_max': LaunchConfiguration('workspace_max'),
            'collision_margin': LaunchConfiguration('collision_margin'),
            'max_objects': LaunchConfiguration('max_objects'),
            'enable_stacking': LaunchConfiguration('enable_stacking'),
        }],
        remappings=[
            ('/object_placer/status', '/virtual_objects/placer_status'),
            ('/object_placer/visualization', '/virtual_objects/placement_markers'),
            ('/object_placer/complete', '/virtual_objects/placement_complete'),
        ]
    )
    
    # Hierarchical Scene Builder Node
    hierarchical_scene_builder = Node(
        package='roarm_isaac_sim',
        executable='hierarchical_scene_builder.py',
        name='hierarchical_scene_builder',
        output='screen',
        parameters=[{
            'workspace_min': LaunchConfiguration('workspace_min'),
            'workspace_max': LaunchConfiguration('workspace_max'),
            'default_complexity': LaunchConfiguration('scene_complexity'),
            'enable_clustering': LaunchConfiguration('enable_clustering'),
            'enable_stacking': LaunchConfiguration('enable_stacking'),
        }],
        remappings=[
            ('/scene_builder/status', '/virtual_objects/scene_status'),
            ('/scene_builder/complete', '/virtual_objects/scene_complete'),
            ('/scene_builder/stats', '/virtual_objects/scene_statistics'),
        ]
    )
    
    # Gazebo Object Interface (conditional)
    gazebo_object_interface = Node(
        package='roarm_isaac_sim',
        executable='gazebo_object_interface.py',
        name='gazebo_object_interface',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('simulator'), "' == 'gazebo' or '",
                LaunchConfiguration('simulator'), "' == 'both'"
            ])
        ),
        parameters=[{
            'gazebo_namespace': '/gazebo',
            'auto_cleanup': 'true',
        }],
        remappings=[
            ('/gazebo_interface/status', '/virtual_objects/gazebo_status'),
            ('/gazebo_interface/spawn_complete', '/virtual_objects/gazebo_spawn_complete'),
        ]
    )
    
    # Virtual Object Creation Controller Node
    virtual_object_controller = Node(
        package='roarm_isaac_sim',
        executable='virtual_object_controller.py',
        name='virtual_object_controller',
        output='screen',
        parameters=[{
            'simulator_type': LaunchConfiguration('simulator'),
            'auto_generate_scenes': LaunchConfiguration('auto_generate'),
            'default_scene_type': 'KITCHEN_COUNTER',
            'generation_interval': 30.0,  # seconds
        }],
        remappings=[
            ('/object_controller/status', '/virtual_objects/controller_status'),
            ('/object_controller/scene_ready', '/virtual_objects/scene_ready'),
        ]
    )
    
    # Static Transform Publishers for visualization
    static_transforms = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='workspace_frame_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'workspace_frame']
        )
    ]
    
    # Log startup information
    startup_info = LogInfo(
        msg=[
            'Starting Virtual Object Creation System:\n',
            '  Simulator: ', LaunchConfiguration('simulator'), '\n',
            '  Stacking enabled: ', LaunchConfiguration('enable_stacking'), '\n',
            '  Clustering enabled: ', LaunchConfiguration('enable_clustering'), '\n',
            '  Scene complexity: ', LaunchConfiguration('scene_complexity'), '\n',
            '  Max objects: ', LaunchConfiguration('max_objects'), '\n',
            '  Workspace: ', LaunchConfiguration('workspace_min'), ' to ', LaunchConfiguration('workspace_max')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        simulator_arg,
        enable_stacking_arg,
        enable_clustering_arg,
        scene_complexity_arg,
        workspace_min_arg,
        workspace_max_arg,
        max_objects_arg,
        collision_margin_arg,
        auto_generate_arg,
        
        # Startup information
        startup_info,
        
        # Core nodes
        advanced_object_placer,
        hierarchical_scene_builder,
        virtual_object_controller,
        
        # Conditional nodes
        gazebo_object_interface,
        
        # Static transforms
        GroupAction(
            actions=static_transforms
        )
    ])