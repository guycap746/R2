#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get package directories
    xarm_moveit_config = get_package_share_directory('xarm_6_moveit_config')
    xarm_description = get_package_share_directory('xarm_6_description')
    
    # File paths
    urdf_file = os.path.join(xarm_description, 'urdf', 'xarm_6.urdf')
    srdf_file = os.path.join(xarm_moveit_config, 'config', 'xarm_6.srdf')
    kinematics_yaml = os.path.join(xarm_moveit_config, 'config', 'kinematics.yaml')
    joint_limits_yaml = os.path.join(xarm_description, 'config', 'joint_limits.yaml')
    moveit_controllers_yaml = os.path.join(xarm_moveit_config, 'config', 'moveit_controllers.yaml')
    
    # Robot description
    robot_description_content = ParameterValue(
        Command(['cat ', urdf_file]),
        value_type=str
    )
    
    # Robot description semantic
    robot_description_semantic_content = ParameterValue(
        Command(['cat ', srdf_file]),
        value_type=str
    )
    
    # Kinematics configuration
    kinematics_yaml_content = {}
    if os.path.exists(kinematics_yaml):
        kinematics_yaml_content = PathJoinSubstitution([
            FindPackageShare("xarm_6_moveit_config"),
            "config",
            "kinematics.yaml"
        ])
    
    # Planning configuration
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1
        }
    }
    
    ompl_planning_yaml = os.path.join(xarm_moveit_config, 'config', 'ompl_planning.yaml')
    if os.path.exists(ompl_planning_yaml):
        ompl_planning_pipeline_config['move_group'].update(
            {'ompl': PathJoinSubstitution([FindPackageShare("xarm_6_moveit_config"), "config", "ompl_planning.yaml"])}
        )
    
    # Trajectory execution configuration
    moveit_simple_controllers_yaml = {}
    if os.path.exists(moveit_controllers_yaml):
        moveit_simple_controllers_yaml = PathJoinSubstitution([
            FindPackageShare("xarm_6_moveit_config"),
            "config",
            "moveit_controllers.yaml"
        ])
    
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01
    }
    
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True
    }
    
    # Common parameters for all nodes
    moveit_config = {
        'robot_description': robot_description_content,
        'robot_description_semantic': robot_description_semantic_content,
        'robot_description_kinematics': kinematics_yaml_content,
        'planning_pipelines': ['ompl'],
        'use_sim_time': use_sim_time,
    }
    moveit_config.update(ompl_planning_pipeline_config)
    moveit_config.update(trajectory_execution)
    moveit_config.update(planning_scene_monitor_parameters)
    
    if moveit_simple_controllers_yaml:
        moveit_config['moveit_simple_controller_manager'] = moveit_simple_controllers_yaml
        moveit_config['moveit_controller_manager'] = 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[moveit_config],
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # RViz with MoveIt configuration
    rviz_config_file = os.path.join(xarm_moveit_config, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{
            'robot_description': robot_description_content,
            'robot_description_semantic': robot_description_semantic_content,
            'robot_description_kinematics': kinematics_yaml_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    return [
        robot_state_publisher,
        joint_state_publisher,
        move_group_node,
        rviz_node
    ]

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])