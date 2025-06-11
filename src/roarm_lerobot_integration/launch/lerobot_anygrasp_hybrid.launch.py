#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch LeRobot-AnyGrasp Hybrid System
    
    This launch file starts the complete hybrid manipulation system that combines
    LeRobot AI policies with AnyGrasp classical grasp planning.
    """
    
    # Declare launch arguments
    hybrid_mode_arg = DeclareLaunchArgument(
        'hybrid_mode',
        default_value='hybrid_validate',
        description='Hybrid execution mode: lerobot_only, anygrasp_only, hybrid_validate, hybrid_enhance, adaptive'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.7',
        description='Minimum confidence threshold for execution'
    )
    
    anygrasp_weight_arg = DeclareLaunchArgument(
        'anygrasp_weight',
        default_value='0.4',
        description='Weight for AnyGrasp scores in hybrid scoring'
    )
    
    lerobot_weight_arg = DeclareLaunchArgument(
        'lerobot_weight',
        default_value='0.6',
        description='Weight for LeRobot confidence in hybrid scoring'
    )
    
    enable_learning_arg = DeclareLaunchArgument(
        'enable_learning',
        default_value='true',
        description='Enable adaptive learning from execution results'
    )
    
    enable_visualization_arg = DeclareLaunchArgument(
        'enable_visualization',
        default_value='true',
        description='Enable grasp visualization'
    )
    
    policy_path_arg = DeclareLaunchArgument(
        'policy_path',
        default_value='/tmp/lerobot_models/best_model.pt',
        description='Path to LeRobot policy model'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('roarm_lerobot_integration'),
            'config',
            'roarm_lerobot_config.yaml'
        ]),
        description='Configuration file path'
    )
    
    # Get launch configurations
    hybrid_mode = LaunchConfiguration('hybrid_mode')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    anygrasp_weight = LaunchConfiguration('anygrasp_weight')
    lerobot_weight = LaunchConfiguration('lerobot_weight')
    enable_learning = LaunchConfiguration('enable_learning')
    enable_visualization = LaunchConfiguration('enable_visualization')
    policy_path = LaunchConfiguration('policy_path')
    config_file = LaunchConfiguration('config_file')
    
    # LeRobot-AnyGrasp Integration Node
    hybrid_integration_node = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_anygrasp_integration.py',
        name='lerobot_anygrasp_integration',
        output='screen',
        parameters=[{
            'hybrid_mode': hybrid_mode,
            'confidence_threshold': confidence_threshold,
            'anygrasp_weight': anygrasp_weight,
            'lerobot_weight': lerobot_weight,
            'enable_learning': enable_learning,
            'max_grasp_candidates': 10,
            'execution_timeout': 30.0
        }],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/camera/color/image_raw', '/camera/color/image_raw'),
            ('/camera/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
            ('/camera/depth/color/points', '/camera/depth/color/points')
        ]
    )
    
    # LeRobot Bridge Node (for policy execution)
    lerobot_bridge_node = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_bridge.py',
        name='lerobot_bridge_hybrid',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/camera/color/image_raw', '/camera/color/image_raw')
        ]
    )
    
    # LeRobot Policy Executor (for real-time inference)
    policy_executor_node = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_policy_executor.py',
        name='lerobot_policy_executor_hybrid',
        output='screen',
        parameters=[{
            'model_dir': '/tmp/lerobot_models',
            'policy_type': 'act',
            'execution_fps': 30,
            'safety_checks': True,
            'max_joint_velocity': 0.5,
            'enable_visualization': enable_visualization
        }],
        condition=IfCondition(LaunchConfiguration('hybrid_mode'))
    )
    
    # AnyGrasp Interactive Node (existing)
    anygrasp_node = Node(
        package='roarm_anygrasp_integration',
        executable='anygrasp_interactive_node.py',
        name='anygrasp_interactive_hybrid',
        output='screen',
        parameters=[{
            'enable_hybrid_mode': True,
            'hybrid_confidence_threshold': confidence_threshold
        }]
    )
    
    # Grasp Coordinator (existing)
    grasp_coordinator_node = Node(
        package='roarm_anygrasp_integration', 
        executable='grasp_coordinator.py',
        name='grasp_coordinator_hybrid',
        output='screen',
        parameters=[{
            'hybrid_integration': True,
            'lerobot_validation': True
        }]
    )
    
    # Evaluation Node for Hybrid Performance
    evaluation_node = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_evaluation_node.py',
        name='hybrid_evaluation',
        output='screen',
        parameters=[{
            'evaluation_config': '/tmp/hybrid_evaluation_config.json',
            'results_dir': '/tmp/lerobot_hybrid_evaluation',
            'num_evaluation_episodes': 5,
            'generate_plots': enable_visualization
        }],
        condition=IfCondition(enable_learning)
    )
    
    # Performance Monitor Node
    performance_monitor = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_performance_monitor.py',
        name='hybrid_performance_monitor',
        output='screen',
        parameters=[{
            'monitor_hybrid': True,
            'log_performance': True,
            'adaptive_tuning': enable_learning
        }]
    )
    
    # Visualization Node (optional)
    visualization_node = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_hybrid_visualizer.py',
        name='hybrid_visualizer',
        output='screen',
        parameters=[{
            'show_grasp_candidates': True,
            'show_confidence_scores': True,
            'show_hybrid_scores': True
        }],
        condition=IfCondition(enable_visualization)
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        hybrid_mode_arg,
        confidence_threshold_arg,
        anygrasp_weight_arg,
        lerobot_weight_arg,
        enable_learning_arg,
        enable_visualization_arg,
        policy_path_arg,
        config_file_arg,
        
        # Launch info
        LogInfo(msg=['Starting LeRobot-AnyGrasp Hybrid System']),
        LogInfo(msg=['Hybrid Mode: ', hybrid_mode]),
        LogInfo(msg=['Confidence Threshold: ', confidence_threshold]),
        
        # Core hybrid system
        GroupAction([
            hybrid_integration_node,
            lerobot_bridge_node,
            policy_executor_node,
            anygrasp_node,
            grasp_coordinator_node
        ]),
        
        # Optional components
        GroupAction([
            evaluation_node,
            performance_monitor,
            visualization_node
        ]),
        
        LogInfo(msg=['LeRobot-AnyGrasp Hybrid System launched successfully'])
    ])