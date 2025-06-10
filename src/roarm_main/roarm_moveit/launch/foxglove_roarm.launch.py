#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Foxglove-specific launch file for RoArm M3 with enhanced web visualization
    
    This launch file provides:
    1. Foxglove Bridge with optimized configuration
    2. Enhanced topic publishing for web interface
    3. Parameter configuration for runtime tuning
    4. Diagnostic monitoring
    """
    
    # Declare launch arguments
    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Port for Foxglove Bridge WebSocket server'
    )
    
    foxglove_address_arg = DeclareLaunchArgument(
        'foxglove_address',
        default_value='0.0.0.0',
        description='Address for Foxglove Bridge to bind to'
    )
    
    enable_tls_arg = DeclareLaunchArgument(
        'enable_tls',
        default_value='false',
        description='Enable TLS/SSL for Foxglove Bridge'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('roarm_moveit'),
            'config',
            'foxglove_bridge_config.yaml'
        ]),
        description='Path to Foxglove Bridge configuration file'
    )
    
    # Launch configurations
    foxglove_port = LaunchConfiguration('foxglove_port')
    foxglove_address = LaunchConfiguration('foxglove_address')
    enable_tls = LaunchConfiguration('enable_tls')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    
    # Foxglove Bridge node with comprehensive configuration
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': foxglove_port,
            'address': foxglove_address,
            'tls': enable_tls,
            'topic_whitelist': [
                # Robot state and control
                '/joint_states',
                '/tf',
                '/tf_static',
                '/robot_description',
                
                # Camera and perception
                '/camera/color/image_raw',
                '/camera/depth/image_rect_raw', 
                '/camera/depth/color/points',
                '/camera/color/camera_info',
                '/camera/depth/camera_info',
                
                # AnyGrasp and grasping
                '/anygrasp/grasp_poses',
                '/anygrasp/debug_markers',
                '/anygrasp/filtered_grasps',
                '/anygrasp/best_grasp',
                
                # MoveIt planning and execution
                '/move_group/display_planned_path',
                '/move_group/monitored_planning_scene',
                '/execute_trajectory/feedback',
                '/execute_trajectory/status',
                '/execute_trajectory/result',
                
                # Servo control
                '/servo_node/delta_joint_cmds',
                '/servo_node/delta_twist_cmds',
                '/servo_node/status',
                
                # Robot driver
                '/roarm_driver/status',
                '/roarm_driver/joint_commands',
                '/led_ctrl',
                '/gripper_cmd',
                
                # System monitoring
                '/diagnostics',
                '/rosout',
                '/parameter_events',
                
                # Web application control
                '/webappcontrol',
                '/roarm_web_app/.*'
            ],
            'service_whitelist': [
                '/get_pose_cmd',
                '/move_point_cmd',
                '/move_circle_cmd',
                '/move_group/.*',
                '/servo_node/.*',
                '/get_parameters',
                '/set_parameters',
                '/list_parameters'
            ],
            'parameter_whitelist': [
                '/servo_node/.*',
                '/anygrasp_node/.*',
                '/roarm_driver/.*',
                '/move_group/.*'
            ],
            'use_sim_time': use_sim_time,
            'num_threads': 4,
            'max_qos_depth': 10,
            'use_compression': False
        }]
    )
    
    # Static transform publisher for camera mounting
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_link',
        arguments=['0.2', '0.0', '0.4', '0', '0.5', '0', 'base_link', 'camera_link'],
        output='screen'
    )
    
    # Robot state publisher for enhanced URDF visualization
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_foxglove',
        output='screen',
        parameters=[{
            'robot_description': open('/ros2_ws/src/roarm_main/roarm_description/urdf/roarm_description.urdf').read(),
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0
        }]
    )
    
    # Diagnostics aggregator for system monitoring
    diagnostics_node = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        output='screen',
        parameters=[{
            'analyzers': {
                'roarm_system': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'RoArm M3 System',
                    'contains': ['roarm_driver', 'moveit', 'anygrasp']
                },
                'sensors': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer', 
                    'path': 'Sensors',
                    'contains': ['camera', 'realsense']
                }
            },
            'use_sim_time': use_sim_time
        }]
    )
    
    # Parameter bridge for dynamic reconfiguration via Foxglove
    parameter_bridge_node = Node(
        package='foxglove_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        foxglove_port_arg,
        foxglove_address_arg,
        enable_tls_arg,
        use_sim_time_arg,
        config_file_arg,
        
        # Nodes
        foxglove_bridge_node,
        camera_tf_node,
        robot_state_publisher,
        diagnostics_node,
        # parameter_bridge_node,  # Uncomment if available in your distribution
    ])