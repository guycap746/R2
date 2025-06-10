#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for RoArm M3 with AnyGrasp integration
    
    This launch file starts:
    1. RoArm M3 driver
    2. MoveIt2 planning
    3. RealSense camera
    4. AnyGrasp grasp detection
    5. Integrated grasping pipeline
    """
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware for testing'
    )
    
    camera_enabled_arg = DeclareLaunchArgument(
        'camera_enabled',
        default_value='true',
        description='Enable RealSense camera'
    )
    
    dual_camera_arg = DeclareLaunchArgument(
        'dual_camera',
        default_value='false',
        description='Enable dual camera setup (primary + side camera for verification)'
    )
    
    anygrasp_enabled_arg = DeclareLaunchArgument(
        'anygrasp_enabled',
        default_value='true',
        description='Enable AnyGrasp grasp detection'
    )
    
    roboflow_enabled_arg = DeclareLaunchArgument(
        'roboflow_enabled',
        default_value='true',
        description='Enable Roboflow image upload for training data collection'
    )
    
    # Get package directories
    roarm_moveit_dir = get_package_share_directory('roarm_moveit')
    realsense_launch_dir = get_package_share_directory('realsense_launch')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    camera_enabled = LaunchConfiguration('camera_enabled')
    dual_camera = LaunchConfiguration('dual_camera')
    anygrasp_enabled = LaunchConfiguration('anygrasp_enabled')
    roboflow_enabled = LaunchConfiguration('roboflow_enabled')
    
    # RoArm driver node
    roarm_driver_node = Node(
        package='roarm_driver',
        executable='roarm_driver',
        name='roarm_driver',
        output='screen',
        condition=UnlessCondition(use_fake_hardware)
    )
    
    # MoveIt2 move_group launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('roarm_moveit'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_fake_hardware': use_fake_hardware,
        }.items()
    )
    
    # RViz launch for visualization
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('roarm_moveit'),
                'launch',
                'moveit_rviz.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Single camera launch (D405 configuration)
    single_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense_launch'),
                'launch',
                'd405_rgbd.launch.py'
            ])
        ]),
        condition=IfCondition(PythonExpression([camera_enabled, ' and not ', dual_camera]))
    )
    
    # Dual camera launch (Primary + Side cameras)
    dual_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense_launch'),
                'launch',
                'dual_d405.launch.py'
            ])
        ]),
        condition=IfCondition(PythonExpression([camera_enabled, ' and ', dual_camera]))
    )
    
    # Interactive AnyGrasp node for user-guided grasp selection
    anygrasp_interactive_node = Node(
        package='roarm_anygrasp_integration',
        executable='anygrasp_interactive_node',
        name='anygrasp_interactive_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_detect': False,  # Manual detection trigger
            'detection_confidence_threshold': 0.6,
            'max_candidates': 5,
            'min_grasp_width': 0.01,
            'max_grasp_width': 0.12,
        }],
        condition=IfCondition(anygrasp_enabled)
    )
    
    # Grasp execution coordinator with user confirmation workflow
    grasp_coordinator_node = Node(
        package='roarm_anygrasp_integration',
        executable='grasp_coordinator',
        name='grasp_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'approach_distance': 0.1,
            'lift_distance': 0.05,
            'gripper_open_position': 0.0,
            'gripper_close_position': 1.2,
            'movement_speed': 0.1,
            'max_candidates': 5,
            'min_confidence': 0.6,
        }],
        condition=IfCondition(anygrasp_enabled)
    )
    
    # Roboflow integration for training data collection
    roboflow_integration_node = Node(
        package='roarm_anygrasp_integration',
        executable='roboflow_integration_node',
        name='roboflow_integration_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'roboflow_api_key': '',  # Set via service or parameter
            'workspace_name': 'roarm-grasping',
            'project_name': 'grasp-detection',
            'auto_upload': True,
            'storage_path': '/tmp/roboflow_images',
        }],
        condition=IfCondition(roboflow_enabled)
    )
    
    # Servo control for fine manipulation
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moveit_servo'),
                'launch',
                'servo_control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Dual camera capture node for synchronized image capture
    dual_camera_capture_node = Node(
        package='roarm_anygrasp_integration',
        executable='dual_camera_capture_node',
        name='dual_camera_capture_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'storage_path': '/tmp/dual_camera_captures',
            'save_raw_images': True,
            'save_metadata': True,
            'auto_capture_on_grasp': True,
            'verification_pose_timeout': 10.0,
        }],
        condition=IfCondition(dual_camera)
    )
    
    # Grasp verification coordinator for robot positioning
    grasp_verification_coordinator_node = Node(
        package='roarm_anygrasp_integration',
        executable='grasp_verification_coordinator',
        name='grasp_verification_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_verification': True,
            'verification_display_time': 3.0,
            'verification_timeout': 15.0,
        }],
        condition=IfCondition(dual_camera)
    )
    
    # Foxglove Bridge for web-based visualization and control
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'tls': False,
            'topic_whitelist': [
                '/joint_states',
                '/tf',
                '/tf_static',
                '/robot_description',
                '/camera/.*',
                '/camera_primary/.*',
                '/camera_side/.*',
                '/anygrasp/.*',
                '/grasp_coordinator/.*',
                '/grasp_verification/.*',
                '/dual_camera/.*',
                '/roboflow/.*',
                '/move_group/.*',
                '/servo_node/.*',
                '/roarm_driver/.*',
                '/diagnostics',
                '/rosout',
                '/gripper_cmd',
                '/led_ctrl'
            ],
            'service_whitelist': [
                '/get_pose_cmd',
                '/move_point_cmd', 
                '/move_circle_cmd',
                '/anygrasp/get_candidates',
                '/anygrasp/select_grasp',
                '/grasp_coordinator/start_grasp_workflow',
                '/grasp_coordinator/execute_selected',
                '/grasp_verification/verify_grasp',
                '/dual_camera/capture_now',
                '/roboflow/upload_image',
                '/roboflow/configure'
            ],
            'use_sim_time': use_sim_time,
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        use_fake_hardware_arg,
        camera_enabled_arg,
        anygrasp_enabled_arg,
        roboflow_enabled_arg,
        dual_camera_arg,
        
        # Nodes and launches
        roarm_driver_node,
        move_group_launch,
        rviz_launch,
        single_camera_launch,
        dual_camera_launch,
        anygrasp_interactive_node,
        grasp_coordinator_node,
        roboflow_integration_node,
        dual_camera_capture_node,
        grasp_verification_coordinator_node,
        servo_launch,
        foxglove_bridge_node,
    ])