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
    Launch file for ChArUco camera calibration workflow
    
    This launch file starts:
    1. Camera nodes (primary and/or side)
    2. ChArUco calibration node
    3. Hand-eye calibration node (optional)
    4. Robot driver and MoveIt (for hand-eye calibration)
    """
    
    # Declare launch arguments
    enable_primary_camera_arg = DeclareLaunchArgument(
        'enable_primary_camera',
        default_value='true',
        description='Enable primary camera for calibration'
    )
    
    enable_side_camera_arg = DeclareLaunchArgument(
        'enable_side_camera',
        default_value='false',
        description='Enable side camera for calibration'
    )
    
    enable_hand_eye_calibration_arg = DeclareLaunchArgument(
        'enable_hand_eye_calibration',
        default_value='false',
        description='Enable hand-eye calibration workflow'
    )
    
    enable_robot_arg = DeclareLaunchArgument(
        'enable_robot',
        default_value='false',
        description='Enable robot driver and MoveIt for hand-eye calibration'
    )
    
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware for testing'
    )
    
    board_squares_x_arg = DeclareLaunchArgument(
        'board_squares_x',
        default_value='7',
        description='Number of squares in X direction'
    )
    
    board_squares_y_arg = DeclareLaunchArgument(
        'board_squares_y',
        default_value='5', 
        description='Number of squares in Y direction'
    )
    
    square_length_arg = DeclareLaunchArgument(
        'square_length',
        default_value='0.04',
        description='Length of each square in meters'
    )
    
    marker_length_arg = DeclareLaunchArgument(
        'marker_length',
        default_value='0.032',
        description='Length of each marker in meters'
    )
    
    # Launch configurations
    enable_primary_camera = LaunchConfiguration('enable_primary_camera')
    enable_side_camera = LaunchConfiguration('enable_side_camera')
    enable_hand_eye_calibration = LaunchConfiguration('enable_hand_eye_calibration')
    enable_robot = LaunchConfiguration('enable_robot')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    board_squares_x = LaunchConfiguration('board_squares_x')
    board_squares_y = LaunchConfiguration('board_squares_y')
    square_length = LaunchConfiguration('square_length')
    marker_length = LaunchConfiguration('marker_length')
    
    # Get package directories
    realsense_launch_dir = get_package_share_directory('realsense_launch')
    roarm_moveit_dir = get_package_share_directory('roarm_moveit')
    
    # Primary camera launch
    primary_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense_launch'),
                'launch',
                'd405_rgbd.launch.py'
            ])
        ]),
        condition=IfCondition(enable_primary_camera)
    )
    
    # Side camera launch (for dual camera calibration)
    side_camera_launch = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d405_side',
        namespace='camera_side',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('realsense_launch'),
                'config',
                'd405_side_config.yaml'
            ]),
            {
                'camera_name': 'd405_side',
                'camera_namespace': 'camera_side',
                'tf_prefix': 'camera_side',
            }
        ],
        condition=IfCondition(enable_side_camera),
        remappings=[
            ('/camera/color/image_raw', '/camera_side/color/image_raw'),
            ('/camera/depth/image_rect_raw', '/camera_side/depth/image_rect_raw'),
            ('/camera/color/camera_info', '/camera_side/color/camera_info'),
            ('/camera/depth/camera_info', '/camera_side/depth/camera_info'),
        ]
    )
    
    # ChArUco calibration node
    charuco_calibration_node = Node(
        package='roarm_anygrasp_integration',
        executable='charuco_calibration_node',
        name='charuco_calibration_node',
        output='screen',
        parameters=[{
            'board_squares_x': board_squares_x,
            'board_squares_y': board_squares_y,
            'square_length': square_length,
            'marker_length': marker_length,
            'dictionary_id': 0,  # DICT_4X4_50
            'min_samples': 10,
            'max_samples': 30,
            'collection_interval': 3.0,
            'auto_collect': False,
            'save_images': True,
            'calibration_data_path': '/tmp/charuco_calibration'
        }]
    )
    
    # Hand-eye calibration node
    hand_eye_calibration_node = Node(
        package='roarm_anygrasp_integration',
        executable='hand_eye_calibration_node',
        name='hand_eye_calibration_node',
        output='screen',
        parameters=[{
            'board_squares_x': board_squares_x,
            'board_squares_y': board_squares_y,
            'square_length': square_length,
            'marker_length': marker_length,
            'dictionary_id': 0,
            'calibration_poses_count': 8,
            'workspace_center_x': 0.25,
            'workspace_center_y': 0.0,
            'workspace_center_z': 0.15,
            'workspace_radius': 0.15,
            'min_board_distance': 0.2,
            'max_board_distance': 0.4,
            'data_path': '/tmp/hand_eye_calibration'
        }],
        condition=IfCondition(enable_hand_eye_calibration)
    )
    
    # Robot driver (for hand-eye calibration)
    roarm_driver_node = Node(
        package='roarm_driver',
        executable='roarm_driver',
        name='roarm_driver',
        output='screen',
        condition=IfCondition(enable_robot)
    )
    
    # MoveIt2 launch (for hand-eye calibration)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('roarm_moveit'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'use_fake_hardware': use_fake_hardware,
        }.items(),
        condition=IfCondition(enable_robot)
    )
    
    # RViz for visualization
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('roarm_moveit'),
                'launch',
                'moveit_rviz.launch.py'
            ])
        ]),
        condition=IfCondition(enable_robot)
    )
    
    # Static transforms for cameras (if not using robot)
    primary_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='primary_camera_tf_publisher',
        arguments=[
            '0.2', '0.0', '0.4',        # x, y, z translation
            '0', '0.5', '0', '0.866',   # quaternion (30 deg pitch down)
            'base_link',
            'camera_primary_link'
        ],
        condition=IfCondition(enable_primary_camera)
    )
    
    side_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='side_camera_tf_publisher',
        arguments=[
            '0.3', '-0.4', '0.2',       # x, y, z translation
            '0', '0', '0.707', '0.707', # quaternion (90 deg yaw)
            'base_link',
            'camera_side_link'
        ],
        condition=IfCondition(enable_side_camera)
    )
    
    return LaunchDescription([
        # Launch arguments
        enable_primary_camera_arg,
        enable_side_camera_arg,
        enable_hand_eye_calibration_arg,
        enable_robot_arg,
        use_fake_hardware_arg,
        board_squares_x_arg,
        board_squares_y_arg,
        square_length_arg,
        marker_length_arg,
        
        # Camera launches
        primary_camera_launch,
        side_camera_launch,
        
        # Static transforms (when not using robot)
        primary_camera_tf,
        side_camera_tf,
        
        # Calibration nodes
        charuco_calibration_node,
        hand_eye_calibration_node,
        
        # Robot system (for hand-eye calibration)
        roarm_driver_node,
        move_group_launch,
        rviz_launch,
    ])