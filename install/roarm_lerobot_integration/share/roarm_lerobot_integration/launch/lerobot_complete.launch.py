#!/usr/bin/env python3
"""
Complete LeRobot Integration Launch File

This launch file starts all components needed for LeRobot integration with ROS2,
including data collection, policy training, and model execution.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Package directory
    pkg_share = FindPackageShare('roarm_lerobot_integration')
    
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='data_collection',
        choices=['data_collection', 'training', 'execution', 'full'],
        description='LeRobot operation mode'
    )
    
    enable_bridge_arg = DeclareLaunchArgument(
        'enable_bridge',
        default_value='true',
        description='Enable LeRobot-ROS2 bridge'
    )
    
    enable_data_collection_arg = DeclareLaunchArgument(
        'enable_data_collection',
        default_value='true',
        description='Enable data collection for training'
    )
    
    enable_policy_execution_arg = DeclareLaunchArgument(
        'enable_policy_execution',
        default_value='false',
        description='Enable policy execution on robot'
    )
    
    hf_token_arg = DeclareLaunchArgument(
        'hf_token',
        default_value='',
        description='Hugging Face API token for model upload/download'
    )
    
    dataset_repo_arg = DeclareLaunchArgument(
        'dataset_repo_id',
        default_value='roarm/manipulation_dataset',
        description='Hugging Face dataset repository ID'
    )
    
    policy_repo_arg = DeclareLaunchArgument(
        'policy_repo_id',
        default_value='roarm/manipulation_policy',
        description='Hugging Face policy repository ID'
    )
    
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value='/tmp/lerobot_data',
        description='Directory for storing collected data'
    )
    
    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        default_value='/tmp/lerobot_models',
        description='Directory for storing trained models'
    )
    
    policy_type_arg = DeclareLaunchArgument(
        'policy_type',
        default_value='act',
        choices=['act', 'diffusion', 'cnn_lstm', 'transformer'],
        description='Type of policy to train/execute'
    )
    
    batch_size_arg = DeclareLaunchArgument(
        'batch_size',
        default_value='32',
        description='Training batch size'
    )
    
    learning_rate_arg = DeclareLaunchArgument(
        'learning_rate',
        default_value='0.0001',
        description='Learning rate for training'
    )
    
    num_epochs_arg = DeclareLaunchArgument(
        'num_epochs',
        default_value='100',
        description='Number of training epochs'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Data collection and policy execution frequency'
    )
    
    max_episodes_arg = DeclareLaunchArgument(
        'max_episodes',
        default_value='100',
        description='Maximum number of episodes to collect'
    )
    
    max_episode_steps_arg = DeclareLaunchArgument(
        'max_episode_steps',
        default_value='1000',
        description='Maximum steps per episode'
    )
    
    enable_wandb_arg = DeclareLaunchArgument(
        'enable_wandb',
        default_value='false',
        description='Enable Weights & Biases logging'
    )
    
    auto_start_collection_arg = DeclareLaunchArgument(
        'auto_start_collection',
        default_value='false',
        description='Automatically start data collection on launch'
    )
    
    # LeRobot Bridge Node (always enabled if bridge is enabled)
    lerobot_bridge = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_bridge.py',
        name='lerobot_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
        parameters=[{
            'robot_type': 'roarm_m3',
            'hf_token': LaunchConfiguration('hf_token'),
            'dataset_repo_id': LaunchConfiguration('dataset_repo_id'),
            'policy_repo_id': LaunchConfiguration('policy_repo_id'),
            'enable_data_collection': LaunchConfiguration('enable_data_collection'),
            'enable_policy_execution': LaunchConfiguration('enable_policy_execution'),
            'camera_topics': ['/camera/color/image_raw', '/camera/depth/image_raw'],
            'joint_state_topic': '/joint_states',
            'control_topic': '/joint_trajectory_controller/joint_trajectory',
            'max_episode_length': LaunchConfiguration('max_episode_steps'),
            'fps': LaunchConfiguration('fps'),
        }],
        remappings=[
            ('/lerobot/status', '/lerobot/bridge_status'),
        ]
    )
    
    # Data Collector Node (enabled for data_collection and full modes)
    lerobot_data_collector = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_data_collector.py',
        name='lerobot_data_collector',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'data_collection' or '",
                LaunchConfiguration('mode'), "' == 'full'"
            ])
        ),
        parameters=[{
            'dataset_name': 'roarm_manipulation',
            'data_dir': LaunchConfiguration('data_dir'),
            'fps': LaunchConfiguration('fps'),
            'max_episodes': LaunchConfiguration('max_episodes'),
            'max_episode_steps': LaunchConfiguration('max_episode_steps'),
            'auto_save_interval': 10,
            'enable_compression': True,
            'collection_mode': 'teleoperation',
        }],
        remappings=[
            ('/lerobot/collector_status', '/lerobot/data_collector_status'),
        ]
    )
    
    # Policy Trainer Node (enabled for training and full modes)
    lerobot_policy_trainer = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_policy_trainer.py',
        name='lerobot_policy_trainer',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'training' or '",
                LaunchConfiguration('mode'), "' == 'full'"
            ])
        ),
        parameters=[{
            'data_dir': LaunchConfiguration('data_dir'),
            'model_dir': LaunchConfiguration('model_dir'),
            'policy_type': LaunchConfiguration('policy_type'),
            'batch_size': LaunchConfiguration('batch_size'),
            'learning_rate': LaunchConfiguration('learning_rate'),
            'num_epochs': LaunchConfiguration('num_epochs'),
            'validation_split': 0.2,
            'save_every_n_epochs': 10,
            'enable_wandb': LaunchConfiguration('enable_wandb'),
            'wandb_project': 'roarm_lerobot',
            'device': 'auto',
            'num_workers': 4,
        }],
        remappings=[
            ('/lerobot/trainer_status', '/lerobot/policy_trainer_status'),
        ]
    )
    
    # Policy Executor Node (enabled for execution and full modes)
    lerobot_policy_executor = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_policy_executor.py',
        name='lerobot_policy_executor',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'execution' or '",
                LaunchConfiguration('mode'), "' == 'full'"
            ])
        ),
        parameters=[{
            'model_dir': LaunchConfiguration('model_dir'),
            'policy_type': LaunchConfiguration('policy_type'),
            'execution_fps': LaunchConfiguration('fps'),
            'safety_checks': True,
            'max_joint_velocity': 0.5,
            'workspace_limits': [-1.0, 1.0, -1.0, 1.0, 0.0, 1.0],
            'enable_visualization': True,
        }],
        remappings=[
            ('/lerobot/executor_status', '/lerobot/policy_executor_status'),
        ]
    )
    
    # Teleop Interface Node (always available for data collection)
    lerobot_teleop_interface = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_teleop_interface.py',
        name='lerobot_teleop_interface',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_data_collection')),
        parameters=[{
            'joystick_device': '/dev/input/js0',
            'enable_keyboard': True,
            'enable_gamepad': True,
            'velocity_scaling': 0.1,
            'position_scaling': 0.01,
        }],
        remappings=[
            ('/lerobot/teleop_status', '/lerobot/teleop_interface_status'),
        ]
    )
    
    # Dataset Manager Node (for dataset operations)
    lerobot_dataset_manager = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_dataset_manager.py',
        name='lerobot_dataset_manager',
        output='screen',
        parameters=[{
            'data_dir': LaunchConfiguration('data_dir'),
            'hf_token': LaunchConfiguration('hf_token'),
            'dataset_repo_id': LaunchConfiguration('dataset_repo_id'),
            'auto_upload': False,
            'upload_interval': 3600,  # 1 hour
            'enable_validation': True,
        }],
        remappings=[
            ('/lerobot/dataset_status', '/lerobot/dataset_manager_status'),
        ]
    )
    
    # Evaluation Node (for model evaluation)
    lerobot_evaluation = Node(
        package='roarm_lerobot_integration',
        executable='lerobot_evaluation_node.py',
        name='lerobot_evaluation',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('mode'), "' == 'execution' or '",
                LaunchConfiguration('mode'), "' == 'full'"
            ])
        ),
        parameters=[{
            'model_dir': LaunchConfiguration('model_dir'),
            'evaluation_episodes': 10,
            'success_threshold': 0.8,
            'timeout_seconds': 60.0,
            'enable_video_recording': True,
            'video_dir': '/tmp/lerobot_videos',
        }],
        remappings=[
            ('/lerobot/evaluation_status', '/lerobot/evaluation_node_status'),
        ]
    )
    
    # Auto-start data collection (if enabled)
    auto_start_collection = TimerAction(
        period=5.0,  # Wait 5 seconds after launch
        actions=[
            Node(
                package='roarm_lerobot_integration',
                executable='lerobot_auto_start.py',
                name='lerobot_auto_start',
                output='screen',
                condition=IfCondition(LaunchConfiguration('auto_start_collection')),
                parameters=[{
                    'start_collection': True,
                    'collection_delay': 2.0,
                }]
            )
        ]
    )
    
    # Static Transform Publishers for LeRobot coordinate frames
    static_transforms = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lerobot_base_frame',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'lerobot_base_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lerobot_camera_frame',
            arguments=['0.5', '0', '0.3', '0', '0.5', '0', 'lerobot_base_frame', 'lerobot_camera_frame']
        )
    ]
    
    # Log startup information
    startup_info = LogInfo(
        msg=[
            'Starting LeRobot Integration:\n',
            '  Mode: ', LaunchConfiguration('mode'), '\n',
            '  Data Collection: ', LaunchConfiguration('enable_data_collection'), '\n',
            '  Policy Execution: ', LaunchConfiguration('enable_policy_execution'), '\n',
            '  Policy Type: ', LaunchConfiguration('policy_type'), '\n',
            '  Data Directory: ', LaunchConfiguration('data_dir'), '\n',
            '  Model Directory: ', LaunchConfiguration('model_dir'), '\n',
            '  FPS: ', LaunchConfiguration('fps'), '\n',
            '  Max Episodes: ', LaunchConfiguration('max_episodes'), '\n',
            '  Max Episode Steps: ', LaunchConfiguration('max_episode_steps')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        mode_arg,
        enable_bridge_arg,
        enable_data_collection_arg,
        enable_policy_execution_arg,
        hf_token_arg,
        dataset_repo_arg,
        policy_repo_arg,
        data_dir_arg,
        model_dir_arg,
        policy_type_arg,
        batch_size_arg,
        learning_rate_arg,
        num_epochs_arg,
        fps_arg,
        max_episodes_arg,
        max_episode_steps_arg,
        enable_wandb_arg,
        auto_start_collection_arg,
        
        # Startup information
        startup_info,
        
        # Core nodes
        lerobot_bridge,
        lerobot_data_collector,
        lerobot_policy_trainer,
        lerobot_policy_executor,
        lerobot_teleop_interface,
        lerobot_dataset_manager,
        lerobot_evaluation,
        
        # Static transforms
        GroupAction(
            actions=static_transforms
        ),
        
        # Auto-start actions
        auto_start_collection,
    ])