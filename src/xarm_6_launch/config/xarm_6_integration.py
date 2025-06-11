#!/usr/bin/env python3
"""
xArm 6 Training Infrastructure Integration

This module provides integration between xArm 6 robot and the existing
LeRobot/AnyGrasp training infrastructure.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from typing import Dict, List, Optional, Tuple
import yaml
import os

# ROS2 message imports
from std_msgs.msg import String, Bool, Float32MultiArray
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Pose, PoseStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Training infrastructure imports
try:
    from roarm_lerobot_integration.lerobot_data_collector import LeRobotDataCollector
    from roarm_lerobot_integration.training_workflow_manager import TrainingWorkflowManager
    from roarm_anygrasp_integration.anygrasp_interactive_node import AnyGraspInteractiveNode
    TRAINING_AVAILABLE = True
except ImportError:
    TRAINING_AVAILABLE = False
    print("Training infrastructure not available")

class XArm6TrainingIntegration(Node):
    def __init__(self):
        super().__init__('xarm_6_training_integration')
        
        # Load configuration
        self.load_configuration()
        
        # Robot state
        self.current_joint_state = None
        self.current_tcp_pose = None
        self.gripper_state = 0.0
        
        # Training components
        self.data_collector = None
        self.workflow_manager = None
        self.anygrasp_node = None
        
        # Publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/xarm_6_joint_trajectory_controller/joint_trajectory', 10)
        self.gripper_command_pub = self.create_publisher(
            Float32MultiArray, '/gripper_controller/commands', 10)
        self.training_status_pub = self.create_publisher(
            String, '/xarm_6_training/status', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.tcp_pose_sub = self.create_subscription(
            PoseStamped, '/xarm_6/tcp_pose', self.tcp_pose_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/xarm_6/tcp_camera/image_raw', self.camera_callback, 10)
        self.training_command_sub = self.create_subscription(
            String, '/xarm_6_training/commands', self.training_command_callback, 10)
        
        # Initialize training components
        if TRAINING_AVAILABLE:
            self.initialize_training_components()
        
        # Timer for periodic updates
        self.update_timer = self.create_timer(0.02, self.update_training_state)  # 50 Hz
        
        self.get_logger().info('xArm 6 Training Integration initialized')
        
    def load_configuration(self):
        """Load configuration from YAML file"""
        config_path = os.path.join(
            os.path.dirname(__file__), 'xarm_6_training_config.yaml'
        )
        
        try:
            with open(config_path, 'r') as file:
                self.config = yaml.safe_load(file)
            self.get_logger().info(f'Loaded configuration from {config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load configuration: {e}')
            # Use default configuration
            self.config = self.get_default_config()
    
    def get_default_config(self):
        """Get default configuration"""
        return {
            'robot_config': {
                'name': 'xarm_6',
                'dof': 6,
                'joints': {
                    'names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
                }
            },
            'training': {
                'dataset': {'fps': 30, 'episode_length': 200},
                'model': {'batch_size': 8, 'learning_rate': 1e-4}
            }
        }
    
    def initialize_training_components(self):
        """Initialize training infrastructure components"""
        try:
            # Initialize data collector
            self.data_collector = LeRobotDataCollector()
            self.data_collector.configure_for_robot('xarm_6', self.config)
            
            # Initialize workflow manager
            self.workflow_manager = TrainingWorkflowManager()
            self.workflow_manager.configure_for_robot('xarm_6', self.config)
            
            # Initialize AnyGrasp if enabled
            if self.config.get('anygrasp', {}).get('enabled', False):
                self.anygrasp_node = AnyGraspInteractiveNode()
                self.anygrasp_node.configure_for_robot('xarm_6')
            
            self.get_logger().info('Training components initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize training components: {e}')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        self.current_joint_state = msg
        
        # Extract xArm 6 joint positions
        if msg.name and msg.position:
            joint_names = self.config['robot_config']['joints']['names']
            joint_positions = []
            
            for joint_name in joint_names:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    joint_positions.append(msg.position[idx])
                else:
                    joint_positions.append(0.0)
            
            # Update training data if collecting
            if self.data_collector and self.data_collector.is_collecting():
                self.data_collector.add_observation('joint_positions', joint_positions)
                if msg.velocity:
                    joint_velocities = [msg.velocity[msg.name.index(name)] if name in msg.name else 0.0 
                                      for name in joint_names]
                    self.data_collector.add_observation('joint_velocities', joint_velocities)
    
    def tcp_pose_callback(self, msg):
        """Handle TCP pose updates"""
        self.current_tcp_pose = msg.pose
        
        # Convert to array format for training
        if self.data_collector and self.data_collector.is_collecting():
            tcp_array = [
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                msg.pose.orientation.x, msg.pose.orientation.y, 
                msg.pose.orientation.z, msg.pose.orientation.w
            ]
            self.data_collector.add_observation('tcp_pose', tcp_array)
    
    def camera_callback(self, msg):
        """Handle camera image updates"""
        if self.data_collector and self.data_collector.is_collecting():
            # Convert ROS image to numpy array for training
            try:
                import cv2
                from cv_bridge import CvBridge
                
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                self.data_collector.add_observation('tcp_camera_image', cv_image)
                
            except Exception as e:
                self.get_logger().debug(f'Camera processing failed: {e}')
    
    def training_command_callback(self, msg):
        """Handle training commands"""
        command = msg.data.lower()
        
        try:
            if command == 'start_collection':
                self.start_data_collection()
            elif command == 'stop_collection':
                self.stop_data_collection()
            elif command == 'start_training':
                self.start_training()
            elif command == 'stop_training':
                self.stop_training()
            elif command == 'evaluate_model':
                self.evaluate_model()
            elif command.startswith('move_to_'):
                pose_name = command.replace('move_to_', '')
                self.move_to_predefined_pose(pose_name)
            else:
                self.get_logger().warning(f'Unknown training command: {command}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to execute training command: {e}')
    
    def start_data_collection(self):
        """Start data collection for training"""
        if self.data_collector:
            self.data_collector.start_episode()
            self.publish_training_status('Data collection started')
            self.get_logger().info('Started data collection')
        else:
            self.get_logger().error('Data collector not available')
    
    def stop_data_collection(self):
        """Stop data collection"""
        if self.data_collector:
            self.data_collector.end_episode()
            self.publish_training_status('Data collection stopped')
            self.get_logger().info('Stopped data collection')
    
    def start_training(self):
        """Start model training"""
        if self.workflow_manager:
            self.workflow_manager.start_training()
            self.publish_training_status('Training started')
            self.get_logger().info('Started model training')
        else:
            self.get_logger().error('Workflow manager not available')
    
    def stop_training(self):
        """Stop model training"""
        if self.workflow_manager:
            self.workflow_manager.stop_training()
            self.publish_training_status('Training stopped')
            self.get_logger().info('Stopped model training')
    
    def evaluate_model(self):
        """Evaluate trained model"""
        if self.workflow_manager:
            results = self.workflow_manager.evaluate_model()
            self.publish_training_status(f'Evaluation completed: {results}')
            self.get_logger().info(f'Model evaluation: {results}')
        else:
            self.get_logger().error('Workflow manager not available')
    
    def move_to_predefined_pose(self, pose_name: str):
        """Move robot to predefined pose"""
        predefined_poses = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, -0.785, 0.0, 0.0, 0.785, 0.0],
            'vertical': [0.0, -1.5708, 0.0, 0.0, 1.5708, 0.0],
            'pick': [0.0, -0.5, 0.5, 0.0, 1.0, 0.0],
            'place': [1.57, -0.5, 0.5, 0.0, 1.0, 0.0]
        }
        
        if pose_name in predefined_poses:
            target_positions = predefined_poses[pose_name]
            self.execute_joint_trajectory(target_positions)
            self.get_logger().info(f'Moving to {pose_name} pose')
        else:
            self.get_logger().error(f'Unknown pose: {pose_name}')
    
    def execute_joint_trajectory(self, target_positions: List[float], duration: float = 3.0):
        """Execute joint trajectory"""
        try:
            trajectory = JointTrajectory()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = self.config['robot_config']['joints']['names']
            
            point = JointTrajectoryPoint()
            point.positions = target_positions
            point.velocities = [0.0] * len(target_positions)
            point.accelerations = [0.0] * len(target_positions)
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1) * 1e9)
            
            trajectory.points = [point]
            self.joint_trajectory_pub.publish(trajectory)
            
            # Record action for training
            if self.data_collector and self.data_collector.is_collecting():
                self.data_collector.add_action('joint_targets', target_positions)
            
        except Exception as e:
            self.get_logger().error(f'Failed to execute trajectory: {e}')
    
    def update_training_state(self):
        """Periodic update for training state"""
        if self.workflow_manager:
            training_metrics = self.workflow_manager.get_current_metrics()
            if training_metrics:
                self.publish_training_status(f'Metrics: {training_metrics}')
    
    def publish_training_status(self, message: str):
        """Publish training status message"""
        msg = String()
        msg.data = f"[XARM_6_TRAINING] {message}"
        self.training_status_pub.publish(msg)
    
    def get_robot_state_dict(self) -> Dict:
        """Get current robot state as dictionary"""
        state = {}
        
        if self.current_joint_state:
            joint_names = self.config['robot_config']['joints']['names']
            positions = []
            velocities = []
            
            for joint_name in joint_names:
                if joint_name in self.current_joint_state.name:
                    idx = self.current_joint_state.name.index(joint_name)
                    positions.append(self.current_joint_state.position[idx])
                    if self.current_joint_state.velocity:
                        velocities.append(self.current_joint_state.velocity[idx])
                    else:
                        velocities.append(0.0)
                else:
                    positions.append(0.0)
                    velocities.append(0.0)
            
            state['joint_positions'] = positions
            state['joint_velocities'] = velocities
        
        if self.current_tcp_pose:
            state['tcp_pose'] = [
                self.current_tcp_pose.position.x,
                self.current_tcp_pose.position.y,
                self.current_tcp_pose.position.z,
                self.current_tcp_pose.orientation.x,
                self.current_tcp_pose.orientation.y,
                self.current_tcp_pose.orientation.z,
                self.current_tcp_pose.orientation.w
            ]
        
        state['gripper_state'] = self.gripper_state
        state['timestamp'] = self.get_clock().now().nanoseconds
        
        return state


def main(args=None):
    rclpy.init(args=args)
    
    try:
        integration_node = XArm6TrainingIntegration()
        rclpy.spin(integration_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'integration_node' in locals():
            integration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()