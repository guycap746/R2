#!/usr/bin/env python3
"""
LeRobot Policy Executor

This module handles real-time execution of trained LeRobot policies,
including safety monitoring, performance evaluation, and adaptive control.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import torch
import json
import time
from typing import Dict, List, Optional, Any
from pathlib import Path
import cv2
from threading import Lock

# ROS2 message types
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge

class PolicyExecutionState:
    """Policy execution states"""
    IDLE = "idle"
    LOADING = "loading"
    READY = "ready"
    EXECUTING = "executing"
    PAUSED = "paused"
    ERROR = "error"

class LeRobotPolicyExecutor(Node):
    """Real-time policy executor for LeRobot models"""
    
    def __init__(self):
        super().__init__('lerobot_policy_executor')
        
        # Parameters
        self.declare_parameter('model_dir', '/tmp/lerobot_models')
        self.declare_parameter('policy_type', 'act')
        self.declare_parameter('execution_fps', 30)
        self.declare_parameter('safety_checks', True)
        self.declare_parameter('max_joint_velocity', 0.5)
        self.declare_parameter('workspace_limits', [-1.0, 1.0, -1.0, 1.0, 0.0, 1.0])
        self.declare_parameter('enable_visualization', True)
        
        # Get parameters
        self.model_dir = Path(self.get_parameter('model_dir').get_parameter_value().string_value)
        self.policy_type = self.get_parameter('policy_type').get_parameter_value().string_value
        self.execution_fps = self.get_parameter('execution_fps').get_parameter_value().integer_value
        self.safety_checks = self.get_parameter('safety_checks').get_parameter_value().bool_value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').get_parameter_value().double_value
        self.workspace_limits = self.get_parameter('workspace_limits').get_parameter_value().double_array_value
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # State variables
        self.execution_state = PolicyExecutionState.IDLE
        self.current_policy = None
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.data_lock = Lock()
        
        # Sensor data
        self.current_joint_state = None
        self.current_images = {}
        
        # Execution statistics
        self.execution_stats = {
            'total_steps': 0,
            'successful_steps': 0,
            'failed_steps': 0,
            'average_inference_time': 0.0,
            'average_execution_time': 0.0
        }
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/lerobot/executor_status', 10)
        self.action_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.stats_pub = self.create_publisher(String, '/lerobot/execution_stats', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        
        # Control subscribers
        self.load_policy_sub = self.create_subscription(
            String, '/lerobot/load_policy', self.load_policy_callback, 10)
        self.start_execution_sub = self.create_subscription(
            Bool, '/lerobot/start_execution', self.start_execution_callback, 10)
        self.stop_execution_sub = self.create_subscription(
            Bool, '/lerobot/stop_execution', self.stop_execution_callback, 10)
        
        # Execution timer
        self.execution_timer = None
        
        self.get_logger().info('LeRobot Policy Executor initialized')
        self.publish_status('Executor ready - waiting for policy')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        with self.data_lock:
            self.current_joint_state = msg
    
    def image_callback(self, msg):
        """Handle image updates"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.data_lock:
                self.current_images['rgb'] = {
                    'image': cv_image,
                    'timestamp': msg.header.stamp
                }
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')
    
    def load_policy_callback(self, msg):
        """Handle policy loading request"""
        policy_path = msg.data
        self.load_policy(policy_path)
    
    def start_execution_callback(self, msg):
        """Handle execution start request"""
        if msg.data:
            self.start_execution()
    
    def stop_execution_callback(self, msg):
        """Handle execution stop request"""
        if msg.data:
            self.stop_execution()
    
    def load_policy(self, policy_path: str):
        """Load policy from file or Hugging Face Hub"""
        try:
            self.execution_state = PolicyExecutionState.LOADING
            self.publish_status(f'Loading policy: {policy_path}')
            
            # Determine if it's a local file or HF Hub model
            if policy_path.startswith('/') or policy_path.startswith('./'):
                # Local file path
                model_file = Path(policy_path)
                if not model_file.exists():
                    # Try model directory
                    model_file = self.model_dir / policy_path
                    if not model_file.exists():
                        raise FileNotFoundError(f'Model file not found: {policy_path}')
                
                # Load model checkpoint
                checkpoint = torch.load(model_file, map_location=self.device)
                self.current_policy = self.create_policy_from_checkpoint(checkpoint)
                
            else:
                # Hugging Face Hub model (placeholder implementation)
                self.publish_status('Hugging Face Hub loading not yet implemented')
                return False
            
            self.execution_state = PolicyExecutionState.READY
            self.publish_status(f'Policy loaded successfully: {policy_path}')
            return True
            
        except Exception as e:
            self.execution_state = PolicyExecutionState.ERROR
            self.publish_status(f'Failed to load policy: {e}')
            self.get_logger().error(f'Policy loading failed: {e}')
            return False
    
    def create_policy_from_checkpoint(self, checkpoint):
        """Create policy model from checkpoint"""
        try:
            config = checkpoint.get('config', {})
            policy_type = config.get('policy_type', self.policy_type)
            
            # Create model based on type
            if policy_type == 'act':
                model = self.create_act_model()
            elif policy_type == 'cnn_lstm':
                model = self.create_cnn_lstm_model()
            else:
                raise ValueError(f'Unsupported policy type: {policy_type}')
            
            # Load state dict
            model.load_state_dict(checkpoint['model_state_dict'])
            model.to(self.device)
            model.eval()
            
            return {
                'model': model,
                'type': policy_type,
                'config': config
            }
            
        except Exception as e:
            self.get_logger().error(f'Failed to create policy from checkpoint: {e}')
            raise
    
    def create_act_model(self):
        """Create ACT model (simplified version)"""
        import torch.nn as nn
        
        class SimpleACT(nn.Module):
            def __init__(self, state_dim=6, action_dim=6, hidden_dim=256):
                super().__init__()
                self.state_encoder = nn.Sequential(
                    nn.Linear(state_dim, hidden_dim),
                    nn.ReLU(),
                    nn.Linear(hidden_dim, hidden_dim),
                    nn.ReLU()
                )
                self.action_decoder = nn.Sequential(
                    nn.Linear(hidden_dim, hidden_dim),
                    nn.ReLU(),
                    nn.Linear(hidden_dim, action_dim)
                )
            
            def forward(self, observations):
                joint_positions = observations['joint_positions']
                encoded = self.state_encoder(joint_positions)
                actions = self.action_decoder(encoded)
                return {'joint_positions': actions}
        
        return SimpleACT()
    
    def create_cnn_lstm_model(self):
        """Create CNN-LSTM model"""
        import torch.nn as nn
        
        class CNNLSTM(nn.Module):
            def __init__(self, state_dim=6, action_dim=6, hidden_dim=256):
                super().__init__()
                self.cnn = nn.Sequential(
                    nn.Conv2d(3, 32, 3, stride=2, padding=1),
                    nn.ReLU(),
                    nn.Conv2d(32, 64, 3, stride=2, padding=1),
                    nn.ReLU(),
                    nn.AdaptiveAvgPool2d((4, 4))
                )
                self.lstm = nn.LSTM(64 * 16 + state_dim, hidden_dim, batch_first=True)
                self.action_decoder = nn.Linear(hidden_dim, action_dim)
            
            def forward(self, observations):
                batch_size = observations['joint_positions'].shape[0]
                
                if 'rgb_image' in observations:
                    images = observations['rgb_image']
                    cnn_features = self.cnn(images)
                    cnn_features = cnn_features.view(batch_size, -1)
                else:
                    cnn_features = torch.zeros(batch_size, 64 * 16, device=observations['joint_positions'].device)
                
                joint_positions = observations['joint_positions']
                combined_features = torch.cat([cnn_features, joint_positions], dim=1)
                lstm_out, _ = self.lstm(combined_features.unsqueeze(1))
                actions = self.action_decoder(lstm_out.squeeze(1))
                
                return {'joint_positions': actions}
        
        return CNNLSTM()
    
    def start_execution(self):
        """Start policy execution"""
        if self.execution_state != PolicyExecutionState.READY:
            self.publish_status('Cannot start execution - policy not ready')
            return
        
        if not self.current_policy:
            self.publish_status('Cannot start execution - no policy loaded')
            return
        
        self.execution_state = PolicyExecutionState.EXECUTING
        
        # Start execution timer
        self.execution_timer = self.create_timer(1.0 / self.execution_fps, self.execute_policy_step)
        
        self.publish_status('Policy execution started')
        self.get_logger().info('Policy execution started')
    
    def stop_execution(self):
        """Stop policy execution"""
        if self.execution_timer:
            self.execution_timer.destroy()
            self.execution_timer = None
        
        self.execution_state = PolicyExecutionState.READY if self.current_policy else PolicyExecutionState.IDLE
        
        self.publish_status('Policy execution stopped')
        self.get_logger().info('Policy execution stopped')
    
    def execute_policy_step(self):
        """Execute one step of the policy"""
        try:
            if self.execution_state != PolicyExecutionState.EXECUTING:
                return
            
            # Create observation
            observation = self.create_observation()
            if observation is None:
                return
            
            # Run inference
            start_time = time.time()
            action = self.infer_action(observation)
            inference_time = time.time() - start_time
            
            if action is None:
                self.execution_stats['failed_steps'] += 1
                return
            
            # Safety checks
            if self.safety_checks and not self.validate_action_safety(action):
                self.get_logger().warning('Action failed safety validation')
                self.execution_stats['failed_steps'] += 1
                return
            
            # Execute action
            start_time = time.time()
            self.execute_action(action)
            execution_time = time.time() - start_time
            
            # Update statistics
            self.execution_stats['total_steps'] += 1
            self.execution_stats['successful_steps'] += 1
            
            # Update timing averages
            total_steps = self.execution_stats['total_steps']
            self.execution_stats['average_inference_time'] = (
                (self.execution_stats['average_inference_time'] * (total_steps - 1) + inference_time) / total_steps
            )
            self.execution_stats['average_execution_time'] = (
                (self.execution_stats['average_execution_time'] * (total_steps - 1) + execution_time) / total_steps
            )
            
            # Publish statistics periodically
            if self.execution_stats['total_steps'] % 30 == 0:  # Every second at 30 FPS
                self.publish_execution_stats()
                
        except Exception as e:
            self.get_logger().error(f'Policy execution step failed: {e}')
            self.execution_stats['failed_steps'] += 1
    
    def create_observation(self):
        """Create observation for policy inference"""
        try:
            if not self.current_joint_state:
                return None
            
            observation = {
                'joint_positions': torch.tensor(
                    self.current_joint_state.position, 
                    dtype=torch.float32, 
                    device=self.device
                ).unsqueeze(0)  # Add batch dimension
            }
            
            # Add images if available and needed by policy
            if self.current_policy['type'] == 'cnn_lstm' and 'rgb' in self.current_images:
                img = self.current_images['rgb']['image']
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).float() / 255.0
                observation['rgb_image'] = img_tensor.unsqueeze(0).to(self.device)
            
            return observation
            
        except Exception as e:
            self.get_logger().error(f'Failed to create observation: {e}')
            return None
    
    def infer_action(self, observation):
        """Run policy inference"""
        try:
            with torch.no_grad():
                output = self.current_policy['model'](observation)
                
                # Extract joint positions
                joint_positions = output['joint_positions'].cpu().numpy()[0]  # Remove batch dimension
                
                return {
                    'joint_positions': joint_positions.tolist(),
                    'timestamp': time.time()
                }
                
        except Exception as e:
            self.get_logger().error(f'Policy inference failed: {e}')
            return None
    
    def validate_action_safety(self, action):
        """Validate action for safety"""
        try:
            joint_positions = action['joint_positions']
            
            # Check joint limits (simplified)
            for i, pos in enumerate(joint_positions):
                if abs(pos) > 3.14159:  # Basic joint limit check
                    self.get_logger().warning(f'Joint {i} position {pos} exceeds limits')
                    return False
            
            # Check velocity limits
            if self.current_joint_state:
                current_positions = list(self.current_joint_state.position)
                dt = 1.0 / self.execution_fps
                
                for i, (current, target) in enumerate(zip(current_positions, joint_positions)):
                    velocity = abs(target - current) / dt
                    if velocity > self.max_joint_velocity:
                        self.get_logger().warning(f'Joint {i} velocity {velocity} exceeds limit {self.max_joint_velocity}')
                        return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Safety validation failed: {e}')
            return False
    
    def execute_action(self, action):
        """Execute action on robot"""
        try:
            # Create joint trajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            trajectory_msg.joint_names = list(self.current_joint_state.name) if self.current_joint_state else []
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = action['joint_positions']
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int((1.0 / self.execution_fps) * 1e9)
            
            trajectory_msg.points = [point]
            
            # Publish action
            self.action_pub.publish(trajectory_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to execute action: {e}')
    
    def publish_execution_stats(self):
        """Publish execution statistics"""
        try:
            stats_msg = String()
            stats_msg.data = json.dumps(self.execution_stats, indent=2)
            self.stats_pub.publish(stats_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish stats: {e}')
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[LEROBOT_EXECUTOR] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        executor = LeRobotPolicyExecutor()
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Policy executor error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()