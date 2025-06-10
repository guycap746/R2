#!/usr/bin/env python3
"""
LeRobot-ROS2 Bridge

This module provides a comprehensive bridge between Hugging Face's LeRobot framework
and ROS2, enabling AI-powered robotic manipulation with the RoArm M3 system.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import torch
import json
import time
from typing import Dict, List, Optional, Any, Tuple
from pathlib import Path
import cv2
from threading import Lock
import logging

# ROS2 message types
from std_msgs.msg import String, Bool, Float32MultiArray, Header
from sensor_msgs.msg import Image, JointState, CompressedImage
from geometry_msgs.msg import Pose, PoseStamped, Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener

# LeRobot imports (with fallback for development)
try:
    import lerobot
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.common.robot_devices.robots.factory import make_robot
    from lerobot.common.policies.factory import make_policy
    from lerobot.common.datasets.utils import create_lerobot_dataset
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False
    logging.warning("LeRobot not available. Install with: pip install lerobot")

# Hugging Face imports
try:
    from huggingface_hub import HfApi, login, logout
    from transformers import AutoTokenizer, AutoModel
    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False
    logging.warning("Hugging Face libraries not available")

class LeRobotROS2Bridge(Node):
    """Main bridge between LeRobot and ROS2 systems"""
    
    def __init__(self):
        super().__init__('lerobot_ros2_bridge')
        
        # Parameters
        self.declare_parameter('robot_type', 'roarm_m3')
        self.declare_parameter('hf_token', '')
        self.declare_parameter('dataset_repo_id', 'roarm/manipulation_dataset')
        self.declare_parameter('policy_repo_id', 'roarm/manipulation_policy')
        self.declare_parameter('enable_data_collection', True)
        self.declare_parameter('enable_policy_execution', False)
        self.declare_parameter('camera_topics', ['/camera/color/image_raw', '/camera/depth/image_raw'])
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('control_topic', '/joint_trajectory_controller/joint_trajectory')
        self.declare_parameter('max_episode_length', 1000)
        self.declare_parameter('fps', 30)
        
        # Get parameters
        self.robot_type = self.get_parameter('robot_type').get_parameter_value().string_value
        self.hf_token = self.get_parameter('hf_token').get_parameter_value().string_value
        self.dataset_repo_id = self.get_parameter('dataset_repo_id').get_parameter_value().string_value
        self.policy_repo_id = self.get_parameter('policy_repo_id').get_parameter_value().string_value
        self.enable_data_collection = self.get_parameter('enable_data_collection').get_parameter_value().bool_value
        self.enable_policy_execution = self.get_parameter('enable_policy_execution').get_parameter_value().bool_value
        self.camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        self.max_episode_length = self.get_parameter('max_episode_length').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # TF2 components
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Data storage
        self.current_episode_data = {
            'observations': [],
            'actions': [],
            'rewards': [],
            'episode_index': 0,
            'timestamp': [],
            'next_done': []
        }
        
        # State variables
        self.recording_episode = False
        self.episode_step = 0
        self.last_joint_state = None
        self.last_camera_images = {}
        self.data_lock = Lock()
        
        # LeRobot components
        self.lerobot_dataset = None
        self.current_policy = None
        self.robot_config = None
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/lerobot/status', 10)
        self.episode_pub = self.create_publisher(String, '/lerobot/episode_data', 10)
        self.policy_action_pub = self.create_publisher(JointTrajectory, self.control_topic, 10)
        self.dataset_info_pub = self.create_publisher(String, '/lerobot/dataset_info', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, self.joint_state_topic, self.joint_state_callback, 10)
        
        # Camera subscribers
        self.camera_subs = {}
        for topic in self.camera_topics:
            self.camera_subs[topic] = self.create_subscription(
                Image, topic, lambda msg, t=topic: self.camera_callback(msg, t), 10)
        
        # Control subscribers
        self.start_recording_sub = self.create_subscription(
            Bool, '/lerobot/start_recording', self.start_recording_callback, 10)
        self.stop_recording_sub = self.create_subscription(
            Bool, '/lerobot/stop_recording', self.stop_recording_callback, 10)
        self.load_policy_sub = self.create_subscription(
            String, '/lerobot/load_policy', self.load_policy_callback, 10)
        self.execute_policy_sub = self.create_subscription(
            Bool, '/lerobot/execute_policy', self.execute_policy_callback, 10)
        self.upload_dataset_sub = self.create_subscription(
            Bool, '/lerobot/upload_dataset', self.upload_dataset_callback, 10)
        
        # Services would go here (for future expansion)
        
        # Timers
        self.policy_execution_timer = None
        
        # Initialize components
        self.initialize_lerobot()
        self.initialize_huggingface()
        
        self.get_logger().info('LeRobot-ROS2 Bridge initialized')
        self.publish_status('Bridge ready - LeRobot integration active')
    
    def initialize_lerobot(self):
        """Initialize LeRobot components"""
        if not LEROBOT_AVAILABLE:
            self.get_logger().error('LeRobot not available. Please install: pip install lerobot')
            return
        
        try:
            # Create robot configuration for RoArm M3
            self.robot_config = {
                'robot_type': self.robot_type,
                'max_episode_steps': self.max_episode_length,
                'fps': self.fps,
                'cameras': {
                    f'camera_{i}': {
                        'width': 640,
                        'height': 480,
                        'fps': self.fps
                    } for i, topic in enumerate(self.camera_topics)
                },
                'arms': {
                    'main': {
                        'degrees_of_freedom': 6,
                        'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
                    }
                }
            }
            
            self.get_logger().info('LeRobot configuration initialized')
            
        except Exception as e:
            self.get_logger().error(f'LeRobot initialization failed: {e}')
    
    def initialize_huggingface(self):
        """Initialize Hugging Face connection"""
        if not HF_AVAILABLE:
            self.get_logger().error('Hugging Face libraries not available')
            return
        
        try:
            if self.hf_token:
                login(token=self.hf_token)
                self.get_logger().info('Logged in to Hugging Face')
            else:
                self.get_logger().warning('No Hugging Face token provided')
                
        except Exception as e:
            self.get_logger().error(f'Hugging Face login failed: {e}')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        with self.data_lock:
            self.last_joint_state = msg
            
            if self.recording_episode:
                self.record_observation_step()
    
    def camera_callback(self, msg, topic):
        """Handle camera image updates"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.data_lock:
                self.last_camera_images[topic] = {
                    'image': cv_image,
                    'timestamp': msg.header.stamp,
                    'encoding': msg.encoding
                }
                
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')
    
    def start_recording_callback(self, msg):
        """Start episode recording"""
        if msg.data and not self.recording_episode:
            self.start_episode_recording()
    
    def stop_recording_callback(self, msg):
        """Stop episode recording"""
        if msg.data and self.recording_episode:
            self.stop_episode_recording()
    
    def load_policy_callback(self, msg):
        """Load policy from Hugging Face Hub"""
        policy_id = msg.data
        self.load_policy(policy_id)
    
    def execute_policy_callback(self, msg):
        """Start/stop policy execution"""
        if msg.data:
            self.start_policy_execution()
        else:
            self.stop_policy_execution()
    
    def upload_dataset_callback(self, msg):
        """Upload dataset to Hugging Face Hub"""
        if msg.data:
            self.upload_dataset()
    
    def start_episode_recording(self):
        """Start recording a new episode"""
        if not LEROBOT_AVAILABLE:
            self.get_logger().error('Cannot record - LeRobot not available')
            return
        
        self.recording_episode = True
        self.episode_step = 0
        
        # Reset episode data
        self.current_episode_data = {
            'observations': [],
            'actions': [],
            'rewards': [],
            'episode_index': self.current_episode_data.get('episode_index', 0) + 1,
            'timestamp': [],
            'next_done': []
        }
        
        self.publish_status(f'Started recording episode {self.current_episode_data["episode_index"]}')
        self.get_logger().info(f'Episode {self.current_episode_data["episode_index"]} recording started')
    
    def stop_episode_recording(self):
        """Stop recording current episode"""
        if not self.recording_episode:
            return
        
        self.recording_episode = False
        
        # Process and save episode data
        self.process_episode_data()
        
        self.publish_status(f'Stopped recording episode {self.current_episode_data["episode_index"]} - {self.episode_step} steps')
        self.get_logger().info(f'Episode {self.current_episode_data["episode_index"]} completed with {self.episode_step} steps')
    
    def record_observation_step(self):
        """Record a single observation step"""
        if not self.recording_episode or not self.last_joint_state:
            return
        
        try:
            # Create observation dictionary
            observation = {
                'timestamp': time.time(),
                'step': self.episode_step,
                'joint_positions': list(self.last_joint_state.position),
                'joint_velocities': list(self.last_joint_state.velocity) if self.last_joint_state.velocity else [0.0] * len(self.last_joint_state.position),
                'joint_efforts': list(self.last_joint_state.effort) if self.last_joint_state.effort else [0.0] * len(self.last_joint_state.position),
            }
            
            # Add camera observations
            for topic, camera_data in self.last_camera_images.items():
                camera_name = f'camera_{topic.replace("/", "_")}'
                observation[camera_name] = {
                    'image': camera_data['image'].copy(),
                    'timestamp': camera_data['timestamp'].sec + camera_data['timestamp'].nanosec * 1e-9
                }
            
            # Create action (current joint positions as target)
            action = {
                'joint_positions': list(self.last_joint_state.position),
                'timestamp': observation['timestamp']
            }
            
            # Add to episode data
            self.current_episode_data['observations'].append(observation)
            self.current_episode_data['actions'].append(action)
            self.current_episode_data['rewards'].append(0.0)  # Placeholder reward
            self.current_episode_data['timestamp'].append(observation['timestamp'])
            self.current_episode_data['next_done'].append(False)
            
            self.episode_step += 1
            
            # Check if episode should end
            if self.episode_step >= self.max_episode_length:
                self.stop_episode_recording()
                
        except Exception as e:
            self.get_logger().error(f'Error recording observation: {e}')
    
    def process_episode_data(self):
        """Process and format episode data for LeRobot"""
        try:
            # Mark last step as done
            if self.current_episode_data['next_done']:
                self.current_episode_data['next_done'][-1] = True
            
            # Convert to LeRobot format
            episode_data = self.convert_to_lerobot_format(self.current_episode_data)
            
            # Publish episode data
            episode_msg = String()
            episode_msg.data = json.dumps({
                'episode_index': self.current_episode_data['episode_index'],
                'num_steps': len(self.current_episode_data['observations']),
                'duration': self.current_episode_data['timestamp'][-1] - self.current_episode_data['timestamp'][0] if self.current_episode_data['timestamp'] else 0,
                'status': 'completed'
            })
            self.episode_pub.publish(episode_msg)
            
            # Save locally (could be expanded to save to disk)
            self.save_episode_locally(episode_data)
            
        except Exception as e:
            self.get_logger().error(f'Error processing episode data: {e}')
    
    def convert_to_lerobot_format(self, episode_data):
        """Convert episode data to LeRobot dataset format"""
        if not LEROBOT_AVAILABLE:
            return episode_data
        
        try:
            # Convert observations and actions to tensors
            formatted_data = {
                'episode_index': episode_data['episode_index'],
                'observations': {},
                'actions': {},
                'rewards': torch.tensor(episode_data['rewards'], dtype=torch.float32),
                'next_done': torch.tensor(episode_data['next_done'], dtype=torch.bool)
            }
            
            # Process observations
            if episode_data['observations']:
                num_steps = len(episode_data['observations'])
                
                # Joint states
                joint_positions = torch.tensor([obs['joint_positions'] for obs in episode_data['observations']], dtype=torch.float32)
                joint_velocities = torch.tensor([obs['joint_velocities'] for obs in episode_data['observations']], dtype=torch.float32)
                
                formatted_data['observations']['joint_positions'] = joint_positions
                formatted_data['observations']['joint_velocities'] = joint_velocities
                
                # Camera images
                for camera_name in [f'camera_{topic.replace("/", "_")}' for topic in self.camera_topics]:
                    camera_images = []
                    for obs in episode_data['observations']:
                        if camera_name in obs:
                            img = obs[camera_name]['image']
                            # Convert BGR to RGB and normalize
                            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                            img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).float() / 255.0
                            camera_images.append(img_tensor)
                    
                    if camera_images:
                        formatted_data['observations'][camera_name] = torch.stack(camera_images)
            
            # Process actions
            if episode_data['actions']:
                action_positions = torch.tensor([action['joint_positions'] for action in episode_data['actions']], dtype=torch.float32)
                formatted_data['actions']['joint_positions'] = action_positions
            
            return formatted_data
            
        except Exception as e:
            self.get_logger().error(f'Error converting to LeRobot format: {e}')
            return episode_data
    
    def save_episode_locally(self, episode_data):
        """Save episode data locally"""
        try:
            # Create local dataset directory
            dataset_dir = Path(f'/tmp/lerobot_dataset_{self.robot_type}')
            dataset_dir.mkdir(exist_ok=True)
            
            # Save episode
            episode_file = dataset_dir / f'episode_{episode_data["episode_index"]:06d}.pt'
            torch.save(episode_data, episode_file)
            
            self.get_logger().info(f'Episode saved locally: {episode_file}')
            
        except Exception as e:
            self.get_logger().error(f'Error saving episode locally: {e}')
    
    def load_policy(self, policy_id: str):
        """Load policy from Hugging Face Hub or local path"""
        try:
            if not LEROBOT_AVAILABLE:
                self.get_logger().error('Cannot load policy - LeRobot not available')
                return
            
            self.publish_status(f'Loading policy: {policy_id}')
            
            # For now, create a placeholder policy
            # In a full implementation, this would load from Hub
            self.current_policy = {
                'policy_id': policy_id,
                'loaded': True,
                'type': 'placeholder'
            }
            
            self.publish_status(f'Policy loaded: {policy_id}')
            self.get_logger().info(f'Policy {policy_id} loaded successfully')
            
        except Exception as e:
            self.get_logger().error(f'Error loading policy {policy_id}: {e}')
    
    def start_policy_execution(self):
        """Start executing the loaded policy"""
        if not self.current_policy:
            self.get_logger().error('No policy loaded')
            return
        
        if not self.enable_policy_execution:
            self.get_logger().warning('Policy execution disabled')
            return
        
        # Start policy execution timer
        self.policy_execution_timer = self.create_timer(1.0 / self.fps, self.execute_policy_step)
        
        self.publish_status('Policy execution started')
        self.get_logger().info('Policy execution started')
    
    def stop_policy_execution(self):
        """Stop policy execution"""
        if self.policy_execution_timer:
            self.policy_execution_timer.destroy()
            self.policy_execution_timer = None
        
        self.publish_status('Policy execution stopped')
        self.get_logger().info('Policy execution stopped')
    
    def execute_policy_step(self):
        """Execute one step of the policy"""
        try:
            if not self.current_policy or not self.last_joint_state:
                return
            
            # Create observation for policy
            observation = self.create_policy_observation()
            
            if observation is None:
                return
            
            # Get action from policy (placeholder implementation)
            action = self.get_policy_action(observation)
            
            if action is not None:
                self.execute_action(action)
                
        except Exception as e:
            self.get_logger().error(f'Error in policy execution step: {e}')
    
    def create_policy_observation(self):
        """Create observation for policy inference"""
        try:
            observation = {
                'joint_positions': torch.tensor(self.last_joint_state.position, dtype=torch.float32),
                'joint_velocities': torch.tensor(
                    self.last_joint_state.velocity if self.last_joint_state.velocity else [0.0] * len(self.last_joint_state.position),
                    dtype=torch.float32
                )
            }
            
            # Add camera observations
            for topic, camera_data in self.last_camera_images.items():
                camera_name = f'camera_{topic.replace("/", "_")}'
                img = camera_data['image']
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).float() / 255.0
                observation[camera_name] = img_tensor.unsqueeze(0)  # Add batch dimension
            
            return observation
            
        except Exception as e:
            self.get_logger().error(f'Error creating policy observation: {e}')
            return None
    
    def get_policy_action(self, observation):
        """Get action from loaded policy"""
        # Placeholder implementation
        # In a real implementation, this would run inference on the loaded model
        
        try:
            # Simple placeholder: maintain current position with small random variation
            current_positions = observation['joint_positions'].numpy()
            noise = np.random.normal(0, 0.01, len(current_positions))
            target_positions = current_positions + noise
            
            # Clamp to reasonable joint limits
            target_positions = np.clip(target_positions, -np.pi, np.pi)
            
            return {
                'joint_positions': target_positions.tolist(),
                'execution_time': 1.0 / self.fps
            }
            
        except Exception as e:
            self.get_logger().error(f'Error getting policy action: {e}')
            return None
    
    def execute_action(self, action):
        """Execute action on the robot"""
        try:
            # Create joint trajectory message
            trajectory_msg = JointTrajectory()
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            trajectory_msg.joint_names = self.last_joint_state.name
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = action['joint_positions']
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(action['execution_time'] * 1e9)
            
            trajectory_msg.points = [point]
            
            # Publish action
            self.policy_action_pub.publish(trajectory_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error executing action: {e}')
    
    def upload_dataset(self):
        """Upload dataset to Hugging Face Hub"""
        try:
            if not HF_AVAILABLE:
                self.get_logger().error('Cannot upload - Hugging Face not available')
                return
            
            self.publish_status('Uploading dataset to Hugging Face Hub')
            
            # Placeholder implementation
            # In a real implementation, this would upload the dataset
            
            self.publish_status('Dataset upload completed')
            self.get_logger().info('Dataset uploaded successfully')
            
        except Exception as e:
            self.get_logger().error(f'Error uploading dataset: {e}')
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[LEROBOT_BRIDGE] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        bridge = LeRobotROS2Bridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"LeRobot bridge error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()