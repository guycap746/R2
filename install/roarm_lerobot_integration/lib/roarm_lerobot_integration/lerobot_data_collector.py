#!/usr/bin/env python3
"""
LeRobot Data Collector

This module handles data collection for training LeRobot policies,
including teleoperation recording, demonstration collection, and dataset management.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import torch
import json
import time
import h5py
from typing import Dict, List, Optional, Any
from pathlib import Path
import cv2
from threading import Lock
from datetime import datetime

# ROS2 message types
from std_msgs.msg import String, Bool, Int32, Float32
from sensor_msgs.msg import Image, JointState, Joy
from geometry_msgs.msg import Pose, PoseStamped, Twist
from trajectory_msgs.msg import JointTrajectory
from cv_bridge import CvBridge

# LeRobot imports
try:
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.common.datasets.utils import create_lerobot_dataset
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False

class DataCollectionMode:
    """Data collection modes"""
    TELEOPERATION = "teleoperation"
    DEMONSTRATION = "demonstration"
    AUTONOMOUS = "autonomous"
    EVALUATION = "evaluation"

class LeRobotDataCollector(Node):
    """Data collector for LeRobot training datasets"""
    
    def __init__(self):
        super().__init__('lerobot_data_collector')
        
        # Parameters
        self.declare_parameter('dataset_name', 'roarm_manipulation')
        self.declare_parameter('data_dir', '/tmp/lerobot_data')
        self.declare_parameter('fps', 30)
        self.declare_parameter('max_episodes', 100)
        self.declare_parameter('max_episode_steps', 1000)
        self.declare_parameter('auto_save_interval', 10)
        self.declare_parameter('enable_compression', True)
        self.declare_parameter('collection_mode', 'teleoperation')
        
        # Get parameters
        self.dataset_name = self.get_parameter('dataset_name').get_parameter_value().string_value
        self.data_dir = Path(self.get_parameter('data_dir').get_parameter_value().string_value)
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.max_episodes = self.get_parameter('max_episodes').get_parameter_value().integer_value
        self.max_episode_steps = self.get_parameter('max_episode_steps').get_parameter_value().integer_value
        self.auto_save_interval = self.get_parameter('auto_save_interval').get_parameter_value().integer_value
        self.enable_compression = self.get_parameter('enable_compression').get_parameter_value().bool_value
        self.collection_mode = self.get_parameter('collection_mode').get_parameter_value().string_value
        
        # Create data directory
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Data storage
        self.episodes_data = []
        self.current_episode = {
            'observations': [],
            'actions': [],
            'rewards': [],
            'episode_index': 0,
            'start_time': None,
            'end_time': None,
            'metadata': {}
        }
        
        # State tracking
        self.collecting = False
        self.episode_step = 0
        self.total_episodes = 0
        self.data_lock = Lock()
        
        # Sensor data buffers
        self.current_joint_state = None
        self.current_images = {}
        self.current_pose = None
        self.current_joy = None
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/lerobot/collector_status', 10)
        self.dataset_stats_pub = self.create_publisher(String, '/lerobot/dataset_stats', 10)
        self.episode_progress_pub = self.create_publisher(Float32, '/lerobot/episode_progress', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        
        # Control subscribers
        self.start_collection_sub = self.create_subscription(
            Bool, '/lerobot/start_collection', self.start_collection_callback, 10)
        self.stop_collection_sub = self.create_subscription(
            Bool, '/lerobot/stop_collection', self.stop_collection_callback, 10)
        self.save_episode_sub = self.create_subscription(
            Bool, '/lerobot/save_episode', self.save_episode_callback, 10)
        self.reset_dataset_sub = self.create_subscription(
            Bool, '/lerobot/reset_dataset', self.reset_dataset_callback, 10)
        
        # Timers
        self.collection_timer = None
        self.auto_save_timer = self.create_timer(
            self.auto_save_interval, self.auto_save_check)
        
        self.get_logger().info('LeRobot Data Collector initialized')
        self.publish_status('Data collector ready')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        with self.data_lock:
            self.current_joint_state = msg
    
    def image_callback(self, msg):
        """Handle RGB image updates"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.data_lock:
                self.current_images['rgb'] = {
                    'image': cv_image,
                    'timestamp': msg.header.stamp,
                    'width': msg.width,
                    'height': msg.height
                }
        except Exception as e:
            self.get_logger().error(f'RGB image callback error: {e}')
    
    def depth_callback(self, msg):
        """Handle depth image updates"""
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
            with self.data_lock:
                self.current_images['depth'] = {
                    'image': depth_image,
                    'timestamp': msg.header.stamp,
                    'width': msg.width,
                    'height': msg.height
                }
        except Exception as e:
            self.get_logger().error(f'Depth image callback error: {e}')
    
    def pose_callback(self, msg):
        """Handle robot pose updates"""
        with self.data_lock:
            self.current_pose = msg
    
    def joy_callback(self, msg):
        """Handle joystick/gamepad input for teleoperation"""
        with self.data_lock:
            self.current_joy = msg
            
            # Check for collection control buttons
            if len(msg.buttons) > 0:
                # Button 0: Start/stop collection
                if msg.buttons[0] == 1:
                    if not self.collecting:
                        self.start_collection()
                    else:
                        self.stop_collection()
                
                # Button 1: Save current episode
                if len(msg.buttons) > 1 and msg.buttons[1] == 1:
                    self.save_current_episode()
    
    def start_collection_callback(self, msg):
        """Handle start collection request"""
        if msg.data:
            self.start_collection()
    
    def stop_collection_callback(self, msg):
        """Handle stop collection request"""
        if msg.data:
            self.stop_collection()
    
    def save_episode_callback(self, msg):
        """Handle save episode request"""
        if msg.data:
            self.save_current_episode()
    
    def reset_dataset_callback(self, msg):
        """Handle reset dataset request"""
        if msg.data:
            self.reset_dataset()
    
    def start_collection(self):
        """Start data collection"""
        if self.collecting:
            self.get_logger().warning('Collection already in progress')
            return
        
        if self.total_episodes >= self.max_episodes:
            self.get_logger().warning(f'Maximum episodes ({self.max_episodes}) reached')
            return
        
        self.collecting = True
        self.episode_step = 0
        
        # Reset current episode
        self.current_episode = {
            'observations': [],
            'actions': [],
            'rewards': [],
            'episode_index': self.total_episodes,
            'start_time': time.time(),
            'end_time': None,
            'metadata': {
                'collection_mode': self.collection_mode,
                'fps': self.fps,
                'max_steps': self.max_episode_steps
            }
        }
        
        # Start collection timer
        self.collection_timer = self.create_timer(1.0 / self.fps, self.collect_data_step)
        
        self.publish_status(f'Started collecting episode {self.total_episodes}')
        self.get_logger().info(f'Data collection started - Episode {self.total_episodes}')
    
    def stop_collection(self):
        """Stop data collection"""
        if not self.collecting:
            return
        
        self.collecting = False
        
        # Stop collection timer
        if self.collection_timer:
            self.collection_timer.destroy()
            self.collection_timer = None
        
        # Finalize episode
        self.current_episode['end_time'] = time.time()
        self.current_episode['metadata']['actual_steps'] = self.episode_step
        self.current_episode['metadata']['duration'] = (
            self.current_episode['end_time'] - self.current_episode['start_time']
        )
        
        self.publish_status(f'Stopped collecting episode {self.total_episodes} - {self.episode_step} steps')
        self.get_logger().info(f'Data collection stopped - {self.episode_step} steps recorded')
    
    def collect_data_step(self):
        """Collect one step of data"""
        try:
            if not self.collecting:
                return
            
            # Check if episode should end
            if self.episode_step >= self.max_episode_steps:
                self.stop_collection()
                return
            
            # Collect observation
            observation = self.create_observation()
            if observation is None:
                return
            
            # Collect action
            action = self.create_action()
            if action is None:
                return
            
            # Calculate reward (placeholder)
            reward = self.calculate_reward(observation, action)
            
            # Store data
            with self.data_lock:
                self.current_episode['observations'].append(observation)
                self.current_episode['actions'].append(action)
                self.current_episode['rewards'].append(reward)
            
            self.episode_step += 1
            
            # Publish progress
            progress = self.episode_step / self.max_episode_steps
            progress_msg = Float32()
            progress_msg.data = progress
            self.episode_progress_pub.publish(progress_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in data collection step: {e}')
    
    def create_observation(self):
        """Create observation from current sensor data"""
        try:
            if not self.current_joint_state:
                return None
            
            observation = {
                'timestamp': time.time(),
                'step': self.episode_step,
                'joint_positions': list(self.current_joint_state.position),
                'joint_velocities': list(self.current_joint_state.velocity) if self.current_joint_state.velocity else [0.0] * len(self.current_joint_state.position),
                'joint_efforts': list(self.current_joint_state.effort) if self.current_joint_state.effort else [0.0] * len(self.current_joint_state.position),
            }
            
            # Add images
            for img_type, img_data in self.current_images.items():
                observation[f'{img_type}_image'] = img_data['image'].copy()
                observation[f'{img_type}_timestamp'] = (
                    img_data['timestamp'].sec + img_data['timestamp'].nanosec * 1e-9
                )
            
            # Add pose if available
            if self.current_pose:
                observation['end_effector_pose'] = {
                    'position': [
                        self.current_pose.pose.position.x,
                        self.current_pose.pose.position.y,
                        self.current_pose.pose.position.z
                    ],
                    'orientation': [
                        self.current_pose.pose.orientation.x,
                        self.current_pose.pose.orientation.y,
                        self.current_pose.pose.orientation.z,
                        self.current_pose.pose.orientation.w
                    ]
                }
            
            return observation
            
        except Exception as e:
            self.get_logger().error(f'Error creating observation: {e}')
            return None
    
    def create_action(self):
        """Create action from current control input"""
        try:
            action = {
                'timestamp': time.time(),
                'step': self.episode_step,
                'type': self.collection_mode
            }
            
            if self.collection_mode == DataCollectionMode.TELEOPERATION:
                # Use joystick input to create action
                if self.current_joy and self.current_joint_state:
                    # Simple mapping: use axes to modify joint positions
                    current_positions = list(self.current_joint_state.position)
                    
                    if len(self.current_joy.axes) >= 6:
                        # Map joystick axes to joint velocities
                        joint_velocities = [axis * 0.1 for axis in self.current_joy.axes[:6]]  # Scale down
                        target_positions = [
                            pos + vel * (1.0 / self.fps) 
                            for pos, vel in zip(current_positions, joint_velocities)
                        ]
                    else:
                        target_positions = current_positions
                    
                    action['joint_positions'] = target_positions
                    action['joint_velocities'] = joint_velocities if 'joint_velocities' in locals() else [0.0] * len(current_positions)
                else:
                    # No input, maintain current position
                    if self.current_joint_state:
                        action['joint_positions'] = list(self.current_joint_state.position)
                        action['joint_velocities'] = [0.0] * len(self.current_joint_state.position)
                    else:
                        return None
            
            elif self.collection_mode == DataCollectionMode.DEMONSTRATION:
                # Use current joint state as demonstration
                if self.current_joint_state:
                    action['joint_positions'] = list(self.current_joint_state.position)
                    action['joint_velocities'] = list(self.current_joint_state.velocity) if self.current_joint_state.velocity else [0.0] * len(self.current_joint_state.position)
                else:
                    return None
            
            else:
                # Other modes - placeholder
                if self.current_joint_state:
                    action['joint_positions'] = list(self.current_joint_state.position)
                    action['joint_velocities'] = [0.0] * len(self.current_joint_state.position)
                else:
                    return None
            
            return action
            
        except Exception as e:
            self.get_logger().error(f'Error creating action: {e}')
            return None
    
    def calculate_reward(self, observation, action):
        """Calculate reward for the current step (placeholder)"""
        # Placeholder reward calculation
        # In a real implementation, this would be task-specific
        
        try:
            # Simple reward: penalize large joint velocities (encourage smooth motion)
            if 'joint_velocities' in action:
                velocity_penalty = -0.01 * sum(abs(v) for v in action['joint_velocities'])
                return max(0.0, 1.0 + velocity_penalty)
            else:
                return 1.0
                
        except Exception as e:
            self.get_logger().error(f'Error calculating reward: {e}')
            return 0.0
    
    def save_current_episode(self):
        """Save the current episode to storage"""
        if not self.current_episode['observations']:
            self.get_logger().warning('No data to save in current episode')
            return
        
        try:
            # Add episode to dataset
            self.episodes_data.append(self.current_episode.copy())
            self.total_episodes += 1
            
            # Save to disk
            self.save_episode_to_disk(self.current_episode)
            
            # Publish statistics
            self.publish_dataset_stats()
            
            self.publish_status(f'Episode {self.current_episode["episode_index"]} saved - {len(self.current_episode["observations"])} steps')
            self.get_logger().info(f'Episode {self.current_episode["episode_index"]} saved successfully')
            
        except Exception as e:
            self.get_logger().error(f'Error saving episode: {e}')
    
    def save_episode_to_disk(self, episode):
        """Save episode data to disk"""
        try:
            episode_dir = self.data_dir / f'episode_{episode["episode_index"]:06d}'
            episode_dir.mkdir(exist_ok=True)
            
            # Save metadata
            metadata_file = episode_dir / 'metadata.json'
            with open(metadata_file, 'w') as f:
                json.dump(episode['metadata'], f, indent=2)
            
            # Save observations and actions
            if LEROBOT_AVAILABLE:
                # Save in LeRobot format
                self.save_lerobot_format(episode, episode_dir)
            else:
                # Save in HDF5 format
                self.save_hdf5_format(episode, episode_dir)
            
        except Exception as e:
            self.get_logger().error(f'Error saving episode to disk: {e}')
    
    def save_lerobot_format(self, episode, episode_dir):
        """Save episode in LeRobot dataset format"""
        try:
            # Convert to tensors
            num_steps = len(episode['observations'])
            
            # Process observations
            observations = {}
            if num_steps > 0:
                # Joint data
                joint_positions = torch.tensor([obs['joint_positions'] for obs in episode['observations']], dtype=torch.float32)
                joint_velocities = torch.tensor([obs['joint_velocities'] for obs in episode['observations']], dtype=torch.float32)
                
                observations['joint_positions'] = joint_positions
                observations['joint_velocities'] = joint_velocities
                
                # Images
                for img_type in ['rgb', 'depth']:
                    if f'{img_type}_image' in episode['observations'][0]:
                        images = []
                        for obs in episode['observations']:
                            img = obs[f'{img_type}_image']
                            if img_type == 'rgb':
                                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                                img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).float() / 255.0
                            else:  # depth
                                img_tensor = torch.from_numpy(img).float()
                                if len(img_tensor.shape) == 2:
                                    img_tensor = img_tensor.unsqueeze(0)  # Add channel dimension
                            images.append(img_tensor)
                        
                        if images:
                            observations[f'{img_type}_image'] = torch.stack(images)
            
            # Process actions
            actions = {}
            if episode['actions']:
                action_positions = torch.tensor([action['joint_positions'] for action in episode['actions']], dtype=torch.float32)
                action_velocities = torch.tensor([action['joint_velocities'] for action in episode['actions']], dtype=torch.float32)
                
                actions['joint_positions'] = action_positions
                actions['joint_velocities'] = action_velocities
            
            # Save episode data
            episode_data = {
                'observations': observations,
                'actions': actions,
                'rewards': torch.tensor(episode['rewards'], dtype=torch.float32),
                'episode_index': episode['episode_index'],
                'metadata': episode['metadata']
            }
            
            torch.save(episode_data, episode_dir / 'episode_data.pt')
            
        except Exception as e:
            self.get_logger().error(f'Error saving LeRobot format: {e}')
    
    def save_hdf5_format(self, episode, episode_dir):
        """Save episode in HDF5 format"""
        try:
            hdf5_file = episode_dir / 'episode_data.h5'
            
            with h5py.File(hdf5_file, 'w') as f:
                # Observations group
                obs_group = f.create_group('observations')
                
                if episode['observations']:
                    # Joint data
                    joint_positions = np.array([obs['joint_positions'] for obs in episode['observations']])
                    joint_velocities = np.array([obs['joint_velocities'] for obs in episode['observations']])
                    
                    obs_group.create_dataset('joint_positions', data=joint_positions, compression='gzip' if self.enable_compression else None)
                    obs_group.create_dataset('joint_velocities', data=joint_velocities, compression='gzip' if self.enable_compression else None)
                    
                    # Images
                    for img_type in ['rgb', 'depth']:
                        if f'{img_type}_image' in episode['observations'][0]:
                            images = np.array([obs[f'{img_type}_image'] for obs in episode['observations']])
                            obs_group.create_dataset(f'{img_type}_image', data=images, compression='gzip' if self.enable_compression else None)
                
                # Actions group
                act_group = f.create_group('actions')
                
                if episode['actions']:
                    action_positions = np.array([action['joint_positions'] for action in episode['actions']])
                    action_velocities = np.array([action['joint_velocities'] for action in episode['actions']])
                    
                    act_group.create_dataset('joint_positions', data=action_positions, compression='gzip' if self.enable_compression else None)
                    act_group.create_dataset('joint_velocities', data=action_velocities, compression='gzip' if self.enable_compression else None)
                
                # Rewards
                f.create_dataset('rewards', data=np.array(episode['rewards']), compression='gzip' if self.enable_compression else None)
                
                # Metadata as attributes
                for key, value in episode['metadata'].items():
                    f.attrs[key] = value
            
        except Exception as e:
            self.get_logger().error(f'Error saving HDF5 format: {e}')
    
    def auto_save_check(self):
        """Periodic auto-save check"""
        if self.collecting and self.episode_step > 0:
            # Auto-save every N steps
            if self.episode_step % (self.auto_save_interval * self.fps) == 0:
                self.get_logger().info(f'Auto-save checkpoint at step {self.episode_step}')
    
    def reset_dataset(self):
        """Reset the entire dataset"""
        with self.data_lock:
            self.episodes_data.clear()
            self.total_episodes = 0
            
            if self.collecting:
                self.stop_collection()
        
        self.publish_status('Dataset reset')
        self.get_logger().info('Dataset has been reset')
    
    def publish_dataset_stats(self):
        """Publish dataset statistics"""
        try:
            stats = {
                'total_episodes': self.total_episodes,
                'total_steps': sum(len(ep['observations']) for ep in self.episodes_data),
                'total_duration': sum(ep['metadata'].get('duration', 0) for ep in self.episodes_data),
                'collection_mode': self.collection_mode,
                'data_dir': str(self.data_dir),
                'dataset_size_mb': self.calculate_dataset_size(),
                'average_episode_length': np.mean([len(ep['observations']) for ep in self.episodes_data]) if self.episodes_data else 0
            }
            
            stats_msg = String()
            stats_msg.data = json.dumps(stats, indent=2)
            self.dataset_stats_pub.publish(stats_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing dataset stats: {e}')
    
    def calculate_dataset_size(self):
        """Calculate approximate dataset size in MB"""
        try:
            total_size = 0
            for episode_dir in self.data_dir.glob('episode_*'):
                for file_path in episode_dir.rglob('*'):
                    if file_path.is_file():
                        total_size += file_path.stat().st_size
            
            return total_size / (1024 * 1024)  # Convert to MB
            
        except Exception:
            return 0.0
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[LEROBOT_COLLECTOR] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        collector = LeRobotDataCollector()
        rclpy.spin(collector)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Data collector error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()