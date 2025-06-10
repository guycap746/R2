#!/usr/bin/env python3
"""
LeRobot Policy Trainer

This module handles training of LeRobot policies using collected datasets,
including imitation learning, reinforcement learning, and model evaluation.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
import json
import time
from typing import Dict, List, Optional, Any, Tuple
from pathlib import Path
import cv2
import h5py
from threading import Lock, Thread
import logging

# ROS2 message types
from std_msgs.msg import String, Bool, Int32, Float32
from sensor_msgs.msg import Image

# LeRobot imports
try:
    from lerobot.common.policies.factory import make_policy
    from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.common.policies.act import ACT
    from lerobot.common.policies.diffusion import DiffusionPolicy
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False

# Hugging Face imports
try:
    from huggingface_hub import HfApi, Repository, login
    from transformers import get_scheduler
    HF_AVAILABLE = True
except ImportError:
    HF_AVAILABLE = False

# Visualization and logging
try:
    import wandb
    WANDB_AVAILABLE = True
except ImportError:
    WANDB_AVAILABLE = False

class PolicyType:
    """Supported policy types"""
    ACT = "act"
    DIFFUSION = "diffusion"
    TRANSFORMER = "transformer"
    CNN_LSTM = "cnn_lstm"

class TrainingPhase:
    """Training phases"""
    INITIALIZATION = "initialization"
    TRAINING = "training"
    VALIDATION = "validation"
    EVALUATION = "evaluation"
    COMPLETED = "completed"
    ERROR = "error"

class RoArmDataset(Dataset):
    """PyTorch dataset for RoArm manipulation data"""
    
    def __init__(self, data_dir: Path, transform=None):
        self.data_dir = data_dir
        self.transform = transform
        self.episodes = []
        
        # Load episode data
        self.load_episodes()
    
    def load_episodes(self):
        """Load all episodes from data directory"""
        episode_dirs = sorted(self.data_dir.glob('episode_*'))
        
        for episode_dir in episode_dirs:
            try:
                # Try to load LeRobot format first
                episode_file = episode_dir / 'episode_data.pt'
                if episode_file.exists():
                    episode_data = torch.load(episode_file, map_location='cpu')
                    self.episodes.append(episode_data)
                else:
                    # Fall back to HDF5 format
                    hdf5_file = episode_dir / 'episode_data.h5'
                    if hdf5_file.exists():
                        episode_data = self.load_hdf5_episode(hdf5_file)
                        self.episodes.append(episode_data)
                        
            except Exception as e:
                logging.warning(f"Failed to load episode from {episode_dir}: {e}")
    
    def load_hdf5_episode(self, hdf5_file):
        """Load episode from HDF5 format"""
        episode_data = {
            'observations': {},
            'actions': {},
            'rewards': None,
            'metadata': {}
        }
        
        with h5py.File(hdf5_file, 'r') as f:
            # Load observations
            obs_group = f['observations']
            for key in obs_group.keys():
                data = torch.from_numpy(obs_group[key][:]).float()
                episode_data['observations'][key] = data
            
            # Load actions
            act_group = f['actions']
            for key in act_group.keys():
                data = torch.from_numpy(act_group[key][:]).float()
                episode_data['actions'][key] = data
            
            # Load rewards
            if 'rewards' in f:
                episode_data['rewards'] = torch.from_numpy(f['rewards'][:]).float()
            
            # Load metadata
            for key in f.attrs.keys():
                episode_data['metadata'][key] = f.attrs[key]
        
        return episode_data
    
    def __len__(self):
        return sum(len(ep['observations']['joint_positions']) for ep in self.episodes)
    
    def __getitem__(self, idx):
        # Find which episode and step this index corresponds to
        current_idx = 0
        for episode in self.episodes:
            episode_length = len(episode['observations']['joint_positions'])
            if current_idx + episode_length > idx:
                step_idx = idx - current_idx
                return self.get_episode_step(episode, step_idx)
            current_idx += episode_length
        
        raise IndexError("Index out of range")
    
    def get_episode_step(self, episode, step_idx):
        """Get a single step from an episode"""
        observation = {}
        action = {}
        
        # Get observations
        for key, data in episode['observations'].items():
            observation[key] = data[step_idx]
        
        # Get actions
        for key, data in episode['actions'].items():
            action[key] = data[step_idx]
        
        # Get reward
        reward = episode['rewards'][step_idx] if episode['rewards'] is not None else 0.0
        
        if self.transform:
            observation, action = self.transform(observation, action)
        
        return observation, action, reward

class LeRobotPolicyTrainer(Node):
    """Policy trainer for LeRobot models"""
    
    def __init__(self):
        super().__init__('lerobot_policy_trainer')
        
        # Parameters
        self.declare_parameter('data_dir', '/tmp/lerobot_data')
        self.declare_parameter('model_dir', '/tmp/lerobot_models')
        self.declare_parameter('policy_type', 'act')
        self.declare_parameter('batch_size', 32)
        self.declare_parameter('learning_rate', 1e-4)
        self.declare_parameter('num_epochs', 100)
        self.declare_parameter('validation_split', 0.2)
        self.declare_parameter('save_every_n_epochs', 10)
        self.declare_parameter('enable_wandb', False)
        self.declare_parameter('wandb_project', 'roarm_lerobot')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('num_workers', 4)
        
        # Get parameters
        self.data_dir = Path(self.get_parameter('data_dir').get_parameter_value().string_value)
        self.model_dir = Path(self.get_parameter('model_dir').get_parameter_value().string_value)
        self.policy_type = self.get_parameter('policy_type').get_parameter_value().string_value
        self.batch_size = self.get_parameter('batch_size').get_parameter_value().integer_value
        self.learning_rate = self.get_parameter('learning_rate').get_parameter_value().double_value
        self.num_epochs = self.get_parameter('num_epochs').get_parameter_value().integer_value
        self.validation_split = self.get_parameter('validation_split').get_parameter_value().double_value
        self.save_every_n_epochs = self.get_parameter('save_every_n_epochs').get_parameter_value().integer_value
        self.enable_wandb = self.get_parameter('enable_wandb').get_parameter_value().bool_value
        self.wandb_project = self.get_parameter('wandb_project').get_parameter_value().string_value
        self.device_param = self.get_parameter('device').get_parameter_value().string_value
        self.num_workers = self.get_parameter('num_workers').get_parameter_value().integer_value
        
        # Setup device
        if self.device_param == 'auto':
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        else:
            self.device = torch.device(self.device_param)
        
        # Create directories
        self.model_dir.mkdir(parents=True, exist_ok=True)
        
        # Training state
        self.training_active = False
        self.current_epoch = 0
        self.training_thread = None
        self.training_lock = Lock()
        
        # Model components
        self.model = None
        self.optimizer = None
        self.scheduler = None
        self.train_loader = None
        self.val_loader = None
        
        # Training statistics
        self.training_stats = {
            'epoch': 0,
            'train_loss': 0.0,
            'val_loss': 0.0,
            'best_val_loss': float('inf'),
            'training_time': 0.0,
            'samples_seen': 0
        }
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/lerobot/trainer_status', 10)
        self.training_stats_pub = self.create_publisher(String, '/lerobot/training_stats', 10)
        self.training_progress_pub = self.create_publisher(Float32, '/lerobot/training_progress', 10)
        
        # Subscribers
        self.start_training_sub = self.create_subscription(
            String, '/lerobot/start_training', self.start_training_callback, 10)
        self.stop_training_sub = self.create_subscription(
            Bool, '/lerobot/stop_training', self.stop_training_callback, 10)
        self.save_model_sub = self.create_subscription(
            Bool, '/lerobot/save_model', self.save_model_callback, 10)
        
        # Initialize components
        self.initialize_wandb()
        
        self.get_logger().info('LeRobot Policy Trainer initialized')
        self.publish_status('Trainer ready')
    
    def initialize_wandb(self):
        """Initialize Weights & Biases logging"""
        if self.enable_wandb and WANDB_AVAILABLE:
            try:
                wandb.init(
                    project=self.wandb_project,
                    config={
                        'policy_type': self.policy_type,
                        'batch_size': self.batch_size,
                        'learning_rate': self.learning_rate,
                        'num_epochs': self.num_epochs,
                        'device': str(self.device)
                    }
                )
                self.get_logger().info('Weights & Biases initialized')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize W&B: {e}')
    
    def start_training_callback(self, msg):
        """Handle start training request"""
        config_str = msg.data
        try:
            config = json.loads(config_str) if config_str else {}
            self.start_training(config)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid training configuration JSON')
    
    def stop_training_callback(self, msg):
        """Handle stop training request"""
        if msg.data:
            self.stop_training()
    
    def save_model_callback(self, msg):
        """Handle save model request"""
        if msg.data:
            self.save_model()
    
    def start_training(self, config: Dict = None):
        """Start training process"""
        if self.training_active:
            self.get_logger().warning('Training already in progress')
            return
        
        # Update configuration if provided
        if config:
            self.update_config(config)
        
        # Start training in separate thread
        self.training_thread = Thread(target=self.training_loop)
        self.training_thread.daemon = True
        self.training_thread.start()
        
        self.publish_status('Training started')
        self.get_logger().info('Training process started')
    
    def stop_training(self):
        """Stop training process"""
        with self.training_lock:
            self.training_active = False
        
        if self.training_thread and self.training_thread.is_alive():
            self.training_thread.join(timeout=5.0)
        
        self.publish_status('Training stopped')
        self.get_logger().info('Training process stopped')
    
    def update_config(self, config: Dict):
        """Update training configuration"""
        if 'batch_size' in config:
            self.batch_size = config['batch_size']
        if 'learning_rate' in config:
            self.learning_rate = config['learning_rate']
        if 'num_epochs' in config:
            self.num_epochs = config['num_epochs']
        if 'policy_type' in config:
            self.policy_type = config['policy_type']
    
    def training_loop(self):
        """Main training loop"""
        try:
            with self.training_lock:
                self.training_active = True
            
            self.publish_status('Initializing training')
            
            # Load dataset
            if not self.load_dataset():
                self.publish_status('Failed to load dataset')
                return
            
            # Initialize model
            if not self.initialize_model():
                self.publish_status('Failed to initialize model')
                return
            
            # Training loop
            self.publish_status(f'Starting training for {self.num_epochs} epochs')
            start_time = time.time()
            
            for epoch in range(self.num_epochs):
                if not self.training_active:
                    break
                
                self.current_epoch = epoch
                
                # Training phase
                train_loss = self.train_epoch()
                
                # Validation phase
                val_loss = self.validate_epoch()
                
                # Update statistics
                self.training_stats.update({
                    'epoch': epoch,
                    'train_loss': train_loss,
                    'val_loss': val_loss,
                    'training_time': time.time() - start_time
                })
                
                # Save best model
                if val_loss < self.training_stats['best_val_loss']:
                    self.training_stats['best_val_loss'] = val_loss
                    self.save_model(is_best=True)
                
                # Periodic save
                if (epoch + 1) % self.save_every_n_epochs == 0:
                    self.save_model()
                
                # Publish statistics
                self.publish_training_stats()
                
                # Log to W&B
                if self.enable_wandb and WANDB_AVAILABLE:
                    wandb.log({
                        'epoch': epoch,
                        'train_loss': train_loss,
                        'val_loss': val_loss,
                        'learning_rate': self.optimizer.param_groups[0]['lr']
                    })
                
                # Update learning rate
                if self.scheduler:
                    self.scheduler.step()
                
                # Publish progress
                progress = (epoch + 1) / self.num_epochs
                progress_msg = Float32()
                progress_msg.data = progress
                self.training_progress_pub.publish(progress_msg)
                
                self.get_logger().info(f'Epoch {epoch+1}/{self.num_epochs} - Train Loss: {train_loss:.4f}, Val Loss: {val_loss:.4f}')
            
            # Final save
            self.save_model(is_final=True)
            
            with self.training_lock:
                self.training_active = False
            
            self.publish_status('Training completed successfully')
            
        except Exception as e:
            self.get_logger().error(f'Training failed: {e}')
            self.publish_status(f'Training failed: {e}')
            with self.training_lock:
                self.training_active = False
    
    def load_dataset(self):
        """Load and prepare dataset"""
        try:
            self.publish_status('Loading dataset')
            
            # Create dataset
            dataset = RoArmDataset(self.data_dir)
            
            if len(dataset) == 0:
                self.get_logger().error('No data found in dataset')
                return False
            
            # Split dataset
            dataset_size = len(dataset)
            val_size = int(self.validation_split * dataset_size)
            train_size = dataset_size - val_size
            
            train_dataset, val_dataset = torch.utils.data.random_split(
                dataset, [train_size, val_size])
            
            # Create data loaders
            self.train_loader = DataLoader(
                train_dataset,
                batch_size=self.batch_size,
                shuffle=True,
                num_workers=self.num_workers,
                collate_fn=self.collate_fn
            )
            
            self.val_loader = DataLoader(
                val_dataset,
                batch_size=self.batch_size,
                shuffle=False,
                num_workers=self.num_workers,
                collate_fn=self.collate_fn
            )
            
            self.get_logger().info(f'Dataset loaded: {train_size} train, {val_size} val samples')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to load dataset: {e}')
            return False
    
    def collate_fn(self, batch):
        """Custom collate function for batching"""
        observations = {}
        actions = {}
        rewards = []
        
        # Extract data from batch
        for obs, action, reward in batch:
            rewards.append(reward)
            
            # Stack observations
            for key, value in obs.items():
                if key not in observations:
                    observations[key] = []
                observations[key].append(value)
            
            # Stack actions
            for key, value in action.items():
                if key not in actions:
                    actions[key] = []
                actions[key].append(value)
        
        # Convert to tensors
        for key in observations:
            observations[key] = torch.stack(observations[key])
        
        for key in actions:
            actions[key] = torch.stack(actions[key])
        
        rewards = torch.tensor(rewards, dtype=torch.float32)
        
        return observations, actions, rewards
    
    def initialize_model(self):
        """Initialize the policy model"""
        try:
            self.publish_status(f'Initializing {self.policy_type} model')
            
            if self.policy_type == PolicyType.ACT:
                self.model = self.create_act_model()
            elif self.policy_type == PolicyType.DIFFUSION:
                self.model = self.create_diffusion_model()
            elif self.policy_type == PolicyType.CNN_LSTM:
                self.model = self.create_cnn_lstm_model()
            else:
                self.get_logger().error(f'Unsupported policy type: {self.policy_type}')
                return False
            
            # Move to device
            self.model = self.model.to(self.device)
            
            # Initialize optimizer
            self.optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)
            
            # Initialize scheduler
            self.scheduler = get_scheduler(
                "cosine",
                optimizer=self.optimizer,
                num_warmup_steps=100,
                num_training_steps=self.num_epochs * len(self.train_loader)
            ) if HF_AVAILABLE else None
            
            self.get_logger().info(f'{self.policy_type} model initialized')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize model: {e}')
            return False
    
    def create_act_model(self):
        """Create ACT (Action Chunking Transformer) model"""
        # Simplified ACT-like model for demonstration
        class SimpleACT(nn.Module):
            def __init__(self, state_dim=6, action_dim=6, hidden_dim=256):
                super().__init__()
                self.state_dim = state_dim
                self.action_dim = action_dim
                
                # Encoder for state
                self.state_encoder = nn.Sequential(
                    nn.Linear(state_dim, hidden_dim),
                    nn.ReLU(),
                    nn.Linear(hidden_dim, hidden_dim),
                    nn.ReLU()
                )
                
                # Decoder for actions
                self.action_decoder = nn.Sequential(
                    nn.Linear(hidden_dim, hidden_dim),
                    nn.ReLU(),
                    nn.Linear(hidden_dim, action_dim)
                )
            
            def forward(self, observations):
                # Simple joint state prediction
                joint_positions = observations['joint_positions']
                
                # Encode current state
                encoded = self.state_encoder(joint_positions)
                
                # Decode next action
                actions = self.action_decoder(encoded)
                
                return {'joint_positions': actions}
        
        return SimpleACT()
    
    def create_diffusion_model(self):
        """Create diffusion policy model"""
        # Placeholder for diffusion model
        return self.create_act_model()  # Use ACT as fallback
    
    def create_cnn_lstm_model(self):
        """Create CNN-LSTM model for visual policies"""
        class CNNLSTM(nn.Module):
            def __init__(self, state_dim=6, action_dim=6, hidden_dim=256):
                super().__init__()
                
                # CNN for image processing
                self.cnn = nn.Sequential(
                    nn.Conv2d(3, 32, 3, stride=2, padding=1),
                    nn.ReLU(),
                    nn.Conv2d(32, 64, 3, stride=2, padding=1),
                    nn.ReLU(),
                    nn.Conv2d(64, 128, 3, stride=2, padding=1),
                    nn.ReLU(),
                    nn.AdaptiveAvgPool2d((4, 4))
                )
                
                # LSTM for temporal processing
                self.lstm = nn.LSTM(128 * 16 + state_dim, hidden_dim, batch_first=True)
                
                # Action decoder
                self.action_decoder = nn.Linear(hidden_dim, action_dim)
            
            def forward(self, observations):
                batch_size = observations['joint_positions'].shape[0]
                
                # Process images if available
                if 'rgb_image' in observations:
                    images = observations['rgb_image']
                    cnn_features = self.cnn(images)
                    cnn_features = cnn_features.view(batch_size, -1)
                else:
                    cnn_features = torch.zeros(batch_size, 128 * 16, device=observations['joint_positions'].device)
                
                # Combine with joint state
                joint_positions = observations['joint_positions']
                combined_features = torch.cat([cnn_features, joint_positions], dim=1)
                
                # LSTM processing
                lstm_out, _ = self.lstm(combined_features.unsqueeze(1))
                
                # Decode actions
                actions = self.action_decoder(lstm_out.squeeze(1))
                
                return {'joint_positions': actions}
        
        return CNNLSTM()
    
    def train_epoch(self):
        """Train for one epoch"""
        self.model.train()
        total_loss = 0.0
        num_batches = 0
        
        for batch_idx, (observations, actions, rewards) in enumerate(self.train_loader):
            if not self.training_active:
                break
            
            # Move to device
            observations = {k: v.to(self.device) for k, v in observations.items()}
            actions = {k: v.to(self.device) for k, v in actions.items()}
            
            # Forward pass
            self.optimizer.zero_grad()
            predicted_actions = self.model(observations)
            
            # Calculate loss (MSE for joint positions)
            loss = nn.MSELoss()(predicted_actions['joint_positions'], actions['joint_positions'])
            
            # Backward pass
            loss.backward()
            self.optimizer.step()
            
            total_loss += loss.item()
            num_batches += 1
            
            # Update samples seen
            self.training_stats['samples_seen'] += observations['joint_positions'].shape[0]
        
        return total_loss / max(num_batches, 1)
    
    def validate_epoch(self):
        """Validate for one epoch"""
        self.model.eval()
        total_loss = 0.0
        num_batches = 0
        
        with torch.no_grad():
            for observations, actions, rewards in self.val_loader:
                if not self.training_active:
                    break
                
                # Move to device
                observations = {k: v.to(self.device) for k, v in observations.items()}
                actions = {k: v.to(self.device) for k, v in actions.items()}
                
                # Forward pass
                predicted_actions = self.model(observations)
                
                # Calculate loss
                loss = nn.MSELoss()(predicted_actions['joint_positions'], actions['joint_positions'])
                
                total_loss += loss.item()
                num_batches += 1
        
        return total_loss / max(num_batches, 1)
    
    def save_model(self, is_best=False, is_final=False):
        """Save model checkpoint"""
        try:
            checkpoint = {
                'epoch': self.current_epoch,
                'model_state_dict': self.model.state_dict(),
                'optimizer_state_dict': self.optimizer.state_dict(),
                'training_stats': self.training_stats,
                'config': {
                    'policy_type': self.policy_type,
                    'batch_size': self.batch_size,
                    'learning_rate': self.learning_rate
                }
            }
            
            if self.scheduler:
                checkpoint['scheduler_state_dict'] = self.scheduler.state_dict()
            
            # Save regular checkpoint
            checkpoint_path = self.model_dir / f'checkpoint_epoch_{self.current_epoch:04d}.pt'
            torch.save(checkpoint, checkpoint_path)
            
            # Save best model
            if is_best:
                best_path = self.model_dir / 'best_model.pt'
                torch.save(checkpoint, best_path)
                self.get_logger().info(f'Best model saved: {best_path}')
            
            # Save final model
            if is_final:
                final_path = self.model_dir / 'final_model.pt'
                torch.save(checkpoint, final_path)
                self.get_logger().info(f'Final model saved: {final_path}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save model: {e}')
    
    def publish_training_stats(self):
        """Publish training statistics"""
        stats_msg = String()
        stats_msg.data = json.dumps(self.training_stats, indent=2)
        self.training_stats_pub.publish(stats_msg)
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[LEROBOT_TRAINER] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        trainer = LeRobotPolicyTrainer()
        rclpy.spin(trainer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Policy trainer error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()