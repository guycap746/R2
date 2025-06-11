#!/usr/bin/env python3
"""
LeRobot Dataset Manager

This module handles dataset management operations including
Hugging Face Hub integration, dataset validation, and storage optimization.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import json
import time
import h5py
from typing import Dict, List, Optional, Any
from pathlib import Path
import shutil
import zipfile
from datetime import datetime

# ROS2 message types
from std_msgs.msg import String, Bool, Float32
from roarm_lerobot_integration.srv import ValidateDataset

# Try to import Hugging Face Hub
try:
    from huggingface_hub import HfApi, upload_file, snapshot_download
    HF_HUB_AVAILABLE = True
except ImportError:
    HF_HUB_AVAILABLE = False

class DatasetManagerState:
    """Dataset manager states"""
    IDLE = "idle"
    UPLOADING = "uploading"
    DOWNLOADING = "downloading"
    VALIDATING = "validating"
    PROCESSING = "processing"
    ERROR = "error"

class LeRobotDatasetManager(Node):
    """Dataset management for LeRobot integration"""
    
    def __init__(self):
        super().__init__('lerobot_dataset_manager')
        
        # Parameters
        self.declare_parameter('data_dir', '/tmp/lerobot_data')
        self.declare_parameter('hf_token', '')
        self.declare_parameter('dataset_repo', 'roarm/manipulation_dataset')
        self.declare_parameter('auto_upload', False)
        self.declare_parameter('upload_interval', 3600)  # 1 hour
        self.declare_parameter('compression_level', 6)
        
        # Get parameters
        self.data_dir = Path(self.get_parameter('data_dir').get_parameter_value().string_value)
        self.hf_token = self.get_parameter('hf_token').get_parameter_value().string_value
        self.dataset_repo = self.get_parameter('dataset_repo').get_parameter_value().string_value
        self.auto_upload = self.get_parameter('auto_upload').get_parameter_value().bool_value
        self.upload_interval = self.get_parameter('upload_interval').get_parameter_value().integer_value
        self.compression_level = self.get_parameter('compression_level').get_parameter_value().integer_value
        
        # State variables
        self.manager_state = DatasetManagerState.IDLE
        self.hf_api = None
        
        # Dataset statistics
        self.dataset_stats = {
            'total_episodes': 0,
            'total_steps': 0,
            'total_size_mb': 0.0,
            'last_upload': None,
            'validation_status': 'unknown'
        }
        
        # Initialize Hugging Face API
        if HF_HUB_AVAILABLE and self.hf_token:
            try:
                self.hf_api = HfApi(token=self.hf_token)
                self.get_logger().info('Hugging Face Hub API initialized')
            except Exception as e:
                self.get_logger().warning(f'Failed to initialize HF API: {e}')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/lerobot/dataset_manager_status', 10)
        self.stats_pub = self.create_publisher(String, '/lerobot/dataset_stats', 10)
        
        # Subscribers
        self.upload_request_sub = self.create_subscription(
            String, '/lerobot/upload_dataset', self.upload_dataset_callback, 10)
        self.download_request_sub = self.create_subscription(
            String, '/lerobot/download_dataset', self.download_dataset_callback, 10)
        self.validate_request_sub = self.create_subscription(
            Bool, '/lerobot/validate_dataset', self.validate_dataset_callback, 10)
        
        # Services
        self.validate_service = self.create_service(
            ValidateDataset, '/lerobot/validate_dataset_service', self.validate_dataset_service)
        
        # Timers
        self.stats_timer = self.create_timer(10.0, self.update_dataset_stats)
        
        if self.auto_upload:
            self.upload_timer = self.create_timer(self.upload_interval, self.auto_upload_callback)
        
        # Ensure data directory exists
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        self.get_logger().info('LeRobot Dataset Manager initialized')
        self.publish_status('Dataset manager ready')
        self.update_dataset_stats()
    
    def upload_dataset_callback(self, msg):
        """Handle dataset upload request"""
        dataset_path = msg.data if msg.data else str(self.data_dir)
        self.upload_to_hub(dataset_path)
    
    def download_dataset_callback(self, msg):
        """Handle dataset download request"""
        repo_id = msg.data if msg.data else self.dataset_repo
        self.download_from_hub(repo_id)
    
    def validate_dataset_callback(self, msg):
        """Handle dataset validation request"""
        if msg.data:
            self.validate_dataset()
    
    def auto_upload_callback(self):
        """Auto upload timer callback"""
        if self.manager_state == DatasetManagerState.IDLE:
            self.upload_to_hub(str(self.data_dir))
    
    def upload_to_hub(self, dataset_path: str):
        """Upload dataset to Hugging Face Hub"""
        if not HF_HUB_AVAILABLE:
            self.publish_status('Hugging Face Hub not available')
            return False
        
        if not self.hf_api:
            self.publish_status('Hugging Face API not initialized')
            return False
        
        try:
            self.manager_state = DatasetManagerState.UPLOADING
            self.publish_status(f'Starting upload to {self.dataset_repo}')
            
            dataset_path = Path(dataset_path)
            if not dataset_path.exists():
                raise FileNotFoundError(f'Dataset path not found: {dataset_path}')
            
            # Create compressed archive
            archive_path = self.create_dataset_archive(dataset_path)
            
            # Upload to Hub
            self.get_logger().info(f'Uploading {archive_path} to {self.dataset_repo}')
            
            upload_file(
                path_or_fileobj=str(archive_path),
                path_in_repo=f"dataset_{datetime.now().strftime('%Y%m%d_%H%M%S')}.zip",
                repo_id=self.dataset_repo,
                token=self.hf_token,
                repo_type="dataset"
            )
            
            # Clean up temporary archive
            archive_path.unlink()
            
            self.dataset_stats['last_upload'] = datetime.now().isoformat()
            self.manager_state = DatasetManagerState.IDLE
            self.publish_status('Dataset upload completed successfully')
            return True
            
        except Exception as e:
            self.manager_state = DatasetManagerState.ERROR
            self.publish_status(f'Upload failed: {e}')
            self.get_logger().error(f'Dataset upload failed: {e}')
            return False
    
    def download_from_hub(self, repo_id: str):
        """Download dataset from Hugging Face Hub"""
        if not HF_HUB_AVAILABLE:
            self.publish_status('Hugging Face Hub not available')
            return False
        
        try:
            self.manager_state = DatasetManagerState.DOWNLOADING
            self.publish_status(f'Starting download from {repo_id}')
            
            # Download dataset
            download_path = snapshot_download(
                repo_id=repo_id,
                repo_type="dataset",
                local_dir=str(self.data_dir / "downloaded"),
                token=self.hf_token
            )
            
            self.manager_state = DatasetManagerState.IDLE
            self.publish_status(f'Dataset downloaded to {download_path}')
            return True
            
        except Exception as e:
            self.manager_state = DatasetManagerState.ERROR
            self.publish_status(f'Download failed: {e}')
            self.get_logger().error(f'Dataset download failed: {e}')
            return False
    
    def create_dataset_archive(self, dataset_path: Path) -> Path:
        """Create compressed archive of dataset"""
        archive_path = dataset_path.parent / f"dataset_{datetime.now().strftime('%Y%m%d_%H%M%S')}.zip"
        
        with zipfile.ZipFile(archive_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=self.compression_level) as zipf:
            for file_path in dataset_path.rglob('*'):
                if file_path.is_file():
                    arcname = file_path.relative_to(dataset_path)
                    zipf.write(file_path, arcname)
        
        return archive_path
    
    def validate_dataset(self):
        """Validate dataset integrity and format"""
        try:
            self.manager_state = DatasetManagerState.VALIDATING
            self.publish_status('Starting dataset validation')
            
            validation_results = {
                'valid': True,
                'errors': [],
                'warnings': [],
                'statistics': {}
            }
            
            # Check if data directory exists
            if not self.data_dir.exists():
                validation_results['valid'] = False
                validation_results['errors'].append('Data directory does not exist')
                return validation_results
            
            # Validate episodes
            episode_files = list(self.data_dir.glob('episode_*.hdf5'))
            if not episode_files:
                validation_results['warnings'].append('No episode files found')
            
            valid_episodes = 0
            total_steps = 0
            
            for episode_file in episode_files:
                try:
                    with h5py.File(episode_file, 'r') as f:
                        # Check required groups
                        required_groups = ['observations', 'actions']
                        for group in required_groups:
                            if group not in f:
                                validation_results['errors'].append(f'Missing {group} group in {episode_file.name}')
                                validation_results['valid'] = False
                        
                        # Count steps
                        if 'actions' in f and 'joint_positions' in f['actions']:
                            steps = len(f['actions']['joint_positions'])
                            total_steps += steps
                        
                        valid_episodes += 1
                        
                except Exception as e:
                    validation_results['errors'].append(f'Error reading {episode_file.name}: {e}')
                    validation_results['valid'] = False
            
            validation_results['statistics'] = {
                'total_episodes': len(episode_files),
                'valid_episodes': valid_episodes,
                'total_steps': total_steps
            }
            
            # Update dataset stats
            self.dataset_stats['validation_status'] = 'valid' if validation_results['valid'] else 'invalid'
            
            self.manager_state = DatasetManagerState.IDLE
            status = 'valid' if validation_results['valid'] else 'invalid'
            self.publish_status(f'Dataset validation completed - {status}')
            
            return validation_results
            
        except Exception as e:
            self.manager_state = DatasetManagerState.ERROR
            self.publish_status(f'Validation failed: {e}')
            self.get_logger().error(f'Dataset validation failed: {e}')
            return {'valid': False, 'errors': [str(e)]}
    
    def validate_dataset_service(self, request, response):
        """Service callback for dataset validation"""
        try:
            data_dir = Path(request.data_dir) if request.data_dir else self.data_dir
            
            # Temporarily change data dir for validation
            original_data_dir = self.data_dir
            self.data_dir = data_dir
            
            validation_results = self.validate_dataset()
            
            # Restore original data dir
            self.data_dir = original_data_dir
            
            response.success = validation_results['valid']
            response.message = json.dumps(validation_results)
            
        except Exception as e:
            response.success = False
            response.message = f'Service error: {e}'
        
        return response
    
    def update_dataset_stats(self):
        """Update dataset statistics"""
        try:
            if not self.data_dir.exists():
                return
            
            # Count episodes and steps
            episode_files = list(self.data_dir.glob('episode_*.hdf5'))
            total_episodes = len(episode_files)
            total_steps = 0
            total_size = 0
            
            for episode_file in episode_files:
                try:
                    total_size += episode_file.stat().st_size
                    
                    with h5py.File(episode_file, 'r') as f:
                        if 'actions' in f and 'joint_positions' in f['actions']:
                            total_steps += len(f['actions']['joint_positions'])
                            
                except Exception:
                    continue
            
            # Update statistics
            self.dataset_stats.update({
                'total_episodes': total_episodes,
                'total_steps': total_steps,
                'total_size_mb': total_size / (1024 * 1024)
            })
            
            # Publish statistics
            stats_msg = String()
            stats_msg.data = json.dumps(self.dataset_stats, indent=2)
            self.stats_pub.publish(stats_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to update dataset stats: {e}')
    
    def optimize_storage(self):
        """Optimize dataset storage"""
        try:
            self.manager_state = DatasetManagerState.PROCESSING
            self.publish_status('Starting storage optimization')
            
            episode_files = list(self.data_dir.glob('episode_*.hdf5'))
            
            for episode_file in episode_files:
                # Create optimized copy
                temp_file = episode_file.with_suffix('.tmp')
                
                with h5py.File(episode_file, 'r') as src, h5py.File(temp_file, 'w') as dst:
                    # Copy with compression
                    for key in src.keys():
                        src.copy(key, dst, name=key)
                        if isinstance(dst[key], h5py.Dataset):
                            # Recompress with optimal settings
                            data = dst[key][...]
                            del dst[key]
                            dst.create_dataset(key, data=data, compression='gzip', compression_opts=9)
                
                # Replace original file
                temp_file.replace(episode_file)
            
            self.manager_state = DatasetManagerState.IDLE
            self.publish_status('Storage optimization completed')
            
        except Exception as e:
            self.manager_state = DatasetManagerState.ERROR
            self.publish_status(f'Storage optimization failed: {e}')
            self.get_logger().error(f'Storage optimization failed: {e}')
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[LEROBOT_DATASET_MANAGER] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        manager = LeRobotDatasetManager()
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Dataset manager error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()