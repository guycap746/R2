#!/usr/bin/env python3
"""
LeRobot Evaluation Node

This module provides comprehensive evaluation capabilities for trained LeRobot policies,
including performance assessment, success rate analysis, and comparison metrics.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import torch
import json
import time
from typing import Dict, List, Optional, Any, Tuple
from pathlib import Path
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
import cv2

# ROS2 message types
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Pose
from roarm_lerobot_integration.srv import EvaluatePolicy

class EvaluationState:
    """Evaluation states"""
    IDLE = "idle"
    LOADING_POLICY = "loading_policy"
    RUNNING_EVALUATION = "running_evaluation"
    ANALYZING_RESULTS = "analyzing_results"
    GENERATING_REPORT = "generating_report"
    ERROR = "error"

class TaskEvaluator:
    """Individual task evaluator"""
    
    def __init__(self, task_name: str, success_criteria: Dict):
        self.task_name = task_name
        self.success_criteria = success_criteria
        self.evaluation_history = []
    
    def evaluate_episode(self, episode_data: Dict) -> Dict:
        """Evaluate a single episode"""
        result = {
            'task_name': self.task_name,
            'success': False,
            'metrics': {},
            'timestamp': time.time()
        }
        
        try:
            # Position-based success criteria
            if 'final_position' in self.success_criteria:
                target_pos = self.success_criteria['final_position']
                actual_pos = episode_data.get('final_joint_positions', [])
                
                if len(actual_pos) >= len(target_pos):
                    position_error = np.linalg.norm(np.array(actual_pos[:len(target_pos)]) - np.array(target_pos))
                    result['metrics']['position_error'] = position_error
                    
                    tolerance = self.success_criteria.get('position_tolerance', 0.1)
                    if position_error < tolerance:
                        result['success'] = True
            
            # Time-based criteria
            if 'max_duration' in self.success_criteria:
                episode_duration = episode_data.get('duration', 0)
                result['metrics']['duration'] = episode_duration
                
                if episode_duration > self.success_criteria['max_duration']:
                    result['success'] = False
            
            # Smoothness evaluation
            if 'joint_trajectory' in episode_data:
                trajectory = np.array(episode_data['joint_trajectory'])
                if len(trajectory) > 1:
                    # Calculate path smoothness (acceleration variance)
                    velocities = np.diff(trajectory, axis=0)
                    accelerations = np.diff(velocities, axis=0)
                    smoothness = np.mean(np.var(accelerations, axis=0))
                    result['metrics']['smoothness'] = smoothness
            
            self.evaluation_history.append(result)
            return result
            
        except Exception as e:
            result['error'] = str(e)
            return result

class LeRobotEvaluationNode(Node):
    """Comprehensive evaluation system for LeRobot policies"""
    
    def __init__(self):
        super().__init__('lerobot_evaluation_node')
        
        # Parameters
        self.declare_parameter('evaluation_config', '/tmp/evaluation_config.json')
        self.declare_parameter('results_dir', '/tmp/lerobot_evaluation')
        self.declare_parameter('policy_dir', '/tmp/lerobot_models')
        self.declare_parameter('num_evaluation_episodes', 10)
        self.declare_parameter('evaluation_timeout', 60.0)
        self.declare_parameter('generate_plots', True)
        
        # Get parameters
        self.evaluation_config_path = self.get_parameter('evaluation_config').get_parameter_value().string_value
        self.results_dir = Path(self.get_parameter('results_dir').get_parameter_value().string_value)
        self.policy_dir = Path(self.get_parameter('policy_dir').get_parameter_value().string_value)
        self.num_evaluation_episodes = self.get_parameter('num_evaluation_episodes').get_parameter_value().integer_value
        self.evaluation_timeout = self.get_parameter('evaluation_timeout').get_parameter_value().double_value
        self.generate_plots = self.get_parameter('generate_plots').get_parameter_value().bool_value
        
        # State variables
        self.evaluation_state = EvaluationState.IDLE
        self.current_evaluation = None
        self.task_evaluators = {}
        self.evaluation_results = {}
        
        # Device configuration
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Create results directory
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        # Load evaluation configuration
        self.load_evaluation_config()
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/lerobot/evaluation_status', 10)
        self.results_pub = self.create_publisher(String, '/lerobot/evaluation_results', 10)
        self.metrics_pub = self.create_publisher(String, '/lerobot/evaluation_metrics', 10)
        
        # Subscribers
        self.evaluate_policy_sub = self.create_subscription(
            String, '/lerobot/evaluate_policy', self.evaluate_policy_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Services
        self.evaluate_service = self.create_service(
            EvaluatePolicy, '/lerobot/evaluate_policy_service', self.evaluate_policy_service)
        
        # Current sensor data
        self.current_joint_state = None
        self.episode_data = {
            'start_time': None,
            'joint_trajectory': [],
            'timestamps': []
        }
        
        # Statistics
        self.evaluation_stats = {
            'total_evaluations': 0,
            'successful_evaluations': 0,
            'average_success_rate': 0.0,
            'average_completion_time': 0.0
        }
        
        self.get_logger().info('LeRobot Evaluation Node initialized')
        self.publish_status('Evaluation system ready')
    
    def load_evaluation_config(self):
        """Load evaluation configuration"""
        try:
            if Path(self.evaluation_config_path).exists():
                with open(self.evaluation_config_path, 'r') as f:
                    config = json.load(f)
                
                # Create task evaluators
                for task_config in config.get('tasks', []):
                    task_name = task_config['name']
                    success_criteria = task_config['success_criteria']
                    self.task_evaluators[task_name] = TaskEvaluator(task_name, success_criteria)
                
                self.get_logger().info(f'Loaded {len(self.task_evaluators)} task evaluators')
            else:
                # Create default configuration
                self.create_default_config()
                
        except Exception as e:
            self.get_logger().error(f'Failed to load evaluation config: {e}')
            self.create_default_config()
    
    def create_default_config(self):
        """Create default evaluation configuration"""
        default_config = {
            'tasks': [
                {
                    'name': 'pick_and_place',
                    'description': 'Pick and place manipulation task',
                    'success_criteria': {
                        'final_position': [0.0, 0.5, 0.3, 0.0, 0.0, 0.0],
                        'position_tolerance': 0.05,
                        'max_duration': 30.0
                    }
                },
                {
                    'name': 'reaching',
                    'description': 'Point-to-point reaching task',
                    'success_criteria': {
                        'final_position': [0.2, 0.3, 0.4, 0.0, 0.0, 0.0],
                        'position_tolerance': 0.02,
                        'max_duration': 15.0
                    }
                }
            ]
        }
        
        # Create task evaluators from default config
        for task_config in default_config['tasks']:
            task_name = task_config['name']
            success_criteria = task_config['success_criteria']
            self.task_evaluators[task_name] = TaskEvaluator(task_name, success_criteria)
        
        # Save default config
        try:
            with open(self.evaluation_config_path, 'w') as f:
                json.dump(default_config, f, indent=2)
        except Exception as e:
            self.get_logger().warning(f'Failed to save default config: {e}')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        self.current_joint_state = msg
        
        # Record trajectory during evaluation
        if self.evaluation_state == EvaluationState.RUNNING_EVALUATION and self.episode_data['start_time']:
            self.episode_data['joint_trajectory'].append(list(msg.position))
            self.episode_data['timestamps'].append(time.time())
    
    def evaluate_policy_callback(self, msg):
        """Handle policy evaluation request"""
        policy_info = json.loads(msg.data)
        self.start_policy_evaluation(policy_info)
    
    def evaluate_policy_service(self, request, response):
        """Service callback for policy evaluation"""
        try:
            policy_info = {
                'model_path': request.model_path,
                'num_episodes': request.num_episodes if request.num_episodes > 0 else self.num_evaluation_episodes,
                'task_name': request.task_name if request.task_name else 'default'
            }
            
            results = self.start_policy_evaluation(policy_info)
            
            response.success = results is not None
            response.results = json.dumps(results) if results else "Evaluation failed"
            
        except Exception as e:
            response.success = False
            response.results = f'Service error: {e}'
        
        return response
    
    def start_policy_evaluation(self, policy_info: Dict) -> Optional[Dict]:
        """Start policy evaluation"""
        try:
            self.evaluation_state = EvaluationState.LOADING_POLICY
            self.publish_status(f'Starting evaluation of {policy_info.get("model_path", "unknown")}')
            
            # Load policy model
            model_path = policy_info.get('model_path')
            if not model_path or not Path(model_path).exists():
                raise FileNotFoundError(f'Model not found: {model_path}')
            
            # Set up evaluation
            num_episodes = policy_info.get('num_episodes', self.num_evaluation_episodes)
            task_name = policy_info.get('task_name', 'default')
            
            if task_name not in self.task_evaluators:
                self.get_logger().warning(f'Unknown task: {task_name}, using default evaluator')
                task_name = list(self.task_evaluators.keys())[0] if self.task_evaluators else 'default'
            
            # Run evaluation episodes
            self.evaluation_state = EvaluationState.RUNNING_EVALUATION
            evaluation_results = self.run_evaluation_episodes(num_episodes, task_name)
            
            # Analyze results
            self.evaluation_state = EvaluationState.ANALYZING_RESULTS
            analysis = self.analyze_evaluation_results(evaluation_results)
            
            # Generate report
            if self.generate_plots:
                self.evaluation_state = EvaluationState.GENERATING_REPORT
                self.generate_evaluation_report(analysis, policy_info)
            
            self.evaluation_state = EvaluationState.IDLE
            self.publish_status('Evaluation completed successfully')
            
            # Publish results
            results_msg = String()
            results_msg.data = json.dumps(analysis, indent=2)
            self.results_pub.publish(results_msg)
            
            return analysis
            
        except Exception as e:
            self.evaluation_state = EvaluationState.ERROR
            self.publish_status(f'Evaluation failed: {e}')
            self.get_logger().error(f'Policy evaluation failed: {e}')
            return None
    
    def run_evaluation_episodes(self, num_episodes: int, task_name: str) -> List[Dict]:
        """Run multiple evaluation episodes"""
        results = []
        evaluator = self.task_evaluators.get(task_name)
        
        if not evaluator:
            raise ValueError(f'No evaluator found for task: {task_name}')
        
        for episode_idx in range(num_episodes):
            self.publish_status(f'Running evaluation episode {episode_idx + 1}/{num_episodes}')
            
            # Reset episode data
            self.episode_data = {
                'start_time': time.time(),
                'joint_trajectory': [],
                'timestamps': []
            }
            
            # Simulate policy execution (placeholder)
            # In a real implementation, this would trigger policy execution
            episode_result = self.simulate_episode_execution(episode_idx)
            
            # Evaluate episode
            evaluation = evaluator.evaluate_episode(episode_result)
            results.append(evaluation)
            
            self.get_logger().info(f'Episode {episode_idx + 1} - Success: {evaluation["success"]}')
        
        return results
    
    def simulate_episode_execution(self, episode_idx: int) -> Dict:
        """Simulate episode execution (placeholder implementation)"""
        # This is a placeholder - in real implementation, this would:
        # 1. Load and execute the policy
        # 2. Monitor robot execution
        # 3. Record actual trajectory and outcomes
        
        start_time = time.time()
        
        # Simulate some execution time
        time.sleep(0.1)
        
        # Generate simulated results
        if self.current_joint_state:
            current_positions = list(self.current_joint_state.position)
        else:
            current_positions = [0.0] * 6
        
        # Add some random variation to simulate different outcomes
        noise = np.random.normal(0, 0.01, len(current_positions))
        final_positions = np.array(current_positions) + noise
        
        return {
            'episode_idx': episode_idx,
            'final_joint_positions': final_positions.tolist(),
            'duration': time.time() - start_time,
            'joint_trajectory': self.episode_data['joint_trajectory'] or [current_positions],
            'success_probability': 0.7 + 0.3 * np.random.random()  # Simulated success rate
        }
    
    def analyze_evaluation_results(self, results: List[Dict]) -> Dict:
        """Analyze evaluation results"""
        if not results:
            return {'error': 'No results to analyze'}
        
        analysis = {
            'summary': {},
            'metrics': {},
            'episodes': results,
            'timestamp': datetime.now().isoformat()
        }
        
        # Calculate summary statistics
        successful_episodes = [r for r in results if r.get('success', False)]
        success_rate = len(successful_episodes) / len(results)
        
        analysis['summary'] = {
            'total_episodes': len(results),
            'successful_episodes': len(successful_episodes),
            'success_rate': success_rate,
            'task_name': results[0].get('task_name', 'unknown')
        }
        
        # Calculate detailed metrics
        if successful_episodes:
            durations = [ep.get('metrics', {}).get('duration', 0) for ep in successful_episodes]
            position_errors = [ep.get('metrics', {}).get('position_error', 0) for ep in successful_episodes]
            
            analysis['metrics'] = {
                'average_duration': np.mean(durations) if durations else 0,
                'std_duration': np.std(durations) if durations else 0,
                'average_position_error': np.mean(position_errors) if position_errors else 0,
                'std_position_error': np.std(position_errors) if position_errors else 0
            }
        
        # Update global statistics
        self.evaluation_stats['total_evaluations'] += 1
        self.evaluation_stats['successful_evaluations'] += len(successful_episodes)
        self.evaluation_stats['average_success_rate'] = (
            self.evaluation_stats['successful_evaluations'] / 
            (self.evaluation_stats['total_evaluations'] * len(results))
        )
        
        return analysis
    
    def generate_evaluation_report(self, analysis: Dict, policy_info: Dict):
        """Generate evaluation report with plots"""
        try:
            # Create report directory
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            report_dir = self.results_dir / f"evaluation_{timestamp}"
            report_dir.mkdir(exist_ok=True)
            
            # Save analysis data
            with open(report_dir / 'analysis.json', 'w') as f:
                json.dump(analysis, f, indent=2)
            
            # Generate plots if matplotlib is available
            try:
                self.create_success_rate_plot(analysis, report_dir)
                self.create_metrics_plot(analysis, report_dir)
                self.get_logger().info(f'Evaluation report saved to {report_dir}')
            except Exception as e:
                self.get_logger().warning(f'Failed to generate plots: {e}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to generate evaluation report: {e}')
    
    def create_success_rate_plot(self, analysis: Dict, report_dir: Path):
        """Create success rate visualization"""
        plt.figure(figsize=(10, 6))
        
        episodes = analysis.get('episodes', [])
        episode_numbers = range(1, len(episodes) + 1)
        success_values = [1 if ep.get('success', False) else 0 for ep in episodes]
        
        # Success rate over episodes
        plt.subplot(1, 2, 1)
        plt.plot(episode_numbers, success_values, 'bo-', alpha=0.7)
        plt.title('Episode Success Rate')
        plt.xlabel('Episode Number')
        plt.ylabel('Success (1) / Failure (0)')
        plt.grid(True, alpha=0.3)
        
        # Cumulative success rate
        plt.subplot(1, 2, 2)
        cumulative_success = np.cumsum(success_values) / np.arange(1, len(success_values) + 1)
        plt.plot(episode_numbers, cumulative_success, 'r-', linewidth=2)
        plt.title('Cumulative Success Rate')
        plt.xlabel('Episode Number')
        plt.ylabel('Success Rate')
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(report_dir / 'success_rate.png', dpi=150, bbox_inches='tight')
        plt.close()
    
    def create_metrics_plot(self, analysis: Dict, report_dir: Path):
        """Create metrics visualization"""
        episodes = analysis.get('episodes', [])
        
        # Extract metrics
        durations = [ep.get('metrics', {}).get('duration', 0) for ep in episodes]
        position_errors = [ep.get('metrics', {}).get('position_error', 0) for ep in episodes]
        
        if not durations and not position_errors:
            return
        
        plt.figure(figsize=(12, 4))
        
        if durations:
            plt.subplot(1, 2, 1)
            plt.hist(durations, bins=min(10, len(durations)), alpha=0.7, edgecolor='black')
            plt.title('Episode Duration Distribution')
            plt.xlabel('Duration (seconds)')
            plt.ylabel('Frequency')
            plt.grid(True, alpha=0.3)
        
        if position_errors:
            plt.subplot(1, 2, 2)
            plt.hist(position_errors, bins=min(10, len(position_errors)), alpha=0.7, edgecolor='black')
            plt.title('Position Error Distribution')
            plt.xlabel('Position Error')
            plt.ylabel('Frequency')
            plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(report_dir / 'metrics.png', dpi=150, bbox_inches='tight')
        plt.close()
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[LEROBOT_EVALUATION] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        evaluator = LeRobotEvaluationNode()
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Evaluation node error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()