#!/usr/bin/env python3
"""
LeRobot-AnyGrasp Integration

This module provides seamless integration between LeRobot AI policies and AnyGrasp
classical grasp planning, creating a hybrid approach for robust manipulation.
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
from enum import Enum

# ROS2 message types
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, JointState, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped, Point
from roarm_anygrasp_integration.srv import GetGraspCandidates, SelectGrasp
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge

class HybridMode(Enum):
    """Hybrid execution modes"""
    LEROBOT_ONLY = "lerobot_only"
    ANYGRASP_ONLY = "anygrasp_only"
    HYBRID_VALIDATE = "hybrid_validate"  # AnyGrasp proposes, LeRobot validates
    HYBRID_ENHANCE = "hybrid_enhance"    # LeRobot executes, AnyGrasp refines
    ADAPTIVE = "adaptive"                # Dynamic switching based on confidence

class GraspCandidate:
    """Grasp candidate representation"""
    def __init__(self, pose: Pose, score: float, approach_vector: np.ndarray):
        self.pose = pose
        self.score = score
        self.approach_vector = approach_vector
        self.lerobot_confidence = 0.0
        self.hybrid_score = 0.0

class LeRobotAnyGraspIntegration(Node):
    """Hybrid LeRobot + AnyGrasp manipulation system"""
    
    def __init__(self):
        super().__init__('lerobot_anygrasp_integration')
        
        # Parameters
        self.declare_parameter('hybrid_mode', 'hybrid_validate')
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('anygrasp_weight', 0.4)
        self.declare_parameter('lerobot_weight', 0.6)
        self.declare_parameter('max_grasp_candidates', 10)
        self.declare_parameter('execution_timeout', 30.0)
        self.declare_parameter('enable_learning', True)
        
        # Get parameters
        self.hybrid_mode = HybridMode(self.get_parameter('hybrid_mode').get_parameter_value().string_value)
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.anygrasp_weight = self.get_parameter('anygrasp_weight').get_parameter_value().double_value
        self.lerobot_weight = self.get_parameter('lerobot_weight').get_parameter_value().double_value
        self.max_grasp_candidates = self.get_parameter('max_grasp_candidates').get_parameter_value().integer_value
        self.execution_timeout = self.get_parameter('execution_timeout').get_parameter_value().double_value
        self.enable_learning = self.get_parameter('enable_learning').get_parameter_value().bool_value
        
        # Initialize CV bridge and device
        self.cv_bridge = CvBridge()
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.data_lock = Lock()
        
        # State variables
        self.current_joint_state = None
        self.current_rgb_image = None
        self.current_depth_image = None
        self.current_pointcloud = None
        self.lerobot_policy = None
        
        # Grasp execution state
        self.grasp_candidates = []
        self.selected_grasp = None
        self.execution_active = False
        
        # Performance tracking
        self.execution_history = []
        self.performance_stats = {
            'lerobot_only': {'attempts': 0, 'successes': 0},
            'anygrasp_only': {'attempts': 0, 'successes': 0},
            'hybrid': {'attempts': 0, 'successes': 0}
        }
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/lerobot/hybrid_status', 10)
        self.grasp_viz_pub = self.create_publisher(Image, '/lerobot/grasp_visualization', 10)
        self.performance_pub = self.create_publisher(String, '/lerobot/hybrid_performance', 10)
        self.action_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.rgb_image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_image_callback, 10)
        self.depth_image_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_image_callback, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.pointcloud_callback, 10)
        
        # Control subscribers
        self.execute_grasp_sub = self.create_subscription(
            String, '/lerobot/execute_hybrid_grasp', self.execute_grasp_callback, 10)
        self.load_policy_sub = self.create_subscription(
            String, '/lerobot/load_hybrid_policy', self.load_policy_callback, 10)
        
        # Service clients for AnyGrasp
        self.grasp_candidates_client = self.create_client(GetGraspCandidates, '/anygrasp/get_grasp_candidates')
        self.select_grasp_client = self.create_client(SelectGrasp, '/anygrasp/select_grasp')
        
        # Timers
        self.performance_timer = self.create_timer(30.0, self.publish_performance_stats)
        
        self.get_logger().info(f'LeRobot-AnyGrasp Integration initialized in {self.hybrid_mode.value} mode')
        self.publish_status('Hybrid system ready')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        with self.data_lock:
            self.current_joint_state = msg
    
    def rgb_image_callback(self, msg):
        """Handle RGB image updates"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.data_lock:
                self.current_rgb_image = cv_image
        except Exception as e:
            self.get_logger().error(f'RGB image callback error: {e}')
    
    def depth_image_callback(self, msg):
        """Handle depth image updates"""
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, "passthrough")
            with self.data_lock:
                self.current_depth_image = depth_image
        except Exception as e:
            self.get_logger().error(f'Depth image callback error: {e}')
    
    def pointcloud_callback(self, msg):
        """Handle point cloud updates"""
        with self.data_lock:
            self.current_pointcloud = msg
    
    def load_policy_callback(self, msg):
        """Handle policy loading request"""
        policy_path = msg.data
        self.load_lerobot_policy(policy_path)
    
    def execute_grasp_callback(self, msg):
        """Handle grasp execution request"""
        try:
            request_data = json.loads(msg.data)
            target_object = request_data.get('target_object', 'unknown')
            self.execute_hybrid_grasp(target_object)
        except Exception as e:
            self.get_logger().error(f'Failed to parse grasp request: {e}')
    
    def load_lerobot_policy(self, policy_path: str) -> bool:
        """Load LeRobot policy for hybrid execution"""
        try:
            self.publish_status(f'Loading LeRobot policy: {policy_path}')
            
            # Load policy checkpoint
            if policy_path.startswith('/') or policy_path.startswith('./'):
                model_file = Path(policy_path)
            else:
                model_file = Path('/tmp/lerobot_models') / policy_path
            
            if not model_file.exists():
                raise FileNotFoundError(f'Policy file not found: {model_file}')
            
            checkpoint = torch.load(model_file, map_location=self.device)
            self.lerobot_policy = self.create_policy_from_checkpoint(checkpoint)
            
            self.publish_status('LeRobot policy loaded successfully')
            return True
            
        except Exception as e:
            self.publish_status(f'Failed to load LeRobot policy: {e}')
            self.get_logger().error(f'Policy loading failed: {e}')
            return False
    
    def create_policy_from_checkpoint(self, checkpoint):
        """Create policy model from checkpoint"""
        try:
            import torch.nn as nn
            
            class HybridACT(nn.Module):
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
                    self.confidence_head = nn.Sequential(
                        nn.Linear(hidden_dim, hidden_dim // 2),
                        nn.ReLU(),
                        nn.Linear(hidden_dim // 2, 1),
                        nn.Sigmoid()
                    )
                
                def forward(self, observations):
                    joint_positions = observations['joint_positions']
                    encoded = self.state_encoder(joint_positions)
                    actions = self.action_decoder(encoded)
                    confidence = self.confidence_head(encoded)
                    return {'joint_positions': actions, 'confidence': confidence}
            
            model = HybridACT()
            
            # Load state dict if available
            if 'model_state_dict' in checkpoint:
                model.load_state_dict(checkpoint['model_state_dict'])
            
            model.to(self.device)
            model.eval()
            
            return {
                'model': model,
                'type': checkpoint.get('config', {}).get('policy_type', 'act'),
                'config': checkpoint.get('config', {})
            }
            
        except Exception as e:
            self.get_logger().error(f'Failed to create policy from checkpoint: {e}')
            raise
    
    def execute_hybrid_grasp(self, target_object: str):
        """Execute hybrid grasp using both AnyGrasp and LeRobot"""
        if self.execution_active:
            self.publish_status('Execution already active')
            return
        
        try:
            self.execution_active = True
            self.publish_status(f'Starting hybrid grasp execution for {target_object}')
            
            # Phase 1: Get AnyGrasp candidates
            anygrasp_candidates = self.get_anygrasp_candidates(target_object)
            if not anygrasp_candidates:
                self.publish_status('No grasp candidates from AnyGrasp')
                self.execution_active = False
                return
            
            # Phase 2: Apply hybrid strategy
            if self.hybrid_mode == HybridMode.LEROBOT_ONLY:
                success = self.execute_lerobot_only(target_object)
            elif self.hybrid_mode == HybridMode.ANYGRASP_ONLY:
                success = self.execute_anygrasp_only(anygrasp_candidates)
            elif self.hybrid_mode == HybridMode.HYBRID_VALIDATE:
                success = self.execute_hybrid_validate(anygrasp_candidates)
            elif self.hybrid_mode == HybridMode.HYBRID_ENHANCE:
                success = self.execute_hybrid_enhance(anygrasp_candidates)
            elif self.hybrid_mode == HybridMode.ADAPTIVE:
                success = self.execute_adaptive_hybrid(anygrasp_candidates)
            else:
                success = False
            
            # Phase 3: Record performance
            self.record_execution_result(success, self.hybrid_mode.value)
            
            status = "successful" if success else "failed"
            self.publish_status(f'Hybrid grasp execution {status}')
            
        except Exception as e:
            self.publish_status(f'Hybrid grasp execution failed: {e}')
            self.get_logger().error(f'Hybrid execution error: {e}')
        finally:
            self.execution_active = False
    
    def get_anygrasp_candidates(self, target_object: str) -> List[GraspCandidate]:
        """Get grasp candidates from AnyGrasp"""
        try:
            if not self.grasp_candidates_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning('AnyGrasp service not available')
                return []
            
            request = GetGraspCandidates.Request()
            request.target_object = target_object
            request.max_candidates = self.max_grasp_candidates
            
            future = self.grasp_candidates_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                candidates = []
                
                for i, pose in enumerate(response.grasp_poses):
                    score = response.grasp_scores[i] if i < len(response.grasp_scores) else 0.5
                    approach = np.array([0, 0, -1])  # Default approach vector
                    
                    candidate = GraspCandidate(pose, score, approach)
                    candidates.append(candidate)
                
                self.get_logger().info(f'Received {len(candidates)} grasp candidates from AnyGrasp')
                return candidates
            
            return []
            
        except Exception as e:
            self.get_logger().error(f'Failed to get AnyGrasp candidates: {e}')
            return []
    
    def execute_lerobot_only(self, target_object: str) -> bool:
        """Execute using only LeRobot policy"""
        if not self.lerobot_policy:
            self.get_logger().warning('No LeRobot policy loaded')
            return False
        
        try:
            self.publish_status('Executing with LeRobot only')
            
            # Create observation
            observation = self.create_lerobot_observation()
            if observation is None:
                return False
            
            # Run inference
            with torch.no_grad():
                output = self.lerobot_policy['model'](observation)
                joint_positions = output['joint_positions'].cpu().numpy()[0]
                confidence = output.get('confidence', torch.tensor([0.5])).cpu().item()
            
            # Execute action if confidence is sufficient
            if confidence >= self.confidence_threshold:
                self.execute_joint_trajectory(joint_positions.tolist())
                return True
            else:
                self.get_logger().warning(f'LeRobot confidence {confidence} below threshold {self.confidence_threshold}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'LeRobot-only execution failed: {e}')
            return False
    
    def execute_anygrasp_only(self, candidates: List[GraspCandidate]) -> bool:
        """Execute using only AnyGrasp"""
        try:
            self.publish_status('Executing with AnyGrasp only')
            
            if not candidates:
                return False
            
            # Select best candidate by score
            best_candidate = max(candidates, key=lambda c: c.score)
            
            # Convert to joint trajectory via inverse kinematics (simplified)
            joint_positions = self.pose_to_joint_positions(best_candidate.pose)
            
            if joint_positions:
                self.execute_joint_trajectory(joint_positions)
                return True
            
            return False
            
        except Exception as e:
            self.get_logger().error(f'AnyGrasp-only execution failed: {e}')
            return False
    
    def execute_hybrid_validate(self, candidates: List[GraspCandidate]) -> bool:
        """Execute hybrid approach: AnyGrasp proposes, LeRobot validates"""
        try:
            self.publish_status('Executing hybrid validation approach')
            
            if not self.lerobot_policy or not candidates:
                return False
            
            # Evaluate each candidate with LeRobot
            for candidate in candidates:
                # Create observation with candidate pose
                observation = self.create_lerobot_observation_with_target(candidate.pose)
                
                if observation is not None:
                    with torch.no_grad():
                        output = self.lerobot_policy['model'](observation)
                        confidence = output.get('confidence', torch.tensor([0.5])).cpu().item()
                        candidate.lerobot_confidence = confidence
                        
                        # Compute hybrid score
                        candidate.hybrid_score = (
                            self.anygrasp_weight * candidate.score + 
                            self.lerobot_weight * confidence
                        )
            
            # Select best hybrid candidate
            best_candidate = max(candidates, key=lambda c: c.hybrid_score)
            
            if best_candidate.hybrid_score >= self.confidence_threshold:
                joint_positions = self.pose_to_joint_positions(best_candidate.pose)
                if joint_positions:
                    self.execute_joint_trajectory(joint_positions)
                    return True
            
            return False
            
        except Exception as e:
            self.get_logger().error(f'Hybrid validation failed: {e}')
            return False
    
    def execute_hybrid_enhance(self, candidates: List[GraspCandidate]) -> bool:
        """Execute hybrid approach: LeRobot executes, AnyGrasp refines"""
        try:
            self.publish_status('Executing hybrid enhancement approach')
            
            # First, get LeRobot action
            lerobot_success = self.execute_lerobot_only("hybrid_target")
            
            if not lerobot_success and candidates:
                # Fallback to AnyGrasp if LeRobot fails
                self.get_logger().info('LeRobot failed, falling back to AnyGrasp')
                return self.execute_anygrasp_only(candidates)
            
            return lerobot_success
            
        except Exception as e:
            self.get_logger().error(f'Hybrid enhancement failed: {e}')
            return False
    
    def execute_adaptive_hybrid(self, candidates: List[GraspCandidate]) -> bool:
        """Execute adaptive approach: dynamically choose best method"""
        try:
            self.publish_status('Executing adaptive hybrid approach')
            
            # Analyze current performance stats to choose method
            lerobot_success_rate = self.get_success_rate('lerobot_only')
            anygrasp_success_rate = self.get_success_rate('anygrasp_only') 
            hybrid_success_rate = self.get_success_rate('hybrid')
            
            # Choose best performing method
            if hybrid_success_rate >= max(lerobot_success_rate, anygrasp_success_rate):
                return self.execute_hybrid_validate(candidates)
            elif lerobot_success_rate >= anygrasp_success_rate:
                return self.execute_lerobot_only("adaptive_target")
            else:
                return self.execute_anygrasp_only(candidates)
                
        except Exception as e:
            self.get_logger().error(f'Adaptive hybrid failed: {e}')
            return False
    
    def create_lerobot_observation(self) -> Optional[Dict]:
        """Create observation for LeRobot inference"""
        try:
            if not self.current_joint_state:
                return None
            
            observation = {
                'joint_positions': torch.tensor(
                    self.current_joint_state.position,
                    dtype=torch.float32,
                    device=self.device
                ).unsqueeze(0)
            }
            
            # Add visual information if available
            if self.current_rgb_image is not None:
                img_rgb = cv2.cvtColor(self.current_rgb_image, cv2.COLOR_BGR2RGB)
                img_tensor = torch.from_numpy(img_rgb).permute(2, 0, 1).float() / 255.0
                observation['rgb_image'] = img_tensor.unsqueeze(0).to(self.device)
            
            return observation
            
        except Exception as e:
            self.get_logger().error(f'Failed to create LeRobot observation: {e}')
            return None
    
    def create_lerobot_observation_with_target(self, target_pose: Pose) -> Optional[Dict]:
        """Create observation with target pose information"""
        observation = self.create_lerobot_observation()
        
        if observation is not None:
            # Add target pose as additional input
            target_tensor = torch.tensor([
                target_pose.position.x,
                target_pose.position.y, 
                target_pose.position.z,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w
            ], dtype=torch.float32, device=self.device).unsqueeze(0)
            
            observation['target_pose'] = target_tensor
        
        return observation
    
    def pose_to_joint_positions(self, pose: Pose) -> Optional[List[float]]:
        """Convert pose to joint positions via inverse kinematics (simplified)"""
        # This is a placeholder implementation
        # In a real system, this would use proper inverse kinematics
        try:
            if self.current_joint_state:
                # Simple approach: modify current joint positions slightly
                current_positions = list(self.current_joint_state.position)
                
                # Add small modifications based on target pose
                target_positions = current_positions.copy()
                target_positions[0] += pose.position.x * 0.1
                target_positions[1] += pose.position.y * 0.1
                target_positions[2] += pose.position.z * 0.1
                
                return target_positions
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'Pose to joint conversion failed: {e}')
            return None
    
    def execute_joint_trajectory(self, joint_positions: List[float]):
        """Execute joint trajectory"""
        try:
            if not self.current_joint_state:
                return
            
            trajectory_msg = JointTrajectory()
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            trajectory_msg.joint_names = list(self.current_joint_state.name)
            
            point = JointTrajectoryPoint()
            point.positions = joint_positions[:len(trajectory_msg.joint_names)]
            point.time_from_start.sec = 2  # 2 second execution
            point.time_from_start.nanosec = 0
            
            trajectory_msg.points = [point]
            self.action_pub.publish(trajectory_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to execute joint trajectory: {e}')
    
    def record_execution_result(self, success: bool, method: str):
        """Record execution result for performance tracking"""
        if method in self.performance_stats:
            self.performance_stats[method]['attempts'] += 1
            if success:
                self.performance_stats[method]['successes'] += 1
        
        # Store in execution history
        self.execution_history.append({
            'timestamp': time.time(),
            'method': method,
            'success': success,
            'hybrid_mode': self.hybrid_mode.value
        })
        
        # Keep only recent history
        if len(self.execution_history) > 100:
            self.execution_history = self.execution_history[-100:]
    
    def get_success_rate(self, method: str) -> float:
        """Get success rate for a specific method"""
        if method in self.performance_stats:
            stats = self.performance_stats[method]
            if stats['attempts'] > 0:
                return stats['successes'] / stats['attempts']
        return 0.0
    
    def publish_performance_stats(self):
        """Publish performance statistics"""
        try:
            stats = {
                'performance_by_method': {},
                'recent_executions': len(self.execution_history),
                'current_mode': self.hybrid_mode.value
            }
            
            for method, data in self.performance_stats.items():
                success_rate = self.get_success_rate(method)
                stats['performance_by_method'][method] = {
                    'attempts': data['attempts'],
                    'successes': data['successes'],
                    'success_rate': success_rate
                }
            
            performance_msg = String()
            performance_msg.data = json.dumps(stats, indent=2)
            self.performance_pub.publish(performance_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish performance stats: {e}')
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[LEROBOT_ANYGRASP] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        integration = LeRobotAnyGraspIntegration()
        rclpy.spin(integration)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"LeRobot-AnyGrasp integration error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()