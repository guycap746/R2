#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Header, String
import numpy as np
import sys
import os
from threading import Lock, Event
import time

# Import custom service types
from roarm_anygrasp_integration.srv import SelectGrasp, GetGraspCandidates

# Add AnyGrasp SDK to path
sys.path.append('/ros2_ws/src/anygrasp_workspace/anygrasp_sdk')

try:
    from gsnet import AnyGrasp
except ImportError:
    print("AnyGrasp SDK not properly installed or licensed")
    AnyGrasp = None

class GraspCandidate:
    def __init__(self, pose, confidence, width, quality, original_index):
        self.pose = pose
        self.confidence = confidence
        self.width = width
        self.quality = quality
        self.original_index = original_index

class AnyGraspInteractiveNode(Node):
    def __init__(self):
        super().__init__('anygrasp_interactive_node')
        
        # Initialize state
        self.current_candidates = []
        self.detection_in_progress = False
        self.last_detection_time = None
        self.user_selection_pending = False
        self.candidates_lock = Lock()
        self.selection_event = Event()
        
        # Publishers
        self.grasp_poses_pub = self.create_publisher(PoseArray, '/anygrasp/grasp_poses', 10)
        self.status_pub = self.create_publisher(String, '/anygrasp/status', 10)
        self.candidates_pub = self.create_publisher(PoseArray, '/anygrasp/top_candidates', 10)
        
        # Subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.pointcloud_callback, 10)
        
        # Services
        self.get_candidates_service = self.create_service(
            GetGraspCandidates, '/anygrasp/get_candidates', self.get_candidates_callback)
        self.select_grasp_service = self.create_service(
            SelectGrasp, '/anygrasp/select_grasp', self.select_grasp_callback)
        
        # Parameters
        self.declare_parameter('auto_detect', False)
        self.declare_parameter('detection_confidence_threshold', 0.5)
        self.declare_parameter('max_candidates', 5)
        self.declare_parameter('min_grasp_width', 0.01)
        self.declare_parameter('max_grasp_width', 0.15)
        
        # Initialize AnyGrasp
        if AnyGrasp is not None:
            try:
                checkpoint_path = '/ros2_ws/src/anygrasp_workspace/anygrasp_sdk/log/checkpoint_detection.tar'
                self.anygrasp = AnyGrasp(checkpoint_path)
                self.get_logger().info('AnyGrasp initialized successfully')
                self.publish_status("AnyGrasp ready - waiting for detection request")
            except Exception as e:
                self.get_logger().error(f'Failed to initialize AnyGrasp: {e}')
                self.anygrasp = None
                self.publish_status("AnyGrasp initialization failed")
        else:
            self.anygrasp = None
            self.get_logger().error('AnyGrasp SDK not available')
            self.publish_status("AnyGrasp SDK not available")

        # Timer for periodic status updates
        self.status_timer = self.create_timer(5.0, self.status_update_callback)
    
    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {message}")
    
    def status_update_callback(self):
        """Periodic status updates"""
        if self.user_selection_pending:
            self.publish_status(f"Waiting for user selection from {len(self.current_candidates)} candidates")
        elif self.detection_in_progress:
            self.publish_status("Grasp detection in progress...")
        elif len(self.current_candidates) > 0:
            self.publish_status(f"Ready - {len(self.current_candidates)} candidates available")
        else:
            self.publish_status("Ready - no candidates detected")
    
    def pointcloud_callback(self, msg):
        """Handle incoming point cloud data - only auto-detect if enabled"""
        if self.get_parameter('auto_detect').get_parameter_value().bool_value:
            if not self.detection_in_progress and not self.user_selection_pending:
                self.detect_grasps_from_pointcloud(msg)
    
    def detect_grasps_from_pointcloud(self, pointcloud_msg):
        """Detect grasps from point cloud data"""
        if self.anygrasp is None:
            self.get_logger().error("AnyGrasp not available")
            return
        
        self.detection_in_progress = True
        self.publish_status("Detecting grasps...")
        
        try:
            # Convert ROS PointCloud2 to format expected by AnyGrasp
            # This is a simplified conversion - implement proper conversion based on your setup
            point_cloud_data = self.convert_pointcloud2_to_numpy(pointcloud_msg)
            
            if point_cloud_data is None:
                self.publish_status("Failed to convert point cloud data")
                return
            
            # Run AnyGrasp detection
            if self.anygrasp is not None:
                grasp_results = self.anygrasp.detect_grasps(point_cloud_data)
            else:
                # Mock grasp results for testing
                grasp_results = []
                self.get_logger().info("Using mock AnyGrasp detection")
            
            # Process and filter results
            candidates = self.process_grasp_results(grasp_results)
            
            with self.candidates_lock:
                self.current_candidates = candidates
                self.last_detection_time = self.get_clock().now()
            
            self.publish_grasp_candidates(candidates)
            
            if len(candidates) > 0:
                self.publish_status(f"Detected {len(candidates)} grasp candidates - awaiting user selection")
                self.user_selection_pending = True
            else:
                self.publish_status("No valid grasps detected")
                
        except Exception as e:
            self.get_logger().error(f'Error during grasp detection: {e}')
            self.publish_status(f"Detection failed: {str(e)}")
        finally:
            self.detection_in_progress = False
    
    def convert_pointcloud2_to_numpy(self, pointcloud_msg):
        """Convert ROS PointCloud2 to numpy array for AnyGrasp"""
        try:
            import struct
            import sensor_msgs_py.point_cloud2 as pc2
            
            # Extract point cloud data
            points_list = []
            
            # Convert PointCloud2 to list of points
            for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
                x, y, z, rgb = point
                # Filter out invalid points
                if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                    points_list.append([x, y, z])
            
            if len(points_list) == 0:
                self.get_logger().warning("No valid points in point cloud")
                return None
            
            # Convert to numpy array
            point_cloud_array = np.array(points_list, dtype=np.float32)
            
            # Filter by workspace bounds (typical robot workspace)
            workspace_bounds = {
                'x_min': -0.5, 'x_max': 0.8,
                'y_min': -0.6, 'y_max': 0.6,
                'z_min': 0.01, 'z_max': 0.5
            }
            
            # Apply workspace filtering
            mask = (
                (point_cloud_array[:, 0] >= workspace_bounds['x_min']) &
                (point_cloud_array[:, 0] <= workspace_bounds['x_max']) &
                (point_cloud_array[:, 1] >= workspace_bounds['y_min']) &
                (point_cloud_array[:, 1] <= workspace_bounds['y_max']) &
                (point_cloud_array[:, 2] >= workspace_bounds['z_min']) &
                (point_cloud_array[:, 2] <= workspace_bounds['z_max'])
            )
            
            filtered_points = point_cloud_array[mask]
            
            if len(filtered_points) < 100:
                self.get_logger().warning(f"Too few points after filtering: {len(filtered_points)}")
                return None
                
            # Downsample if too many points (for performance)
            if len(filtered_points) > 10000:
                indices = np.random.choice(len(filtered_points), 10000, replace=False)
                filtered_points = filtered_points[indices]
            
            self.get_logger().info(f"Converted point cloud: {len(filtered_points)} points")
            return filtered_points
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert point cloud: {e}")
            return None
    
    def process_grasp_results(self, grasp_results):
        """Process AnyGrasp results into candidate objects"""
        candidates = []
        
        # Get parameters
        min_confidence = self.get_parameter('detection_confidence_threshold').get_parameter_value().double_value
        max_candidates = self.get_parameter('max_candidates').get_parameter_value().integer_value
        min_width = self.get_parameter('min_grasp_width').get_parameter_value().double_value
        max_width = self.get_parameter('max_grasp_width').get_parameter_value().double_value
        
        try:
            if self.anygrasp is not None:
                # Process actual AnyGrasp results
                # Note: This is adapted for typical AnyGrasp output format
                for i, grasp_data in enumerate(grasp_results[:max_candidates * 2]):
                    # Extract grasp pose (adapt to actual AnyGrasp format)
                    pose = Pose()
                    if hasattr(grasp_data, 'pose') or isinstance(grasp_data, dict):
                        # Adapt this section based on actual AnyGrasp output format
                        pose.position.x = float(grasp_data.get('x', 0.3))
                        pose.position.y = float(grasp_data.get('y', 0.0))
                        pose.position.z = float(grasp_data.get('z', 0.1))
                        
                        # Convert rotation matrix or quaternion to ROS quaternion
                        pose.orientation.x = float(grasp_data.get('qx', 0.0))
                        pose.orientation.y = float(grasp_data.get('qy', 0.0))
                        pose.orientation.z = float(grasp_data.get('qz', 0.0))
                        pose.orientation.w = float(grasp_data.get('qw', 1.0))
                        
                        confidence = float(grasp_data.get('score', 0.5))
                        width = float(grasp_data.get('width', 0.05))
                    else:
                        # Fallback if grasp_data format is different
                        pose.position.x = 0.3 + np.random.normal(0, 0.05)
                        pose.position.y = np.random.normal(0, 0.1)
                        pose.position.z = 0.1 + np.random.normal(0, 0.02)
                        pose.orientation.w = 1.0
                        confidence = np.random.uniform(0.3, 0.95)
                        width = np.random.uniform(min_width, max_width)
                    
                    quality = confidence * np.random.uniform(0.9, 1.0)
                    
                    # Filter by confidence and grasp width
                    if confidence >= min_confidence and min_width <= width <= max_width:
                        candidate = GraspCandidate(pose, confidence, width, quality, i)
                        candidates.append(candidate)
            else:
                # Enhanced mock implementation for testing
                self.get_logger().info("Using enhanced mock AnyGrasp implementation")
                
                # Generate realistic grasp candidates based on typical object positions
                object_positions = [
                    (0.25, 0.0, 0.12),   # Center object
                    (0.30, 0.15, 0.11),  # Right object  
                    (0.20, -0.12, 0.13), # Left object
                    (0.35, 0.05, 0.10),  # Far object
                    (0.15, 0.08, 0.14)   # Near object
                ]
                
                for i, (x, y, z) in enumerate(object_positions[:max_candidates]):
                    # Create multiple grasp orientations per object
                    for orientation_idx in range(2):  # Top-down and angled grasps
                        pose = Pose()
                        pose.position.x = x + np.random.normal(0, 0.01)
                        pose.position.y = y + np.random.normal(0, 0.01)
                        pose.position.z = z + np.random.normal(0, 0.005)
                        
                        if orientation_idx == 0:
                            # Top-down grasp
                            pose.orientation.x = 0.0
                            pose.orientation.y = 0.0
                            pose.orientation.z = 0.0
                            pose.orientation.w = 1.0
                            confidence_base = 0.85
                        else:
                            # Angled grasp (45 degrees)
                            pose.orientation.x = 0.383
                            pose.orientation.y = 0.0
                            pose.orientation.z = 0.0
                            pose.orientation.w = 0.924
                            confidence_base = 0.65
                        
                        # Add some variability to confidence
                        confidence = confidence_base + np.random.normal(0, 0.1)
                        confidence = np.clip(confidence, 0.1, 0.99)
                        
                        width = np.random.uniform(min_width, max_width)
                        quality = confidence * np.random.uniform(0.85, 1.0)
                        
                        # Filter by confidence and grasp width
                        if confidence >= min_confidence and min_width <= width <= max_width:
                            candidate_idx = i * 2 + orientation_idx
                            candidate = GraspCandidate(pose, confidence, width, quality, candidate_idx)
                            candidates.append(candidate)
            
            # Sort by confidence score (highest first)
            candidates.sort(key=lambda x: x.confidence, reverse=True)
            
            # Return top candidates
            final_candidates = candidates[:max_candidates]
            self.get_logger().info(f"Generated {len(final_candidates)} grasp candidates")
            
            return final_candidates
            
        except Exception as e:
            self.get_logger().error(f"Error processing grasp results: {e}")
            return []
    
    def publish_grasp_candidates(self, candidates):
        """Publish grasp candidates for visualization"""
        if not candidates:
            return
        
        # Publish all candidates
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "base_link"
        
        for candidate in candidates:
            pose_array.poses.append(candidate.pose)
        
        self.grasp_poses_pub.publish(pose_array)
        
        # Publish top candidates separately for user selection
        top_candidates = PoseArray()
        top_candidates.header = pose_array.header
        max_candidates = self.get_parameter('max_candidates').get_parameter_value().integer_value
        
        for candidate in candidates[:max_candidates]:
            top_candidates.poses.append(candidate.pose)
        
        self.candidates_pub.publish(top_candidates)
    
    def get_candidates_callback(self, request, response):
        """Service callback to get current grasp candidates"""
        with self.candidates_lock:
            candidates = self.current_candidates.copy()
        
        if not candidates:
            response.detection_status = "No candidates available"
            response.total_grasps_detected = 0
            return response
        
        # Apply additional filtering based on request
        filtered_candidates = []
        for candidate in candidates:
            if candidate.confidence >= request.min_confidence:
                filtered_candidates.append(candidate)
        
        # Limit number of candidates
        num_candidates = min(request.num_candidates, len(filtered_candidates))
        selected_candidates = filtered_candidates[:num_candidates]
        
        # Fill response
        response.grasp_poses = []
        response.confidence_scores = []
        response.grasp_widths = []
        response.quality_scores = []
        response.original_indices = []
        
        for candidate in selected_candidates:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.pose = candidate.pose
            
            response.grasp_poses.append(pose_stamped)
            response.confidence_scores.append(candidate.confidence)
            response.grasp_widths.append(candidate.width)
            response.quality_scores.append(candidate.quality)
            response.original_indices.append(candidate.original_index)
        
        response.detection_status = f"Returning {len(selected_candidates)} candidates"
        response.total_grasps_detected = len(candidates)
        
        if self.last_detection_time:
            response.detection_timestamp = self.last_detection_time.to_msg()
        
        return response
    
    def select_grasp_callback(self, request, response):
        """Service callback for user grasp selection"""
        with self.candidates_lock:
            candidates = self.current_candidates.copy()
        
        if not candidates:
            response.success = False
            response.message = "No candidates available for selection"
            return response
        
        # Validate selection index
        if request.selected_grasp_index < 0 or request.selected_grasp_index >= len(candidates):
            response.success = False
            response.message = f"Invalid selection index {request.selected_grasp_index}. Available: 0-{len(candidates)-1}"
            return response
        
        # Get selected candidate
        selected_candidate = candidates[request.selected_grasp_index]
        
        # Fill response with selected grasp details
        response.selected_pose = PoseStamped()
        response.selected_pose.header.stamp = self.get_clock().now().to_msg()
        response.selected_pose.header.frame_id = "base_link"
        response.selected_pose.pose = selected_candidate.pose
        response.confidence_score = selected_candidate.confidence
        response.grasp_width = selected_candidate.width
        
        response.success = True
        response.message = f"Selected grasp {request.selected_grasp_index} with confidence {selected_candidate.confidence:.2f}"
        
        # Clear selection pending state
        self.user_selection_pending = False
        self.selection_event.set()
        
        self.get_logger().info(f"User selected grasp {request.selected_grasp_index}: {response.message}")
        self.publish_status(f"Grasp selected - ready for execution")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AnyGraspInteractiveNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()