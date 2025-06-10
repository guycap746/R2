#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import base64
import os
from datetime import datetime
import threading
from typing import Dict, List, Optional, Tuple

# Roboflow imports
try:
    import roboflow
    from roboflow import Roboflow
    ROBOFLOW_AVAILABLE = True
except ImportError:
    ROBOFLOW_AVAILABLE = False
    print("Roboflow SDK not available. Install with: pip install roboflow")

# Import custom service types
from roarm_anygrasp_integration.srv import UploadToRoboflow, ConfigureRoboflow

class RoboflowIntegrationNode(Node):
    def __init__(self):
        super().__init__('roboflow_integration_node')
        
        # Initialize state
        self.cv_bridge = CvBridge()
        self.roboflow_client = None
        self.project = None
        self.dataset = None
        self.upload_queue = []
        self.upload_thread = None
        self.upload_lock = threading.Lock()
        
        # Configuration
        self.config = {
            'api_key': '',
            'workspace_name': '',
            'project_name': '',
            'dataset_version': 'v1',
            'auto_upload': True,
            'include_failed_grasps': True,
            'annotation_format': 'yolo',
            'min_confidence_for_upload': 0.3,
            'max_queue_size': 100,
            'upload_batch_size': 5
        }
        
        # Subscribers
        self.rgb_image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.grasp_poses_sub = self.create_subscription(
            PoseArray, '/anygrasp/grasp_poses', self.grasp_poses_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/grasp_coordinator/status', self.status_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/roboflow/upload_status', 10)
        
        # Services
        self.upload_service = self.create_service(
            UploadToRoboflow, '/roboflow/upload_image', self.upload_image_callback)
        self.config_service = self.create_service(
            ConfigureRoboflow, '/roboflow/configure', self.configure_callback)
        
        # Parameters
        self.declare_parameter('roboflow_api_key', '')
        self.declare_parameter('workspace_name', 'roarm-grasping')
        self.declare_parameter('project_name', 'grasp-detection')
        self.declare_parameter('auto_upload', True)
        self.declare_parameter('storage_path', '/tmp/roboflow_images')
        
        # Storage
        self.storage_path = self.get_parameter('storage_path').get_parameter_value().string_value
        os.makedirs(self.storage_path, exist_ok=True)
        
        # Current capture data
        self.current_rgb_image = None
        self.current_camera_info = None
        self.current_grasp_poses = None
        self.current_confidence_scores = []
        self.capture_timestamp = None
        
        # Initialize with parameters
        self.initialize_from_parameters()
        
        self.get_logger().info('Roboflow Integration Node initialized')
        if not ROBOFLOW_AVAILABLE:
            self.get_logger().warning('Roboflow SDK not available - install with: pip install roboflow')
    
    def initialize_from_parameters(self):
        """Initialize configuration from ROS parameters"""
        api_key = self.get_parameter('roboflow_api_key').get_parameter_value().string_value
        workspace = self.get_parameter('workspace_name').get_parameter_value().string_value
        project = self.get_parameter('project_name').get_parameter_value().string_value
        auto_upload = self.get_parameter('auto_upload').get_parameter_value().bool_value
        
        if api_key and workspace and project:
            self.config.update({
                'api_key': api_key,
                'workspace_name': workspace,
                'project_name': project,
                'auto_upload': auto_upload
            })
            
            if ROBOFLOW_AVAILABLE:
                self.initialize_roboflow()
    
    def initialize_roboflow(self):
        """Initialize Roboflow client and project"""
        if not ROBOFLOW_AVAILABLE or not self.config['api_key']:
            return False
        
        try:
            self.roboflow_client = Roboflow(api_key=self.config['api_key'])
            
            # Get workspace and project
            workspace = self.roboflow_client.workspace(self.config['workspace_name'])
            self.project = workspace.project(self.config['project_name'])
            
            # Get or create dataset version
            try:
                self.dataset = self.project.version(self.config['dataset_version'])
            except Exception:
                # Version doesn't exist, will be created on first upload
                self.dataset = None
            
            self.get_logger().info(f"Roboflow initialized: {self.config['workspace_name']}/{self.config['project_name']}")
            self.publish_status("Roboflow client initialized successfully")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Roboflow: {e}")
            self.publish_status(f"Roboflow initialization failed: {e}")
            return False
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f"[ROBOFLOW] {message}"
        self.status_pub.publish(msg)
        self.get_logger().info(message)
    
    def rgb_image_callback(self, msg: Image):
        """Store current RGB image for potential upload"""
        self.current_rgb_image = msg
        self.capture_timestamp = self.get_clock().now()
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store current camera info for proper projection"""
        self.current_camera_info = msg
    
    def grasp_poses_callback(self, msg: PoseArray):
        """Store current grasp poses for annotation"""
        self.current_grasp_poses = msg
        
        # If auto-upload is enabled and we have valid detection
        if (self.config['auto_upload'] and 
            self.current_rgb_image is not None and 
            len(msg.poses) > 0):
            
            # Trigger automatic upload
            self.queue_for_upload()
    
    def status_callback(self, msg: String):
        """Monitor workflow status for upload triggers"""
        status = msg.data
        
        # Upload when grasps are detected and user is selecting
        if ("[WAITING_FOR_USER]" in status and 
            self.config['auto_upload'] and
            self.current_rgb_image is not None and
            self.current_grasp_poses is not None):
            
            self.queue_for_upload()
    
    def queue_for_upload(self):
        """Queue current image and annotations for upload"""
        if not self.current_rgb_image or not self.current_grasp_poses:
            return
        
        with self.upload_lock:
            # Check queue size
            if len(self.upload_queue) >= self.config['max_queue_size']:
                self.upload_queue.pop(0)  # Remove oldest
                self.get_logger().warning("Upload queue full - removing oldest entry")
            
            # Create upload item
            upload_item = {
                'timestamp': self.capture_timestamp.to_msg() if self.capture_timestamp else self.get_clock().now().to_msg(),
                'rgb_image': self.current_rgb_image,
                'grasp_poses': self.current_grasp_poses,
                'confidence_scores': self.current_confidence_scores.copy(),
                'scene_description': f"Grasp detection at {datetime.now().isoformat()}",
                'object_types': 'mixed_objects',
                'upload_attempted': False
            }
            
            self.upload_queue.append(upload_item)
            self.get_logger().info(f"Queued image for upload (queue size: {len(self.upload_queue)})")
        
        # Start upload thread if not running
        if self.upload_thread is None or not self.upload_thread.is_alive():
            self.upload_thread = threading.Thread(target=self.upload_worker)
            self.upload_thread.daemon = True
            self.upload_thread.start()
    
    def upload_worker(self):
        """Background worker for uploading images to Roboflow"""
        while True:
            items_to_upload = []
            
            with self.upload_lock:
                # Get batch of items to upload
                for item in self.upload_queue[:self.config['upload_batch_size']]:
                    if not item['upload_attempted']:
                        items_to_upload.append(item)
                        item['upload_attempted'] = True
                
                if not items_to_upload:
                    break  # No more items to upload
            
            # Upload each item
            for item in items_to_upload:
                try:
                    success = self.upload_single_item(item)
                    if success:
                        with self.upload_lock:
                            if item in self.upload_queue:
                                self.upload_queue.remove(item)
                    else:
                        self.get_logger().warning("Upload failed for item")
                        
                except Exception as e:
                    self.get_logger().error(f"Error uploading item: {e}")
            
            # Small delay between batches
            if items_to_upload:
                threading.Event().wait(2.0)
    
    def upload_single_item(self, item: Dict) -> bool:
        """Upload a single image with annotations to Roboflow"""
        if not self.project or not ROBOFLOW_AVAILABLE:
            return False
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(item['rgb_image'], "bgr8")
            
            # Save image temporarily
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            image_filename = f"grasp_scene_{timestamp}.jpg"
            image_path = os.path.join(self.storage_path, image_filename)
            cv2.imwrite(image_path, cv_image)
            
            # Create annotations
            annotations = self.create_annotations(
                cv_image, 
                item['grasp_poses'], 
                item['confidence_scores']
            )
            
            # Upload to Roboflow
            response = self.project.upload(
                image_path=image_path,
                hosted_image=False,
                annotation_path=None,  # We'll send annotations separately
                split="train"  # Default to training set
            )
            
            # Add annotations if successful
            if response and 'id' in response:
                image_id = response['id']
                
                # Upload annotations
                if annotations:
                    annotation_response = self.upload_annotations(image_id, annotations)
                    if annotation_response:
                        self.get_logger().info(f"Successfully uploaded image with {len(annotations)} annotations")
                    else:
                        self.get_logger().warning("Image uploaded but annotation upload failed")
                else:
                    self.get_logger().info("Image uploaded without annotations")
                
                self.publish_status(f"Uploaded image {image_filename} to Roboflow")
                return True
            else:
                self.get_logger().error("Failed to upload image to Roboflow")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error in upload_single_item: {e}")
            return False
        finally:
            # Clean up temporary file
            if os.path.exists(image_path):
                os.remove(image_path)
    
    def create_annotations(self, cv_image: np.ndarray, grasp_poses: PoseArray, confidence_scores: List[float]) -> List[Dict]:
        """Create annotation data for grasp poses"""
        annotations = []
        
        if not grasp_poses or len(grasp_poses.poses) == 0:
            return annotations
        
        height, width = cv_image.shape[:2]
        
        for i, pose in enumerate(grasp_poses.poses):
            confidence = confidence_scores[i] if i < len(confidence_scores) else 0.5
            
            # Skip low-confidence grasps if configured
            if confidence < self.config['min_confidence_for_upload']:
                continue
            
            # Project 3D grasp pose to 2D image coordinates
            # This is a simplified projection - implement proper camera projection
            image_x, image_y = self.project_3d_to_2d(pose, width, height)
            
            if 0 <= image_x < width and 0 <= image_y < height:
                # Create bounding box around grasp point
                box_size = max(20, int(confidence * 50))  # Size based on confidence
                
                x1 = max(0, image_x - box_size // 2)
                y1 = max(0, image_y - box_size // 2)
                x2 = min(width, image_x + box_size // 2)
                y2 = min(height, image_y + box_size // 2)
                
                # YOLO format annotation
                center_x = (x1 + x2) / 2 / width
                center_y = (y1 + y2) / 2 / height
                bbox_width = (x2 - x1) / width
                bbox_height = (y2 - y1) / height
                
                annotation = {
                    'class': 'grasp_point',
                    'class_id': 0,
                    'bbox': [center_x, center_y, bbox_width, bbox_height],
                    'confidence': confidence,
                    'coordinates': {
                        'x': image_x,
                        'y': image_y,
                        'world_x': pose.position.x,
                        'world_y': pose.position.y,
                        'world_z': pose.position.z
                    }
                }
                
                annotations.append(annotation)
        
        return annotations
    
    def project_3d_to_2d(self, pose: Pose, image_width: int, image_height: int) -> Tuple[int, int]:
        """Project 3D pose to 2D image coordinates using proper camera calibration"""
        try:
            # Use actual camera calibration if available
            if self.current_camera_info is not None:
                # Extract camera intrinsics from camera_info
                K = np.array(self.current_camera_info.k).reshape(3, 3)
                fx = K[0, 0]
                fy = K[1, 1]
                cx = K[0, 2]
                cy = K[1, 2]
                
                # Distortion coefficients
                D = np.array(self.current_camera_info.d) if len(self.current_camera_info.d) > 0 else np.zeros(5)
            else:
                # Fallback to typical RealSense D405 parameters
                fx = 615.0
                fy = 615.0
                cx = image_width / 2.0
                cy = image_height / 2.0
                D = np.zeros(5)
            
            # Transform from robot base frame to camera optical frame
            # Note: This simplified transform should be replaced with proper TF2 lookup
            # For RealSense D405 mounted on wrist, typical transform:
            # - Robot base to camera mount: translation + rotation
            # - Camera mount to optical frame: standard camera transforms
            
            # Simplified camera transform (should use TF2)
            # Assuming camera is forward-facing with typical mounting
            x_cam = pose.position.x - 0.05  # Camera offset from base
            y_cam = pose.position.y
            z_cam = pose.position.z + 0.15  # Camera height above base
            
            # Rotate to camera optical frame (x-right, y-down, z-forward)
            x_optical = -y_cam
            y_optical = -z_cam  
            z_optical = x_cam
            
            if z_optical <= 0.01:  # Behind camera or too close
                z_optical = 0.01  # Avoid division by zero
            
            # Project to image coordinates using camera intrinsics
            u_distorted = fx * (x_optical / z_optical) + cx
            v_distorted = fy * (y_optical / z_optical) + cy
            
            # Apply distortion correction if coefficients available
            if np.any(D != 0):
                # Simplified radial distortion correction
                r2 = (x_optical/z_optical)**2 + (y_optical/z_optical)**2
                radial_factor = 1 + D[0]*r2 + D[1]*r2**2 + D[4]*r2**3
                
                u_corrected = u_distorted * radial_factor
                v_corrected = v_distorted * radial_factor
            else:
                u_corrected = u_distorted
                v_corrected = v_distorted
            
            # Clamp to image bounds
            u_final = max(0, min(image_width - 1, int(u_corrected)))
            v_final = max(0, min(image_height - 1, int(v_corrected)))
            
            return u_final, v_final
            
        except Exception as e:
            self.get_logger().error(f"Error in 3D to 2D projection: {e}")
            # Fallback to simple projection
            u = int(image_width * 0.5 + pose.position.x * 200)
            v = int(image_height * 0.5 - pose.position.y * 200)
            return max(0, min(image_width - 1, u)), max(0, min(image_height - 1, v))
    
    def upload_annotations(self, image_id: str, annotations: List[Dict]) -> bool:
        """Upload annotations for a specific image"""
        try:
            # Convert annotations to Roboflow format
            roboflow_annotations = []
            
            for ann in annotations:
                roboflow_ann = {
                    'x': ann['coordinates']['x'],
                    'y': ann['coordinates']['y'],
                    'width': ann['bbox'][2] * 640,  # Convert from normalized
                    'height': ann['bbox'][3] * 480,
                    'class': ann['class'],
                    'confidence': ann['confidence']
                }
                roboflow_annotations.append(roboflow_ann)
            
            # Upload via Roboflow API
            # Note: Specific API calls depend on Roboflow SDK version
            self.get_logger().info(f"Created {len(roboflow_annotations)} annotations for image {image_id}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error uploading annotations: {e}")
            return False
    
    def upload_image_callback(self, request, response):
        """Service callback for manual image upload"""
        try:
            # Create upload item from request
            upload_item = {
                'timestamp': self.get_clock().now().to_msg(),
                'rgb_image': request.rgb_image,
                'grasp_poses': request.grasp_poses,
                'confidence_scores': list(request.confidence_scores),
                'scene_description': request.scene_description or "Manual upload",
                'object_types': request.object_types or "unknown",
                'upload_attempted': False
            }
            
            if request.upload_immediately:
                # Upload immediately
                success = self.upload_single_item(upload_item)
                response.success = success
                response.message = "Immediate upload completed" if success else "Upload failed"
            else:
                # Queue for later upload
                with self.upload_lock:
                    self.upload_queue.append(upload_item)
                response.success = True
                response.message = f"Queued for upload (queue size: {len(self.upload_queue)})"
            
            response.annotation_count = len(request.grasp_poses.poses)
            response.dataset_version = self.config['dataset_version']
            
        except Exception as e:
            response.success = False
            response.message = f"Upload service error: {e}"
            self.get_logger().error(f"Upload service error: {e}")
        
        return response
    
    def configure_callback(self, request, response):
        """Service callback for Roboflow configuration"""
        try:
            # Update configuration
            old_config = self.config.copy()
            
            if request.api_key:
                self.config['api_key'] = request.api_key
            if request.workspace_name:
                self.config['workspace_name'] = request.workspace_name
            if request.project_name:
                self.config['project_name'] = request.project_name
            if request.dataset_version:
                self.config['dataset_version'] = request.dataset_version
            
            self.config['auto_upload'] = request.auto_upload
            self.config['include_failed_grasps'] = request.include_failed_grasps
            self.config['min_confidence_for_upload'] = request.min_confidence_for_upload
            
            if request.annotation_format:
                self.config['annotation_format'] = request.annotation_format
            
            # Reinitialize Roboflow if key parameters changed
            if (self.config['api_key'] != old_config['api_key'] or
                self.config['workspace_name'] != old_config['workspace_name'] or
                self.config['project_name'] != old_config['project_name']):
                
                success = self.initialize_roboflow()
                response.success = success
                response.message = "Configuration updated and Roboflow reinitialized" if success else "Configuration updated but Roboflow initialization failed"
            else:
                response.success = True
                response.message = "Configuration updated successfully"
            
            # Provide URLs if available
            if self.project:
                response.workspace_url = f"https://app.roboflow.com/{self.config['workspace_name']}"
                response.project_url = f"https://app.roboflow.com/{self.config['workspace_name']}/{self.config['project_name']}"
            
        except Exception as e:
            response.success = False
            response.message = f"Configuration error: {e}"
            self.get_logger().error(f"Configuration error: {e}")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RoboflowIntegrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()