#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from datetime import datetime
import json
import threading
from typing import Dict, Optional, Tuple

class DualCameraCaptureNode(Node):
    def __init__(self):
        super().__init__('dual_camera_capture_node')
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Camera data storage
        self.primary_image = None
        self.side_image = None
        self.primary_camera_info = None
        self.side_camera_info = None
        self.last_primary_timestamp = None
        self.last_side_timestamp = None
        
        # Grasp data
        self.current_grasp_poses = None
        self.current_confidence_scores = []
        self.grasp_execution_status = "idle"
        
        # Capture storage
        self.capture_lock = threading.Lock()
        self.captured_data = []
        
        # Parameters
        self.declare_parameter('storage_path', '/tmp/dual_camera_captures')
        self.declare_parameter('save_raw_images', True)
        self.declare_parameter('save_metadata', True)
        self.declare_parameter('auto_capture_on_grasp', True)
        self.declare_parameter('verification_pose_timeout', 10.0)
        
        # Storage setup
        self.storage_path = self.get_parameter('storage_path').get_parameter_value().string_value
        os.makedirs(self.storage_path, exist_ok=True)
        os.makedirs(os.path.join(self.storage_path, 'primary'), exist_ok=True)
        os.makedirs(os.path.join(self.storage_path, 'side'), exist_ok=True)
        os.makedirs(os.path.join(self.storage_path, 'metadata'), exist_ok=True)
        
        # Subscribers for primary camera
        self.primary_image_sub = self.create_subscription(
            Image, '/camera_primary/color/image_raw', self.primary_image_callback, 10)
        self.primary_info_sub = self.create_subscription(
            CameraInfo, '/camera_primary/color/camera_info', self.primary_info_callback, 10)
        
        # Subscribers for side camera
        self.side_image_sub = self.create_subscription(
            Image, '/camera_side/color/image_raw', self.side_image_callback, 10)
        self.side_info_sub = self.create_subscription(
            CameraInfo, '/camera_side/color/camera_info', self.side_info_callback, 10)
        
        # Subscribers for grasp workflow
        self.grasp_poses_sub = self.create_subscription(
            PoseArray, '/anygrasp/grasp_poses', self.grasp_poses_callback, 10)
        self.coordinator_status_sub = self.create_subscription(
            String, '/grasp_coordinator/status', self.coordinator_status_callback, 10)
        
        # Publishers
        self.capture_status_pub = self.create_publisher(
            String, '/dual_camera/capture_status', 10)
        
        # Services
        self.capture_service = self.create_service(
            String, '/dual_camera/capture_now', self.capture_service_callback)
        
        # Timers
        self.status_timer = self.create_timer(5.0, self.status_update)
        
        self.get_logger().info('Dual Camera Capture Node initialized')
        self.publish_status("Dual camera capture ready")
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f"[DUAL_CAMERA] {message}"
        self.capture_status_pub.publish(msg)
        self.get_logger().info(message)
    
    def primary_image_callback(self, msg: Image):
        """Store primary camera image"""
        self.primary_image = msg
        self.last_primary_timestamp = self.get_clock().now()
    
    def side_image_callback(self, msg: Image):
        """Store side camera image"""
        self.side_image = msg
        self.last_side_timestamp = self.get_clock().now()
    
    def primary_info_callback(self, msg: CameraInfo):
        """Store primary camera info"""
        self.primary_camera_info = msg
    
    def side_info_callback(self, msg: CameraInfo):
        """Store side camera info"""
        self.side_camera_info = msg
    
    def grasp_poses_callback(self, msg: PoseArray):
        """Store current grasp poses"""
        self.current_grasp_poses = msg
        
        # Auto-capture on new grasp detection if enabled
        if (self.get_parameter('auto_capture_on_grasp').get_parameter_value().bool_value and
            self.grasp_execution_status == "detecting"):
            self.capture_dual_images("auto_detection")
    
    def coordinator_status_callback(self, msg: String):
        """Monitor grasp coordinator status"""
        status = msg.data
        old_status = self.grasp_execution_status
        
        # Parse status
        if "[DETECTING]" in status:
            self.grasp_execution_status = "detecting"
        elif "[WAITING_FOR_USER]" in status:
            self.grasp_execution_status = "waiting_for_user"
        elif "[EXECUTING]" in status:
            self.grasp_execution_status = "executing"
        elif "[COMPLETED]" in status:
            self.grasp_execution_status = "completed"
            # Capture verification image after completion
            self.capture_dual_images("post_execution")
        elif "[FAILED]" in status:
            self.grasp_execution_status = "failed"
            # Capture failure image for training
            self.capture_dual_images("execution_failed")
        else:
            self.grasp_execution_status = "idle"
        
        # Log status changes
        if old_status != self.grasp_execution_status:
            self.get_logger().info(f"Grasp status changed: {old_status} -> {self.grasp_execution_status}")
    
    def capture_dual_images(self, capture_reason: str) -> Optional[Dict]:
        """Capture synchronized images from both cameras"""
        with self.capture_lock:
            if not self.primary_image or not self.side_image:
                self.get_logger().warning("Cannot capture - missing camera data")
                return None
            
            timestamp = datetime.now()
            capture_id = f"capture_{timestamp.strftime('%Y%m%d_%H%M%S_%f')}"
            
            try:
                # Convert images to OpenCV format
                primary_cv = self.cv_bridge.imgmsg_to_cv2(self.primary_image, "bgr8")
                side_cv = self.cv_bridge.imgmsg_to_cv2(self.side_image, "bgr8")
                
                # Create capture data
                capture_data = {
                    'capture_id': capture_id,
                    'timestamp': timestamp.isoformat(),
                    'capture_reason': capture_reason,
                    'grasp_execution_status': self.grasp_execution_status,
                    'primary_image_shape': primary_cv.shape,
                    'side_image_shape': side_cv.shape,
                    'grasp_data': {
                        'poses': [],
                        'confidence_scores': self.current_confidence_scores.copy()
                    }
                }
                
                # Add grasp pose data if available
                if self.current_grasp_poses:
                    for pose in self.current_grasp_poses.poses:
                        capture_data['grasp_data']['poses'].append({
                            'position': {
                                'x': pose.position.x,
                                'y': pose.position.y,
                                'z': pose.position.z
                            },
                            'orientation': {
                                'x': pose.orientation.x,
                                'y': pose.orientation.y,
                                'z': pose.orientation.z,
                                'w': pose.orientation.w
                            }
                        })
                
                # Add camera info if available
                if self.primary_camera_info:
                    capture_data['primary_camera_info'] = {
                        'width': self.primary_camera_info.width,
                        'height': self.primary_camera_info.height,
                        'frame_id': self.primary_camera_info.header.frame_id,
                        'distortion_model': self.primary_camera_info.distortion_model
                    }
                
                if self.side_camera_info:
                    capture_data['side_camera_info'] = {
                        'width': self.side_camera_info.width,
                        'height': self.side_camera_info.height,
                        'frame_id': self.side_camera_info.header.frame_id,
                        'distortion_model': self.side_camera_info.distortion_model
                    }
                
                # Save images if enabled
                if self.get_parameter('save_raw_images').get_parameter_value().bool_value:
                    primary_path = os.path.join(self.storage_path, 'primary', f"{capture_id}_primary.jpg")
                    side_path = os.path.join(self.storage_path, 'side', f"{capture_id}_side.jpg")
                    
                    cv2.imwrite(primary_path, primary_cv)
                    cv2.imwrite(side_path, side_cv)
                    
                    capture_data['file_paths'] = {
                        'primary_image': primary_path,
                        'side_image': side_path
                    }
                
                # Save metadata if enabled
                if self.get_parameter('save_metadata').get_parameter_value().bool_value:
                    metadata_path = os.path.join(self.storage_path, 'metadata', f"{capture_id}_metadata.json")
                    with open(metadata_path, 'w') as f:
                        json.dump(capture_data, f, indent=2)
                    capture_data['metadata_path'] = metadata_path
                
                # Store in memory
                self.captured_data.append(capture_data)
                
                # Limit memory usage
                if len(self.captured_data) > 100:
                    self.captured_data.pop(0)
                
                self.get_logger().info(f"Captured dual images: {capture_id} (reason: {capture_reason})")
                self.publish_status(f"Captured {capture_id} - {capture_reason}")
                
                return capture_data
                
            except Exception as e:
                self.get_logger().error(f"Error capturing dual images: {e}")
                return None
    
    def capture_service_callback(self, request, response):
        """Service callback for manual capture"""
        try:
            capture_data = self.capture_dual_images("manual_request")
            
            if capture_data:
                response.data = f"Successfully captured {capture_data['capture_id']}"
            else:
                response.data = "Failed to capture images"
                
        except Exception as e:
            response.data = f"Capture service error: {e}"
            self.get_logger().error(f"Capture service error: {e}")
        
        return response
    
    def get_latest_capture(self) -> Optional[Dict]:
        """Get the most recent capture data"""
        with self.capture_lock:
            return self.captured_data[-1] if self.captured_data else None
    
    def get_captures_by_reason(self, reason: str) -> list:
        """Get captures filtered by reason"""
        with self.capture_lock:
            return [capture for capture in self.captured_data if capture['capture_reason'] == reason]
    
    def status_update(self):
        """Periodic status update"""
        primary_age = None
        side_age = None
        
        if self.last_primary_timestamp:
            primary_age = (self.get_clock().now() - self.last_primary_timestamp).nanoseconds / 1e9
        
        if self.last_side_timestamp:
            side_age = (self.get_clock().now() - self.last_side_timestamp).nanoseconds / 1e9
        
        status_parts = []
        
        # Camera status
        if primary_age is not None and primary_age < 2.0:
            status_parts.append("Primary: OK")
        else:
            status_parts.append("Primary: NO DATA")
        
        if side_age is not None and side_age < 2.0:
            status_parts.append("Side: OK")
        else:
            status_parts.append("Side: NO DATA")
        
        # Capture count
        with self.capture_lock:
            status_parts.append(f"Captures: {len(self.captured_data)}")
        
        # Current status
        status_parts.append(f"Status: {self.grasp_execution_status}")
        
        self.publish_status(" | ".join(status_parts))

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraCaptureNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()