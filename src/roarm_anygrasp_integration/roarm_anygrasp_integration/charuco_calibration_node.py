#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from std_msgs.msg import String, Header
from roarm_anygrasp_integration.srv import CalibrateCamera
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
from datetime import datetime
import threading
from typing import Dict, List, Optional, Tuple
import tf2_ros
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation

class ChArUcoCalibrationNode(Node):
    def __init__(self):
        super().__init__('charuco_calibration_node')
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # ChArUco board parameters
        self.declare_parameter('board_squares_x', 7)
        self.declare_parameter('board_squares_y', 5) 
        self.declare_parameter('square_length', 0.04)  # 4cm squares
        self.declare_parameter('marker_length', 0.032)  # 3.2cm markers (80% of square)
        self.declare_parameter('dictionary_id', 0)  # DICT_4X4_50
        
        # Calibration parameters
        self.declare_parameter('min_samples', 10)
        self.declare_parameter('max_samples', 50)
        self.declare_parameter('collection_interval', 2.0)  # seconds between captures
        self.declare_parameter('auto_collect', False)
        self.declare_parameter('save_images', True)
        self.declare_parameter('calibration_data_path', '/tmp/charuco_calibration')
        
        # Get parameters
        self.board_squares_x = self.get_parameter('board_squares_x').get_parameter_value().integer_value
        self.board_squares_y = self.get_parameter('board_squares_y').get_parameter_value().integer_value
        self.square_length = self.get_parameter('square_length').get_parameter_value().double_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        self.dictionary_id = self.get_parameter('dictionary_id').get_parameter_value().integer_value
        self.min_samples = self.get_parameter('min_samples').get_parameter_value().integer_value
        self.max_samples = self.get_parameter('max_samples').get_parameter_value().integer_value
        
        # Setup ChArUco board
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.dictionary_id)
        self.board = cv2.aruco.CharucoBoard(
            (self.board_squares_x, self.board_squares_y),
            self.square_length,
            self.marker_length,
            self.dictionary
        )
        self.detector_params = cv2.aruco.DetectorParameters()
        
        # Storage setup
        self.calibration_data_path = self.get_parameter('calibration_data_path').get_parameter_value().string_value
        os.makedirs(self.calibration_data_path, exist_ok=True)
        os.makedirs(os.path.join(self.calibration_data_path, 'images'), exist_ok=True)
        os.makedirs(os.path.join(self.calibration_data_path, 'results'), exist_ok=True)
        
        # Calibration data storage
        self.calibration_lock = threading.Lock()
        self.primary_samples = []
        self.side_samples = []
        self.robot_poses = []  # Robot poses during calibration
        self.calibration_in_progress = False
        
        # Camera data
        self.primary_image = None
        self.side_image = None
        self.primary_camera_info = None
        self.side_camera_info = None
        
        # Subscribers
        self.primary_image_sub = self.create_subscription(
            Image, '/camera_primary/color/image_raw', self.primary_image_callback, 10)
        self.side_image_sub = self.create_subscription(
            Image, '/camera_side/color/image_raw', self.side_image_callback, 10)
        self.primary_info_sub = self.create_subscription(
            CameraInfo, '/camera_primary/color/camera_info', self.primary_info_callback, 10)
        self.side_info_sub = self.create_subscription(
            CameraInfo, '/camera_side/color/camera_info', self.side_info_callback, 10)
        
        # Publishers
        self.calibration_status_pub = self.create_publisher(
            String, '/charuco_calibration/status', 10)
        self.detection_image_pub = self.create_publisher(
            Image, '/charuco_calibration/detection_image', 10)
        self.board_pose_pub = self.create_publisher(
            PoseArray, '/charuco_calibration/board_poses', 10)
        
        # Services
        self.calibrate_service = self.create_service(
            CalibrateCamera, '/charuco_calibration/calibrate_cameras', self.calibrate_service_callback)
        self.collect_sample_service = self.create_service(
            String, '/charuco_calibration/collect_sample', self.collect_sample_callback)
        self.generate_board_service = self.create_service(
            String, '/charuco_calibration/generate_board', self.generate_board_callback)
        
        # Timer for auto collection
        if self.get_parameter('auto_collect').get_parameter_value().bool_value:
            self.collection_timer = self.create_timer(
                self.get_parameter('collection_interval').get_parameter_value().double_value,
                self.auto_collect_callback)
        
        self.get_logger().info('ChArUco Calibration Node initialized')
        self.publish_status("ChArUco calibration ready")
        
        # Generate calibration board on startup
        self.generate_calibration_board()
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f"[CHARUCO_CALIB] {message}"
        self.calibration_status_pub.publish(msg)
        self.get_logger().info(message)
    
    def primary_image_callback(self, msg: Image):
        """Store primary camera image"""
        self.primary_image = msg
    
    def side_image_callback(self, msg: Image):
        """Store side camera image"""
        self.side_image = msg
    
    def primary_info_callback(self, msg: CameraInfo):
        """Store primary camera info"""
        self.primary_camera_info = msg
    
    def side_info_callback(self, msg: CameraInfo):
        """Store side camera info"""
        self.side_camera_info = msg
    
    def generate_calibration_board(self):
        """Generate and save ChArUco calibration board"""
        try:
            # Create board image (A4 size at 300 DPI)
            board_size = (2480, 3508)  # A4 at 300 DPI
            board_image = self.board.generateImage(board_size)
            
            # Save board image
            board_path = os.path.join(self.calibration_data_path, 'charuco_board.png')
            cv2.imwrite(board_path, board_image)
            
            # Save board parameters
            board_info = {
                'squares_x': self.board_squares_x,
                'squares_y': self.board_squares_y,
                'square_length_m': self.square_length,
                'marker_length_m': self.marker_length,
                'dictionary_id': self.dictionary_id,
                'board_size_pixels': board_size,
                'created_timestamp': datetime.now().isoformat()
            }
            
            info_path = os.path.join(self.calibration_data_path, 'board_info.json')
            with open(info_path, 'w') as f:
                json.dump(board_info, f, indent=2)
            
            self.get_logger().info(f"Generated ChArUco board: {board_path}")
            self.publish_status(f"Generated calibration board: {board_path}")
            
        except Exception as e:
            self.get_logger().error(f"Error generating calibration board: {e}")
    
    def detect_charuco_board(self, image: np.ndarray, camera_matrix: np.ndarray, 
                           dist_coeffs: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], np.ndarray]:
        """Detect ChArUco board in image and return corners, ids, and annotated image"""
        try:
            # Detect ArUco markers
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
                image, self.dictionary, parameters=self.detector_params)
            
            # Draw detected markers
            annotated_image = image.copy()
            if marker_ids is not None and len(marker_ids) > 0:
                cv2.aruco.drawDetectedMarkers(annotated_image, marker_corners, marker_ids)
                
                # Interpolate ChArUco corners
                ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(  
                    marker_corners, marker_ids, image, self.board)
                
                if ret > 4:  # Need at least 4 corners for pose estimation
                    # Draw ChArUco corners
                    cv2.aruco.drawDetectedCornersCharuco(annotated_image, charuco_corners, charuco_ids)
                    
                    # Estimate board pose
                    success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        charuco_corners, charuco_ids, self.board, camera_matrix, dist_coeffs, None, None)
                    
                    if success:
                        # Draw coordinate axes
                        cv2.drawFrameAxes(annotated_image, camera_matrix, dist_coeffs, 
                                        rvec, tvec, self.square_length)
                        return charuco_corners, charuco_ids, annotated_image
            
            return None, None, annotated_image
            
        except Exception as e:
            self.get_logger().error(f"Error detecting ChArUco board: {e}")
            return None, None, image
    
    def collect_calibration_sample(self) -> bool:
        """Collect a calibration sample from both cameras"""
        with self.calibration_lock:
            if not self.primary_image or not self.side_image:
                self.get_logger().warning("Cannot collect sample - missing camera data")
                return False
            
            if not self.primary_camera_info or not self.side_camera_info:
                self.get_logger().warning("Cannot collect sample - missing camera info")
                return False
            
            try:
                # Convert images
                primary_cv = self.cv_bridge.imgmsg_to_cv2(self.primary_image, "bgr8")
                side_cv = self.cv_bridge.imgmsg_to_cv2(self.side_image, "bgr8")
                
                # Get camera matrices
                primary_K = np.array(self.primary_camera_info.k).reshape(3, 3)
                primary_D = np.array(self.primary_camera_info.d)
                side_K = np.array(self.side_camera_info.k).reshape(3, 3)
                side_D = np.array(self.side_camera_info.d)
                
                # Detect board in both cameras
                primary_corners, primary_ids, primary_annotated = self.detect_charuco_board(
                    primary_cv, primary_K, primary_D)
                side_corners, side_ids, side_annotated = self.detect_charuco_board(
                    side_cv, side_K, side_D)
                
                # Check if board detected in both cameras
                if primary_corners is not None and side_corners is not None:
                    sample_id = f"sample_{len(self.primary_samples):03d}_{datetime.now().strftime('%H%M%S')}"
                    
                    # Store sample data
                    primary_sample = {
                        'corners': primary_corners,
                        'ids': primary_ids,
                        'image_shape': primary_cv.shape,
                        'camera_matrix': primary_K,
                        'dist_coeffs': primary_D,
                        'timestamp': self.primary_image.header.stamp
                    }
                    
                    side_sample = {
                        'corners': side_corners,
                        'ids': side_ids,
                        'image_shape': side_cv.shape,
                        'camera_matrix': side_K,
                        'dist_coeffs': side_D,
                        'timestamp': self.side_image.header.stamp
                    }
                    
                    # Get robot pose if available
                    robot_pose = self.get_current_robot_pose()
                    
                    self.primary_samples.append(primary_sample)
                    self.side_samples.append(side_sample)
                    self.robot_poses.append(robot_pose)
                    
                    # Save images if enabled
                    if self.get_parameter('save_images').get_parameter_value().bool_value:
                        primary_path = os.path.join(self.calibration_data_path, 'images', 
                                                  f"{sample_id}_primary.jpg")
                        side_path = os.path.join(self.calibration_data_path, 'images',
                                               f"{sample_id}_side.jpg")
                        annotated_primary_path = os.path.join(self.calibration_data_path, 'images',
                                                            f"{sample_id}_primary_annotated.jpg")
                        annotated_side_path = os.path.join(self.calibration_data_path, 'images',
                                                         f"{sample_id}_side_annotated.jpg")
                        
                        cv2.imwrite(primary_path, primary_cv)
                        cv2.imwrite(side_path, side_cv)
                        cv2.imwrite(annotated_primary_path, primary_annotated)
                        cv2.imwrite(annotated_side_path, side_annotated)
                    
                    # Publish detection image
                    detection_msg = self.cv_bridge.cv2_to_imgmsg(primary_annotated, "bgr8")
                    self.detection_image_pub.publish(detection_msg)
                    
                    sample_count = len(self.primary_samples)
                    self.publish_status(f"Collected sample {sample_count}/{self.max_samples}")
                    self.get_logger().info(f"Collected calibration sample {sample_id}")
                    
                    return True
                else:
                    self.get_logger().warning("ChArUco board not detected in both cameras")
                    return False
                    
            except Exception as e:
                self.get_logger().error(f"Error collecting calibration sample: {e}")
                return False
    
    def get_current_robot_pose(self) -> Optional[Dict]:
        """Get current robot end-effector pose"""
        try:
            # Try to get transform from base_link to end_effector
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            
            return {
                'translation': {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z
                },
                'rotation': {
                    'x': transform.transform.rotation.x,
                    'y': transform.transform.rotation.y,
                    'z': transform.transform.rotation.z,
                    'w': transform.transform.rotation.w
                },
                'timestamp': transform.header.stamp
            }
        except Exception as e:
            self.get_logger().debug(f"Could not get robot pose: {e}")
            return None
    
    def perform_camera_calibration(self) -> Dict:
        """Perform camera calibration using collected samples"""
        with self.calibration_lock:
            if len(self.primary_samples) < self.min_samples:
                raise ValueError(f"Not enough samples. Need at least {self.min_samples}, have {len(self.primary_samples)}")
            
            try:
                results = {}
                
                # Calibrate primary camera
                self.get_logger().info("Calibrating primary camera...")
                primary_result = self.calibrate_single_camera(self.primary_samples, "primary")
                results['primary_camera'] = primary_result
                
                # Calibrate side camera
                self.get_logger().info("Calibrating side camera...")
                side_result = self.calibrate_single_camera(self.side_samples, "side")
                results['side_camera'] = side_result
                
                # Perform stereo calibration
                self.get_logger().info("Performing stereo calibration...")
                stereo_result = self.calibrate_stereo_cameras()
                results['stereo_calibration'] = stereo_result
                
                # Save results
                results_path = os.path.join(self.calibration_data_path, 'results', 
                                          f"calibration_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
                
                # Convert numpy arrays to lists for JSON serialization
                json_results = self.numpy_to_json(results)
                with open(results_path, 'w') as f:
                    json.dump(json_results, f, indent=2)
                
                self.get_logger().info(f"Calibration results saved to: {results_path}")
                return results
                
            except Exception as e:
                self.get_logger().error(f"Error performing calibration: {e}")
                raise
    
    def calibrate_single_camera(self, samples: List[Dict], camera_name: str) -> Dict:
        """Calibrate a single camera"""
        # Prepare calibration data
        all_corners = []
        all_ids = []
        image_size = None
        
        for sample in samples:
            all_corners.append(sample['corners'])
            all_ids.append(sample['ids'])
            image_size = (sample['image_shape'][1], sample['image_shape'][0])  # (width, height)
        
        # Calibrate camera
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            all_corners, all_ids, self.board, image_size, None, None)
        
        return {
            'reprojection_error': ret,
            'camera_matrix': camera_matrix,
            'distortion_coefficients': dist_coeffs,
            'rotation_vectors': rvecs,
            'translation_vectors': tvecs,
            'image_size': image_size,
            'num_samples': len(samples)
        }
    
    def calibrate_stereo_cameras(self) -> Dict:
        """Perform stereo calibration between primary and side cameras"""
        # This is a simplified stereo calibration
        # In practice, you'd need corresponding points between cameras
        # For now, we'll return the relative transform
        
        # Get the last successful detection poses from both cameras
        # This would normally require simultaneous detection of the same board
        
        return {
            'relative_rotation': np.eye(3),
            'relative_translation': np.array([0.0, 0.0, 0.0]),
            'essential_matrix': np.eye(3),
            'fundamental_matrix': np.eye(3),
            'note': 'Simplified stereo calibration - requires simultaneous board detection'
        }
    
    def numpy_to_json(self, obj):
        """Convert numpy arrays to lists for JSON serialization"""
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {key: self.numpy_to_json(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            return [self.numpy_to_json(item) for item in obj]
        else:
            return obj
    
    def auto_collect_callback(self):
        """Timer callback for automatic sample collection"""
        if self.calibration_in_progress and len(self.primary_samples) < self.max_samples:
            success = self.collect_calibration_sample()
            if success and len(self.primary_samples) >= self.max_samples:
                self.publish_status(f"Auto-collection complete: {len(self.primary_samples)} samples")
    
    def collect_sample_callback(self, request, response):
        """Service callback for manual sample collection"""
        try:
            success = self.collect_calibration_sample()
            if success:
                sample_count = len(self.primary_samples)
                response.data = f"Sample collected successfully. Total: {sample_count}/{self.max_samples}"
            else:
                response.data = "Failed to collect sample - ChArUco board not detected"
        except Exception as e:
            response.data = f"Error collecting sample: {e}"
            self.get_logger().error(f"Sample collection error: {e}")
        
        return response
    
    def calibrate_service_callback(self, request, response):
        """Service callback for camera calibration"""
        try:
            self.calibration_in_progress = True
            self.publish_status("Starting camera calibration...")
            
            results = self.perform_camera_calibration()
            
            # Prepare response
            response.success = True
            response.message = f"Calibration completed using {len(self.primary_samples)} samples"
            response.primary_reprojection_error = float(results['primary_camera']['reprojection_error'])
            response.side_reprojection_error = float(results['side_camera']['reprojection_error'])
            
            self.publish_status(f"Calibration completed - Primary error: {response.primary_reprojection_error:.3f}, Side error: {response.side_reprojection_error:.3f}")
            
        except Exception as e:
            response.success = False
            response.message = f"Calibration failed: {e}"
            response.primary_reprojection_error = 0.0
            response.side_reprojection_error = 0.0
            self.get_logger().error(f"Calibration error: {e}")
        
        finally:
            self.calibration_in_progress = False
        
        return response
    
    def generate_board_callback(self, request, response):
        """Service callback for generating calibration board"""
        try:
            self.generate_calibration_board()
            board_path = os.path.join(self.calibration_data_path, 'charuco_board.png')
            response.data = f"ChArUco board generated: {board_path}"
        except Exception as e:
            response.data = f"Error generating board: {e}"
            self.get_logger().error(f"Board generation error: {e}")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ChArUcoCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()