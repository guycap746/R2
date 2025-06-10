#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from std_msgs.msg import String
from roarm_moveit.srv import MovePointCmd
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
from datetime import datetime
import threading
from typing import Dict, List, Optional, Tuple
import tf2_ros
from scipy.spatial.transform import Rotation
import time

class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__('hand_eye_calibration_node')
        
        # Initialize components
        self.cv_bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # ChArUco board setup (must match calibration node)
        self.declare_parameter('board_squares_x', 7)
        self.declare_parameter('board_squares_y', 5)
        self.declare_parameter('square_length', 0.04)
        self.declare_parameter('marker_length', 0.032)
        self.declare_parameter('dictionary_id', 0)
        
        # Hand-eye calibration parameters
        self.declare_parameter('calibration_poses_count', 10)
        self.declare_parameter('workspace_center_x', 0.25)
        self.declare_parameter('workspace_center_y', 0.0)
        self.declare_parameter('workspace_center_z', 0.15)
        self.declare_parameter('workspace_radius', 0.15)
        self.declare_parameter('min_board_distance', 0.2)
        self.declare_parameter('max_board_distance', 0.4)
        self.declare_parameter('data_path', '/tmp/hand_eye_calibration')
        
        # Setup ChArUco detector
        self.board_squares_x = self.get_parameter('board_squares_x').get_parameter_value().integer_value
        self.board_squares_y = self.get_parameter('board_squares_y').get_parameter_value().integer_value
        self.square_length = self.get_parameter('square_length').get_parameter_value().double_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        self.dictionary_id = self.get_parameter('dictionary_id').get_parameter_value().integer_value
        
        self.dictionary = cv2.aruco.getPredefinedDictionary(self.dictionary_id)
        self.board = cv2.aruco.CharucoBoard(
            (self.board_squares_x, self.board_squares_y),
            self.square_length,
            self.marker_length,
            self.dictionary
        )
        self.detector_params = cv2.aruco.DetectorParameters()
        
        # Storage setup
        self.data_path = self.get_parameter('data_path').get_parameter_value().string_value
        os.makedirs(self.data_path, exist_ok=True)
        os.makedirs(os.path.join(self.data_path, 'poses'), exist_ok=True)
        os.makedirs(os.path.join(self.data_path, 'images'), exist_ok=True)
        
        # Calibration data
        self.calibration_lock = threading.Lock()
        self.robot_poses = []  # Robot end-effector poses
        self.board_poses_camera = []  # Board poses in camera frame
        self.calibration_in_progress = False
        self.current_pose_index = 0
        
        # Camera data
        self.camera_image = None
        self.camera_info = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera_primary/color/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera_primary/color/camera_info', self.camera_info_callback, 10)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/hand_eye_calibration/status', 10)
        self.detection_pub = self.create_publisher(Image, '/hand_eye_calibration/detection_image', 10)
        self.target_poses_pub = self.create_publisher(PoseArray, '/hand_eye_calibration/target_poses', 10)
        
        # Service clients
        self.move_point_client = self.create_client(MovePointCmd, '/move_point_cmd')
        
        # Services
        self.start_calibration_service = self.create_service(
            String, '/hand_eye_calibration/start_calibration', self.start_calibration_callback)
        self.collect_pose_service = self.create_service(
            String, '/hand_eye_calibration/collect_pose', self.collect_pose_callback)
        self.compute_calibration_service = self.create_service(
            String, '/hand_eye_calibration/compute_calibration', self.compute_calibration_callback)
        
        # Wait for services
        self.get_logger().info("Waiting for move_point service...")
        self.move_point_client.wait_for_service(timeout_sec=10.0)
        
        self.get_logger().info('Hand-Eye Calibration Node initialized')
        self.publish_status("Hand-eye calibration ready")
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f"[HAND_EYE] {message}"
        self.status_pub.publish(msg)
        self.get_logger().info(message)
    
    def image_callback(self, msg: Image):
        """Store camera image"""
        self.camera_image = msg
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera info"""
        self.camera_info = msg
    
    def generate_calibration_poses(self) -> List[Dict]:
        """Generate a set of robot poses for hand-eye calibration"""
        poses = []
        pose_count = self.get_parameter('calibration_poses_count').get_parameter_value().integer_value
        
        # Workspace parameters
        center_x = self.get_parameter('workspace_center_x').get_parameter_value().double_value
        center_y = self.get_parameter('workspace_center_y').get_parameter_value().double_value
        center_z = self.get_parameter('workspace_center_z').get_parameter_value().double_value
        radius = self.get_parameter('workspace_radius').get_parameter_value().double_value
        min_dist = self.get_parameter('min_board_distance').get_parameter_value().double_value
        max_dist = self.get_parameter('max_board_distance').get_parameter_value().double_value
        
        for i in range(pose_count):
            # Generate pose around workspace center
            angle = 2 * np.pi * i / pose_count
            
            # Position in circle around center
            x = center_x + radius * np.cos(angle) * 0.5
            y = center_y + radius * np.sin(angle) * 0.5
            z = center_z + (max_dist - min_dist) * (i % 3) / 2  # Vary height
            
            # Orientation looking at board (assumed to be at workspace center)
            # Calculate direction vector from camera to board center
            dx = center_x - x
            dy = center_y - y
            dz = center_z - z
            
            # Convert to rotation matrix (simplified)
            # This assumes the board is horizontal at the workspace center
            # In practice, you'd calculate proper orientation
            roll = 0.0
            pitch = np.arctan2(dz, np.sqrt(dx*dx + dy*dy))
            yaw = np.arctan2(dy, dx)
            
            # Convert to quaternion
            r = Rotation.from_euler('xyz', [roll, pitch, yaw])
            quat = r.as_quat()  # [x, y, z, w]
            
            pose = {
                'position': {'x': x, 'y': y, 'z': z},
                'orientation': {'x': quat[0], 'y': quat[1], 'z': quat[2], 'w': quat[3]},
                'pose_id': i
            }
            poses.append(pose)
        
        return poses
    
    def move_to_pose(self, pose: Dict) -> bool:
        """Move robot to specified pose"""
        try:
            request = MovePointCmd.Request()
            request.x = pose['position']['x']
            request.y = pose['position']['y']
            request.z = pose['position']['z']
            
            self.get_logger().info(f"Moving to pose {pose['pose_id']}: ({request.x:.3f}, {request.y:.3f}, {request.z:.3f})")
            
            future = self.move_point_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            
            if future.result() is not None:
                self.get_logger().info(f"Successfully moved to pose {pose['pose_id']}")
                return True
            else:
                self.get_logger().error(f"Failed to move to pose {pose['pose_id']}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error moving to pose: {e}")
            return False
    
    def detect_board_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Detect ChArUco board and return its pose in camera frame"""
        if not self.camera_image or not self.camera_info:
            return None
        
        try:
            # Convert image
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            
            # Get camera parameters
            camera_matrix = np.array(self.camera_info.k).reshape(3, 3)
            dist_coeffs = np.array(self.camera_info.d)
            
            # Detect markers
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
                cv_image, self.dictionary, parameters=self.detector_params)
            
            if marker_ids is not None and len(marker_ids) > 0:
                # Interpolate ChArUco corners
                ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, cv_image, self.board)
                
                if ret > 4:  # Need at least 4 corners
                    # Estimate board pose
                    success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        charuco_corners, charuco_ids, self.board, camera_matrix, dist_coeffs, None, None)
                    
                    if success:
                        # Draw detection on image
                        annotated_image = cv_image.copy()
                        cv2.aruco.drawDetectedMarkers(annotated_image, marker_corners, marker_ids)
                        cv2.aruco.drawDetectedCornersCharuco(annotated_image, charuco_corners, charuco_ids)
                        cv2.drawFrameAxes(annotated_image, camera_matrix, dist_coeffs, 
                                        rvec, tvec, self.square_length)
                        
                        # Publish detection image
                        detection_msg = self.cv_bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                        self.detection_pub.publish(detection_msg)
                        
                        return rvec, tvec
            
            return None
            
        except Exception as e:
            self.get_logger().error(f"Error detecting board pose: {e}")
            return None
    
    def get_robot_pose(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Get current robot end-effector pose relative to base"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'tool0', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            
            # Extract translation
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Extract rotation
            quat = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
            rotation = Rotation.from_quat(quat)
            rvec = rotation.as_rotvec()
            
            return rvec, translation
            
        except Exception as e:
            self.get_logger().debug(f"Could not get robot pose: {e}")
            return None
    
    def collect_pose_data(self) -> bool:
        """Collect robot pose and board detection for current position"""
        with self.calibration_lock:
            # Wait for stable position
            time.sleep(2.0)
            
            # Get robot pose
            robot_pose = self.get_robot_pose()
            if robot_pose is None:
                self.get_logger().error("Could not get robot pose")
                return False
            
            # Detect board
            board_pose = self.detect_board_pose()
            if board_pose is None:
                self.get_logger().error("Could not detect ChArUco board")
                return False
            
            # Store data
            robot_rvec, robot_tvec = robot_pose
            board_rvec, board_tvec = board_pose
            
            self.robot_poses.append({
                'rvec': robot_rvec,
                'tvec': robot_tvec,
                'pose_index': self.current_pose_index
            })
            
            self.board_poses_camera.append({
                'rvec': board_rvec,
                'tvec': board_tvec,
                'pose_index': self.current_pose_index
            })
            
            # Save data
            pose_data = {
                'robot_pose': {
                    'rvec': robot_rvec.tolist(),
                    'tvec': robot_tvec.tolist()
                },
                'board_pose_camera': {
                    'rvec': board_rvec.tolist(),
                    'tvec': board_tvec.tolist()
                },
                'pose_index': self.current_pose_index,
                'timestamp': datetime.now().isoformat()
            }
            
            pose_file = os.path.join(self.data_path, 'poses', f'pose_{self.current_pose_index:03d}.json')
            with open(pose_file, 'w') as f:
                json.dump(pose_data, f, indent=2)
            
            # Save image
            if self.camera_image:
                cv_image = self.cv_bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                image_file = os.path.join(self.data_path, 'images', f'pose_{self.current_pose_index:03d}.jpg')
                cv2.imwrite(image_file, cv_image)
            
            self.current_pose_index += 1
            count = len(self.robot_poses)
            total = self.get_parameter('calibration_poses_count').get_parameter_value().integer_value
            self.publish_status(f"Collected pose data {count}/{total}")
            
            return True
    
    def compute_hand_eye_calibration(self) -> Dict:
        """Compute hand-eye calibration using collected data"""
        if len(self.robot_poses) < 3:
            raise ValueError(f"Need at least 3 poses for calibration, have {len(self.robot_poses)}")
        
        try:
            # Prepare data for OpenCV hand-eye calibration
            R_gripper2base = []  # Robot poses
            t_gripper2base = []
            R_target2cam = []    # Board poses in camera
            t_target2cam = []
            
            for robot_pose, board_pose in zip(self.robot_poses, self.board_poses_camera):
                # Robot pose (gripper to base)
                R_robot = Rotation.from_rotvec(robot_pose['rvec']).as_matrix()
                t_robot = robot_pose['tvec']
                
                R_gripper2base.append(R_robot)
                t_gripper2base.append(t_robot.reshape(3, 1))
                
                # Board pose in camera (target to camera)
                R_board = Rotation.from_rotvec(board_pose['rvec']).as_matrix()
                t_board = board_pose['tvec']
                
                R_target2cam.append(R_board)
                t_target2cam.append(t_board.reshape(3, 1))
            
            # Perform hand-eye calibration
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=cv2.CALIB_HAND_EYE_TSAI
            )
            
            # Calculate reprojection error
            total_error = 0.0
            for i, (robot_pose, board_pose) in enumerate(zip(self.robot_poses, self.board_poses_camera)):
                # Transform board pose to robot base frame
                R_robot = Rotation.from_rotvec(robot_pose['rvec']).as_matrix()
                t_robot = robot_pose['tvec'].reshape(3, 1)
                
                R_board_cam = Rotation.from_rotvec(board_pose['rvec']).as_matrix()
                t_board_cam = board_pose['tvec'].reshape(3, 1)
                
                # Expected board pose in base frame
                R_board_base_expected = R_robot @ R_cam2gripper @ R_board_cam
                t_board_base_expected = R_robot @ (R_cam2gripper @ t_board_cam + t_cam2gripper) + t_robot
                
                # For error calculation, we'd need ground truth board pose in base frame
                # For now, calculate consistency error between poses
                if i > 0:
                    prev_R = Rotation.from_rotvec(self.robot_poses[i-1]['rvec']).as_matrix()
                    prev_t = self.robot_poses[i-1]['tvec'].reshape(3, 1)
                    prev_R_board = Rotation.from_rotvec(self.board_poses_camera[i-1]['rvec']).as_matrix()
                    prev_t_board = self.board_poses_camera[i-1]['tvec'].reshape(3, 1)
                    
                    prev_board_base = prev_R @ (R_cam2gripper @ prev_t_board + t_cam2gripper) + prev_t
                    error = np.linalg.norm(t_board_base_expected - prev_board_base)
                    total_error += error
            
            avg_error = total_error / max(1, len(self.robot_poses) - 1)
            
            # Convert to quaternion for easier handling
            rotation_quat = Rotation.from_matrix(R_cam2gripper).as_quat()
            
            # Save calibration result
            result = {
                'rotation_matrix': R_cam2gripper.tolist(),
                'translation_vector': t_cam2gripper.flatten().tolist(),
                'rotation_quaternion': rotation_quat.tolist(),  # [x, y, z, w]
                'reprojection_error': float(avg_error),
                'num_poses': len(self.robot_poses),
                'calibration_method': 'CALIB_HAND_EYE_TSAI',
                'timestamp': datetime.now().isoformat()
            }
            
            # Save to file
            result_file = os.path.join(self.data_path, f'hand_eye_calibration_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json')
            with open(result_file, 'w') as f:
                json.dump(result, f, indent=2)
            
            # Publish TF transform
            self.publish_camera_transform(R_cam2gripper, t_cam2gripper.flatten())
            
            self.get_logger().info(f"Hand-eye calibration completed. Result saved to: {result_file}")
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error computing hand-eye calibration: {e}")
            raise
    
    def publish_camera_transform(self, rotation_matrix: np.ndarray, translation: np.ndarray):
        """Publish camera transform as TF"""
        try:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'tool0'  # End-effector frame
            transform.child_frame_id = 'camera_hand_eye_calibrated'
            
            # Set translation
            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])
            
            # Set rotation
            quat = Rotation.from_matrix(rotation_matrix).as_quat()
            transform.transform.rotation.x = float(quat[0])
            transform.transform.rotation.y = float(quat[1])
            transform.transform.rotation.z = float(quat[2])
            transform.transform.rotation.w = float(quat[3])
            
            self.tf_broadcaster.sendTransform(transform)
            self.get_logger().info("Published hand-eye calibration transform")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing transform: {e}")
    
    def start_calibration_callback(self, request, response):
        """Service callback to start automatic calibration sequence"""
        try:
            if self.calibration_in_progress:
                response.data = "Calibration already in progress"
                return response
            
            self.calibration_in_progress = True
            self.current_pose_index = 0
            self.robot_poses.clear()
            self.board_poses_camera.clear()
            
            self.publish_status("Starting automatic hand-eye calibration...")
            
            # Generate calibration poses
            target_poses = self.generate_calibration_poses()
            
            # Publish target poses for visualization
            pose_array = PoseArray()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = 'base_link'
            
            for pose_dict in target_poses:
                pose = Pose()
                pose.position.x = pose_dict['position']['x']
                pose.position.y = pose_dict['position']['y']
                pose.position.z = pose_dict['position']['z']
                pose.orientation.x = pose_dict['orientation']['x']
                pose.orientation.y = pose_dict['orientation']['y']
                pose.orientation.z = pose_dict['orientation']['z']
                pose.orientation.w = pose_dict['orientation']['w']
                pose_array.poses.append(pose)
            
            self.target_poses_pub.publish(pose_array)
            
            # Execute calibration sequence
            success_count = 0
            for i, pose in enumerate(target_poses):
                self.publish_status(f"Moving to calibration pose {i+1}/{len(target_poses)}")
                
                # Move to pose
                if self.move_to_pose(pose):
                    # Collect data at this pose
                    if self.collect_pose_data():
                        success_count += 1
                    else:
                        self.get_logger().warning(f"Failed to collect data at pose {i+1}")
                else:
                    self.get_logger().warning(f"Failed to move to pose {i+1}")
            
            if success_count >= 3:
                # Compute calibration
                self.publish_status("Computing hand-eye calibration...")
                result = self.compute_hand_eye_calibration()
                
                response.data = f"Calibration completed successfully using {success_count} poses. Error: {result['reprojection_error']:.4f}"
                self.publish_status(f"Calibration completed - Error: {result['reprojection_error']:.4f}")
            else:
                response.data = f"Calibration failed - only collected {success_count} valid poses (need at least 3)"
                self.publish_status("Calibration failed - insufficient valid poses")
            
        except Exception as e:
            response.data = f"Calibration error: {e}"
            self.get_logger().error(f"Calibration error: {e}")
        
        finally:
            self.calibration_in_progress = False
        
        return response
    
    def collect_pose_callback(self, request, response):
        """Service callback for manual pose collection"""
        try:
            if self.collect_pose_data():
                count = len(self.robot_poses)
                response.data = f"Pose data collected successfully. Total poses: {count}"
            else:
                response.data = "Failed to collect pose data"
        except Exception as e:
            response.data = f"Error collecting pose: {e}"
            self.get_logger().error(f"Pose collection error: {e}")
        
        return response
    
    def compute_calibration_callback(self, request, response):
        """Service callback for manual calibration computation"""
        try:
            result = self.compute_hand_eye_calibration()
            response.data = f"Hand-eye calibration computed. Error: {result['reprojection_error']:.4f}, Poses: {result['num_poses']}"
        except Exception as e:
            response.data = f"Calibration computation error: {e}"
            self.get_logger().error(f"Calibration computation error: {e}")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()