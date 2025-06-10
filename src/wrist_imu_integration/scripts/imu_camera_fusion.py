#!/usr/bin/env python3
"""
IMU-Camera Data Fusion Node

Combines BNO055 IMU data with D405 camera for enhanced:
- Camera stabilization
- Motion tracking
- Orientation estimation
- Gravity compensation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from sensor_msgs.msg import Imu, PointCloud2, CameraInfo
from geometry_msgs.msg import Vector3Stamped, PoseStamped, TwistStamped
from std_msgs.msg import Header, Bool
from builtin_interfaces.msg import Time

class IMUCameraFusion(Node):
    def __init__(self):
        super().__init__('imu_camera_fusion')
        
        # Parameters
        self.declare_parameter('camera_frame', 'd405_color_optical_frame')
        self.declare_parameter('imu_frame', 'wrist_imu_link')
        self.declare_parameter('publish_filtered_imu', True)
        self.declare_parameter('publish_camera_stabilization', True)
        self.declare_parameter('fusion_frequency', 50.0)
        self.declare_parameter('stabilization_alpha', 0.8)  # Low-pass filter
        self.declare_parameter('motion_threshold', 0.1)  # rad/s
        
        self.camera_frame = self.get_parameter('camera_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.publish_filtered_imu = self.get_parameter('publish_filtered_imu').value
        self.publish_stabilization = self.get_parameter('publish_camera_stabilization').value
        self.fusion_freq = self.get_parameter('fusion_frequency').value
        self.alpha = self.get_parameter('stabilization_alpha').value
        self.motion_threshold = self.get_parameter('motion_threshold').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/wrist_imu/imu', self.imu_callback, sensor_qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, f'/{self.camera_frame.split("_")[0]}/color/camera_info', 
            self.camera_info_callback, sensor_qos)
        
        # Publishers
        if self.publish_filtered_imu:
            self.filtered_imu_pub = self.create_publisher(
                Imu, '/wrist_imu/imu_filtered', 10)
        
        if self.publish_stabilization:
            self.stabilization_pub = self.create_publisher(
                Vector3Stamped, '/camera/stabilization_offset', 10)
            self.motion_status_pub = self.create_publisher(
                Bool, '/camera/motion_detected', 10)
            self.camera_pose_pub = self.create_publisher(
                PoseStamped, '/camera/stabilized_pose', 10)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # State variables
        self.last_imu_msg = None
        self.last_camera_info = None
        self.filtered_orientation = None
        self.angular_velocity_filtered = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration_filtered = np.array([0.0, 0.0, 0.0])
        
        # Motion detection
        self.motion_detected = False
        self.motion_start_time = None
        
        # Timer for fusion processing
        self.fusion_timer = self.create_timer(
            1.0 / self.fusion_freq, self.fusion_callback)
        
        self.get_logger().info(f'IMU-Camera fusion node started')
        self.get_logger().info(f'Camera frame: {self.camera_frame}')
        self.get_logger().info(f'IMU frame: {self.imu_frame}')
        
    def imu_callback(self, msg):
        """Process IMU data"""
        self.last_imu_msg = msg
        
        # Extract data
        orientation = np.array([
            msg.orientation.x, msg.orientation.y, 
            msg.orientation.z, msg.orientation.w
        ])
        angular_velocity = np.array([
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ])
        linear_acceleration = np.array([
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        ])
        
        # Low-pass filter for stabilization
        if self.filtered_orientation is None:
            self.filtered_orientation = orientation
            self.angular_velocity_filtered = angular_velocity
            self.linear_acceleration_filtered = linear_acceleration
        else:
            # Quaternion SLERP for orientation
            self.filtered_orientation = self.slerp_quaternion(
                self.filtered_orientation, orientation, 1.0 - self.alpha)
            
            # Simple low-pass for velocities and accelerations
            self.angular_velocity_filtered = (
                self.alpha * self.angular_velocity_filtered + 
                (1.0 - self.alpha) * angular_velocity
            )
            self.linear_acceleration_filtered = (
                self.alpha * self.linear_acceleration_filtered + 
                (1.0 - self.alpha) * linear_acceleration
            )
        
        # Motion detection
        angular_magnitude = np.linalg.norm(angular_velocity)
        if angular_magnitude > self.motion_threshold:
            if not self.motion_detected:
                self.motion_detected = True
                self.motion_start_time = self.get_clock().now()
                self.get_logger().debug('Motion detected')
        else:
            if self.motion_detected:
                self.motion_detected = False
                self.get_logger().debug('Motion stopped')
        
        # Publish motion status
        if self.publish_stabilization:
            motion_msg = Bool()
            motion_msg.data = self.motion_detected
            self.motion_status_pub.publish(motion_msg)
    
    def camera_info_callback(self, msg):
        """Process camera info"""
        self.last_camera_info = msg
    
    def fusion_callback(self):
        """Main fusion processing"""
        if self.last_imu_msg is None:
            return
            
        # Publish filtered IMU data
        if self.publish_filtered_imu and self.filtered_orientation is not None:
            self.publish_filtered_imu_data()
        
        # Publish camera stabilization data
        if self.publish_stabilization:
            self.publish_camera_stabilization_data()
    
    def publish_filtered_imu_data(self):
        """Publish filtered IMU data"""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        
        # Filtered orientation
        msg.orientation.x = self.filtered_orientation[0]
        msg.orientation.y = self.filtered_orientation[1]
        msg.orientation.z = self.filtered_orientation[2]
        msg.orientation.w = self.filtered_orientation[3]
        
        # Filtered angular velocity
        msg.angular_velocity.x = self.angular_velocity_filtered[0]
        msg.angular_velocity.y = self.angular_velocity_filtered[1]
        msg.angular_velocity.z = self.angular_velocity_filtered[2]
        
        # Filtered linear acceleration
        msg.linear_acceleration.x = self.linear_acceleration_filtered[0]
        msg.linear_acceleration.y = self.linear_acceleration_filtered[1]
        msg.linear_acceleration.z = self.linear_acceleration_filtered[2]
        
        # Copy covariances from original
        msg.orientation_covariance = self.last_imu_msg.orientation_covariance
        msg.angular_velocity_covariance = self.last_imu_msg.angular_velocity_covariance
        msg.linear_acceleration_covariance = self.last_imu_msg.linear_acceleration_covariance
        
        self.filtered_imu_pub.publish(msg)
    
    def publish_camera_stabilization_data(self):
        """Publish camera stabilization data"""
        try:
            # Get transform from IMU to camera
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, self.imu_frame, 
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            # Calculate stabilization offset
            # This represents the orientation correction needed for the camera
            imu_orientation = self.filtered_orientation
            roll, pitch, yaw = euler_from_quaternion(imu_orientation)
            
            # Create stabilization offset (negative to counteract motion)
            offset_msg = Vector3Stamped()
            offset_msg.header = Header()
            offset_msg.header.stamp = self.get_clock().now().to_msg()
            offset_msg.header.frame_id = self.camera_frame
            offset_msg.vector.x = -roll
            offset_msg.vector.y = -pitch
            offset_msg.vector.z = -yaw
            
            self.stabilization_pub.publish(offset_msg)
            
            # Publish stabilized camera pose
            camera_pose = PoseStamped()
            camera_pose.header = offset_msg.header
            camera_pose.pose.position.x = 0.0
            camera_pose.pose.position.y = 0.0
            camera_pose.pose.position.z = 0.0
            
            # Apply stabilization (inverse of IMU orientation)
            stabilized_q = quaternion_from_euler(-roll, -pitch, -yaw)
            camera_pose.pose.orientation.x = stabilized_q[0]
            camera_pose.pose.orientation.y = stabilized_q[1]
            camera_pose.pose.orientation.z = stabilized_q[2]
            camera_pose.pose.orientation.w = stabilized_q[3]
            
            self.camera_pose_pub.publish(camera_pose)
            
        except Exception as e:
            self.get_logger().debug(f'Transform lookup failed: {e}')
    
    def slerp_quaternion(self, q1, q2, t):
        """Spherical linear interpolation for quaternions"""
        # Normalize quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # Compute dot product
        dot = np.dot(q1, q2)
        
        # Ensure shortest path
        if dot < 0.0:
            q2 = -q2
            dot = -dot
        
        # Linear interpolation for very close quaternions
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return result / np.linalg.norm(result)
        
        # Spherical interpolation
        theta_0 = np.arccos(abs(dot))
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        
        return s0 * q1 + s1 * q2


def main(args=None):
    rclpy.init(args=args)
    
    node = IMUCameraFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()