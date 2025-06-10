#!/usr/bin/env python3
"""
Dummy Hardware Simulation Nodes for RoArm M3 System Testing

This script provides simulation nodes for:
- RoArm M3 robot arm
- Intel RealSense D405 cameras (dual setup)
- AnyGrasp detection simulation
- MoveIt planning scene objects

Used for testing system functionality without physical hardware.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2, CameraInfo, JointState, Image, Imu
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Quaternion, Vector3
from std_msgs.msg import Header, String, UInt8
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
import numpy as np
import threading
import time
import random
import math

class DummyRoArmDriver(Node):
    """Simulates RoArm M3 robot driver"""
    
    def __init__(self):
        super().__init__('dummy_roarm_driver')
        
        # Joint state publisher
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.status_pub = self.create_publisher(String, '/roarm/status', 10)
        
        # Joint names for RoArm M3
        self.joint_names = [
            'base_to_link1',
            'link1_to_link2', 
            'link2_to_link3',
            'link3_to_gripper_base',
            'gripper_left_joint',
            'gripper_right_joint'
        ]
        
        # Current joint positions (start at home)
        self.joint_positions = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        self.joint_velocities = [0.0] * 6
        self.joint_efforts = [0.0] * 6
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_joint_state)
        
        # Movement simulation
        self.target_positions = self.joint_positions.copy()
        self.movement_timer = self.create_timer(0.05, self.simulate_movement)
        
        self.get_logger().info('Dummy RoArm driver started')
        
    def publish_joint_state(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.joint_positions
        msg.velocity = self.joint_velocities
        msg.effort = self.joint_efforts
        
        self.joint_pub.publish(msg)
        
        # Publish status
        status = String()
        status.data = "READY"
        self.status_pub.publish(status)
        
    def simulate_movement(self):
        """Simulate smooth joint movement"""
        for i in range(len(self.joint_positions)):
            diff = self.target_positions[i] - self.joint_positions[i]
            if abs(diff) > 0.01:
                self.joint_positions[i] += diff * 0.1  # Smooth interpolation
                self.joint_velocities[i] = diff * 0.1 / 0.05  # Approximate velocity
            else:
                self.joint_velocities[i] = 0.0
        
        # Occasionally move to new random position
        if random.random() < 0.001:  # 0.1% chance per call
            self.move_to_random_position()
    
    def move_to_random_position(self):
        """Move to a random valid position"""
        self.target_positions = [
            random.uniform(-3.14, 3.14),     # base rotation
            random.uniform(-2.0, 0.5),      # shoulder 
            random.uniform(0.5, 2.5),       # elbow
            random.uniform(-1.57, 1.57),    # wrist
            random.uniform(0.0, 0.8),       # gripper left
            random.uniform(0.0, 0.8)        # gripper right
        ]

class DummyCameraNode(Node):
    """Simulates Intel RealSense D405 camera"""
    
    def __init__(self, camera_name='d405', camera_namespace='d405'):
        super().__init__(f'dummy_{camera_name}_node')
        
        self.camera_name = camera_name
        self.camera_namespace = camera_namespace
        
        # Publishers
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, f'/{camera_namespace}/depth/color/points', 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, f'/{camera_namespace}/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, f'/{camera_namespace}/depth/camera_info', 10)
        self.rgb_image_pub = self.create_publisher(
            Image, f'/{camera_namespace}/color/image_raw', 10)
        self.depth_image_pub = self.create_publisher(
            Image, f'/{camera_namespace}/depth/image_rect_raw', 10)
            
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_camera_data)  # 10 Hz
        
        # Camera parameters (D405 specs)
        self.width = 640
        self.height = 480
        self.fx = 500.0  # Focal length X
        self.fy = 500.0  # Focal length Y
        self.cx = 320.0  # Principal point X
        self.cy = 240.0  # Principal point Y
        
        self.get_logger().info(f'Dummy {camera_name} camera started')
        
    def create_camera_info(self):
        """Create camera info message"""
        cam_info = CameraInfo()
        cam_info.header = Header()
        cam_info.header.stamp = self.get_clock().now().to_msg()
        cam_info.header.frame_id = f'{self.camera_namespace}_color_optical_frame'
        
        cam_info.height = self.height
        cam_info.width = self.width
        
        # Camera matrix [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        cam_info.k = [self.fx, 0.0, self.cx,
                      0.0, self.fy, self.cy,
                      0.0, 0.0, 1.0]
        
        # Distortion coefficients (assume no distortion for dummy)
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        cam_info.distortion_model = "plumb_bob"
        
        # Rectification and projection matrices
        cam_info.r = [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
        
        cam_info.p = [self.fx, 0.0, self.cx, 0.0,
                      0.0, self.fy, self.cy, 0.0,
                      0.0, 0.0, 1.0, 0.0]
        
        return cam_info
        
    def create_dummy_pointcloud(self):
        """Create dummy point cloud with some objects"""
        pc_msg = PointCloud2()
        pc_msg.header = Header()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = f'{self.camera_namespace}_color_optical_frame'
        
        pc_msg.height = self.height
        pc_msg.width = self.width
        pc_msg.is_bigendian = False
        pc_msg.point_step = 16  # 4 fields * 4 bytes each
        pc_msg.row_step = pc_msg.point_step * self.width
        pc_msg.is_dense = False
        
        # Simple field description for XYZRGB
        from sensor_msgs_py import point_cloud2
        pc_msg.fields = point_cloud2.create_cloud_xyz32(
            Header(), [[0, 0, 0]]).fields
        
        # Create minimal point cloud data (empty for dummy)
        pc_msg.data = bytearray(pc_msg.row_step * pc_msg.height)
        
        return pc_msg
        
    def create_dummy_image(self, encoding='rgb8'):
        """Create dummy image"""
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = f'{self.camera_namespace}_color_optical_frame'
        
        img_msg.height = self.height
        img_msg.width = self.width
        img_msg.encoding = encoding
        img_msg.is_bigendian = False
        
        if encoding == 'rgb8':
            img_msg.step = self.width * 3
            # Create simple gradient pattern
            data = []
            for y in range(self.height):
                for x in range(self.width):
                    r = int((x / self.width) * 255)
                    g = int((y / self.height) * 255)
                    b = 128
                    data.extend([r, g, b])
            img_msg.data = bytes(data)
        else:  # depth image
            img_msg.step = self.width * 2
            # Create simple depth pattern
            data = []
            for y in range(self.height):
                for x in range(self.width):
                    # Simple depth gradient
                    depth = int(1000 + (x + y) * 2)  # 1-2 meters
                    data.extend([depth & 0xFF, (depth >> 8) & 0xFF])
            img_msg.data = bytes(data)
            
        return img_msg
        
    def publish_camera_data(self):
        """Publish all camera data"""
        # Camera info
        cam_info = self.create_camera_info()
        self.camera_info_pub.publish(cam_info)
        self.depth_info_pub.publish(cam_info)
        
        # Point cloud
        pc_msg = self.create_dummy_pointcloud()
        self.pointcloud_pub.publish(pc_msg)
        
        # RGB image
        rgb_img = self.create_dummy_image('rgb8')
        self.rgb_image_pub.publish(rgb_img)
        
        # Depth image
        depth_img = self.create_dummy_image('16UC1')
        self.depth_image_pub.publish(depth_img)

class DummyAnyGraspNode(Node):
    """Simulates AnyGrasp detection results"""
    
    def __init__(self):
        super().__init__('dummy_anygrasp_node')
        
        # Publishers
        self.grasp_poses_pub = self.create_publisher(
            PoseArray, '/anygrasp/grasp_poses', 10)
        self.top_candidates_pub = self.create_publisher(
            PoseArray, '/anygrasp/top_candidates', 10)
        self.status_pub = self.create_publisher(
            String, '/anygrasp/status', 10)
            
        # Timer for publishing
        self.timer = self.create_timer(2.0, self.publish_grasp_data)  # 0.5 Hz
        
        self.get_logger().info('Dummy AnyGrasp node started')
        
    def create_dummy_grasps(self, num_grasps=10):
        """Create dummy grasp poses"""
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'd405_color_optical_frame'
        
        for i in range(num_grasps):
            pose = Pose()
            
            # Random position in front of camera
            pose.position.x = random.uniform(0.1, 0.5)
            pose.position.y = random.uniform(-0.2, 0.2)
            pose.position.z = random.uniform(0.3, 0.8)
            
            # Random orientation
            roll = random.uniform(-0.5, 0.5)
            pitch = random.uniform(-0.5, 0.5)
            yaw = random.uniform(-3.14, 3.14)
            
            # Convert to quaternion
            qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
            qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
            qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
            qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
            
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            
            pose_array.poses.append(pose)
            
        return pose_array
        
    def publish_grasp_data(self):
        """Publish dummy grasp detection results"""
        # All grasp candidates
        all_grasps = self.create_dummy_grasps(20)
        self.grasp_poses_pub.publish(all_grasps)
        
        # Top candidates (first 5)
        top_grasps = PoseArray()
        top_grasps.header = all_grasps.header
        top_grasps.poses = all_grasps.poses[:5]
        self.top_candidates_pub.publish(top_grasps)
        
        # Status
        status = String()
        status.data = "DETECTED"
        self.status_pub.publish(status)

class DummyPlanningSceneNode(Node):
    """Publishes dummy planning scene objects"""
    
    def __init__(self):
        super().__init__('dummy_planning_scene_node')
        
        # Publisher
        self.scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)
            
        # Timer
        self.timer = self.create_timer(5.0, self.publish_planning_scene)
        
        self.get_logger().info('Dummy planning scene node started')
        
    def create_table_object(self):
        """Create a table collision object"""
        collision_object = CollisionObject()
        collision_object.header = Header()
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.header.frame_id = 'base_link'
        collision_object.id = 'table'
        
        # Table as a box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.8, 0.6, 0.05]  # 80cm x 60cm x 5cm
        
        collision_object.primitives.append(box)
        
        # Table position
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = -0.025  # Half height below base
        pose.orientation.w = 1.0
        
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        
        return collision_object
        
    def publish_planning_scene(self):
        """Publish planning scene with table"""
        scene = PlanningScene()
        scene.name = "dummy_scene"
        scene.is_diff = True
        
        # Add table
        table = self.create_table_object()
        scene.world.collision_objects.append(table)
        
        self.scene_pub.publish(scene)

class DummyWristIMU(Node):
    """Simulates BNO055 wrist IMU"""
    
    def __init__(self):
        super().__init__('dummy_wrist_imu')
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/wrist_imu/imu', 10)
        self.calib_status_pub = self.create_publisher(UInt8, '/wrist_imu/calib_status', 10)
        self.status_pub = self.create_publisher(String, '/wrist_imu/calibration_guidance', 10)
        
        # Timer for publishing
        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50 Hz
        self.calib_timer = self.create_timer(1.0, self.publish_calibration_status)
        
        # IMU state
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.linear_acceleration = np.array([0.0, 0.0, -9.81])  # gravity
        
        # Simulation of hand motion
        self.time_offset = random.uniform(0, 2*np.pi)
        
        # Calibration simulation
        self.calibration_level = 0
        self.calibration_timer_count = 0
        
        self.get_logger().info('Dummy wrist IMU started')
        
    def publish_imu_data(self):
        """Publish simulated IMU data"""
        current_time = time.time() + self.time_offset
        
        # Simulate gentle hand/wrist motion
        roll = 0.1 * np.sin(current_time * 0.5)  # Slow roll motion
        pitch = 0.05 * np.cos(current_time * 0.3)  # Slight pitch variation
        yaw = 0.02 * np.sin(current_time * 0.2)   # Minor yaw drift
        
        # Convert to quaternion
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        
        self.orientation = np.array([
            sr * cp * cy - cr * sp * sy,  # x
            cr * sp * cy + sr * cp * sy,  # y
            cr * cp * sy - sr * sp * cy,  # z
            cr * cp * cy + sr * sp * sy   # w
        ])
        
        # Simulate angular velocity (derivatives of orientation)
        self.angular_velocity = np.array([
            0.05 * np.cos(current_time * 0.5),
            0.015 * np.sin(current_time * 0.3),
            0.004 * np.cos(current_time * 0.2)
        ])
        
        # Simulate linear acceleration (gravity + small movements)
        self.linear_acceleration = np.array([
            0.1 * np.sin(current_time * 0.4),    # Small X movement
            0.05 * np.cos(current_time * 0.6),   # Small Y movement
            -9.81 + 0.02 * np.sin(current_time)  # Gravity + small Z variation
        ])
        
        # Create IMU message
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'wrist_imu_link'
        
        # Orientation
        msg.orientation.x = self.orientation[0]
        msg.orientation.y = self.orientation[1]
        msg.orientation.z = self.orientation[2]
        msg.orientation.w = self.orientation[3]
        
        # Angular velocity
        msg.angular_velocity.x = self.angular_velocity[0]
        msg.angular_velocity.y = self.angular_velocity[1]
        msg.angular_velocity.z = self.angular_velocity[2]
        
        # Linear acceleration
        msg.linear_acceleration.x = self.linear_acceleration[0]
        msg.linear_acceleration.y = self.linear_acceleration[1]
        msg.linear_acceleration.z = self.linear_acceleration[2]
        
        # Covariances (typical values for BNO055)
        orientation_cov = [0.0159] * 9  # ~7 degrees standard deviation
        angular_vel_cov = [0.04] * 9    # ~11 degrees/sec standard deviation
        linear_acc_cov = [0.017] * 9    # ~0.13 m/s² standard deviation
        
        msg.orientation_covariance = orientation_cov
        msg.angular_velocity_covariance = angular_vel_cov
        msg.linear_acceleration_covariance = linear_acc_cov
        
        self.imu_pub.publish(msg)
        
    def publish_calibration_status(self):
        """Publish simulated calibration status"""
        self.calibration_timer_count += 1
        
        # Simulate gradual calibration improvement
        if self.calibration_timer_count < 5:
            # System, Gyro, Accel, Mag (0-3 each)
            calibration_values = [0, 1, 1, 0]
        elif self.calibration_timer_count < 10:
            calibration_values = [1, 2, 2, 1]
        elif self.calibration_timer_count < 15:
            calibration_values = [2, 3, 3, 2]
        else:
            calibration_values = [3, 3, 3, 3]  # Fully calibrated
        
        # Pack into single byte (4 x 2-bit values)
        status_byte = (
            (calibration_values[0] << 6) |  # System
            (calibration_values[1] << 4) |  # Gyro
            (calibration_values[2] << 2) |  # Accel
            calibration_values[3]           # Mag
        )
        
        msg = UInt8()
        msg.data = status_byte
        self.calib_status_pub.publish(msg)
        
        # Publish guidance message
        if self.calibration_timer_count < 15:
            guidance_msg = String()
            guidance_msg.data = f'Simulated calibration in progress: {calibration_values}'
            self.status_pub.publish(guidance_msg)

class DummyOAKDCamera(Node):
    """Simulates OAK-D stereo camera"""
    
    def __init__(self):
        super().__init__('dummy_oak_d_camera')
        
        # Publishers
        self.color_pub = self.create_publisher(Image, '/oak_d/oak_d/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/oak_d/oak_d/depth/image_raw', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/oak_d/oak_d/depth/color/points', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/oak_d/oak_d/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/oak_d/oak_d/depth/camera_info', 10)
        self.left_pub = self.create_publisher(Image, '/oak_d/oak_d/infra1/image_rect_raw', 10)
        self.right_pub = self.create_publisher(Image, '/oak_d/oak_d/infra2/image_rect_raw', 10)
        
        # Timer
        self.timer = self.create_timer(0.033, self.publish_data)  # 30 FPS
        
        # Camera parameters (OAK-D specs)
        self.width = 1920
        self.height = 1080
        self.depth_width = 1280
        self.depth_height = 720
        
        self.get_logger().info('Dummy OAK-D camera started')
        
    def create_camera_info(self, width, height):
        """Create camera info for OAK-D"""
        cam_info = CameraInfo()
        cam_info.header = Header()
        cam_info.header.stamp = self.get_clock().now().to_msg()
        cam_info.header.frame_id = 'oak_d_color_optical_frame'
        
        cam_info.height = height
        cam_info.width = width
        
        # OAK-D approximate intrinsics
        fx = 840.0 if width > 1280 else 640.0
        fy = fx
        cx = width / 2.0
        cy = height / 2.0
        
        cam_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        cam_info.distortion_model = "plumb_bob"
        cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        cam_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return cam_info
        
    def create_dummy_image(self, width, height, encoding='rgb8'):
        """Create dummy image for OAK-D"""
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'oak_d_color_optical_frame'
        
        img_msg.height = height
        img_msg.width = width
        img_msg.encoding = encoding
        img_msg.is_bigendian = False
        
        if encoding == 'rgb8':
            img_msg.step = width * 3
            # OAK-D style gradient with higher quality
            data = []
            for y in range(height):
                for x in range(width):
                    r = int((x / width) * 255)
                    g = int((y / height) * 255)
                    b = int(((x + y) / (width + height)) * 255)
                    data.extend([r, g, b])
            img_msg.data = bytes(data)
        else:  # depth
            img_msg.step = width * 2
            data = []
            for y in range(height):
                for x in range(width):
                    # Better depth simulation
                    depth = int(800 + (x + y) * 1.5)  # 0.8-2.5 meters
                    data.extend([depth & 0xFF, (depth >> 8) & 0xFF])
            img_msg.data = bytes(data)
            
        return img_msg
    
    def publish_data(self):
        """Publish OAK-D camera data"""
        # Color image and info
        color_img = self.create_dummy_image(self.width, self.height, 'rgb8')
        self.color_pub.publish(color_img)
        
        color_info = self.create_camera_info(self.width, self.height)
        self.color_info_pub.publish(color_info)
        
        # Depth image and info
        depth_img = self.create_dummy_image(self.depth_width, self.depth_height, '16UC1')
        depth_img.header.frame_id = 'oak_d_depth_optical_frame'
        self.depth_pub.publish(depth_img)
        
        depth_info = self.create_camera_info(self.depth_width, self.depth_height)
        depth_info.header.frame_id = 'oak_d_depth_optical_frame'
        self.depth_info_pub.publish(depth_info)
        
        # Stereo images
        left_img = self.create_dummy_image(self.depth_width, self.depth_height, 'mono8')
        left_img.header.frame_id = 'oak_d_left_camera_optical_frame'
        self.left_pub.publish(left_img)
        
        right_img = self.create_dummy_image(self.depth_width, self.depth_height, 'mono8')
        right_img.header.frame_id = 'oak_d_right_camera_optical_frame'
        self.right_pub.publish(right_img)
        
        # Point cloud (minimal)
        pc_msg = PointCloud2()
        pc_msg.header = Header()
        pc_msg.header.stamp = self.get_clock().now().to_msg()
        pc_msg.header.frame_id = 'oak_d_color_optical_frame'
        pc_msg.height = self.depth_height
        pc_msg.width = self.depth_width
        pc_msg.is_bigendian = False
        pc_msg.point_step = 16
        pc_msg.row_step = pc_msg.point_step * self.depth_width
        pc_msg.is_dense = False
        pc_msg.data = bytearray(pc_msg.row_step * pc_msg.height)
        self.pointcloud_pub.publish(pc_msg)

class DummyOAK1Camera(Node):
    """Simulates OAK-1 mono camera"""
    
    def __init__(self):
        super().__init__('dummy_oak_1_camera')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/oak_1/oak_1/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/oak_1/oak_1/camera_info', 10)
        
        # Timer - higher frequency for tracking
        self.timer = self.create_timer(0.016, self.publish_data)  # 60 FPS
        
        # Camera parameters
        self.width = 1920
        self.height = 1080
        
        self.get_logger().info('Dummy OAK-1 camera started')
        
    def create_camera_info(self):
        """Create camera info for OAK-1"""
        cam_info = CameraInfo()
        cam_info.header = Header()
        cam_info.header.stamp = self.get_clock().now().to_msg()
        cam_info.header.frame_id = 'oak_1_optical_frame'
        
        cam_info.height = self.height
        cam_info.width = self.width
        
        # OAK-1 approximate intrinsics
        fx = 900.0
        fy = fx
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        cam_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        cam_info.distortion_model = "plumb_bob"
        cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        cam_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        return cam_info
        
    def create_dummy_image(self):
        """Create high-quality dummy image for OAK-1"""
        img_msg = Image()
        img_msg.header = Header()
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'oak_1_optical_frame'
        
        img_msg.height = self.height
        img_msg.width = self.width
        img_msg.encoding = 'mono8'
        img_msg.is_bigendian = False
        img_msg.step = self.width
        
        # High contrast pattern for tracking simulation
        data = []
        current_time = time.time()
        for y in range(self.height):
            for x in range(self.width):
                # Moving pattern for motion simulation
                pattern = (x + y + int(current_time * 10)) % 256
                data.append(pattern)
        
        img_msg.data = bytes(data)
        return img_msg
    
    def publish_data(self):
        """Publish OAK-1 camera data"""
        # Mono image
        img = self.create_dummy_image()
        self.image_pub.publish(img)
        
        # Camera info
        cam_info = self.create_camera_info()
        self.camera_info_pub.publish(cam_info)

def main():
    """Main function to run all dummy nodes"""
    rclpy.init()
    
    # Create all dummy nodes
    nodes = [
        DummyRoArmDriver(),
        DummyCameraNode('d405', 'd405'),  # Primary camera
        DummyCameraNode('d405_side', 'd405_side'),  # Side camera
        DummyAnyGraspNode(),
        DummyPlanningSceneNode(),
        DummyWristIMU(),  # Wrist-mounted IMU
        DummyOAKDCamera(),  # OAK-D stereo camera
        DummyOAK1Camera()   # OAK-1 mono camera
    ]
    
    # Create threads for each node
    threads = []
    for node in nodes:
        def run_node(n):
            try:
                rclpy.spin(n)
            except KeyboardInterrupt:
                pass
            finally:
                n.destroy_node()
        
        thread = threading.Thread(target=run_node, args=(node,))
        thread.daemon = True
        thread.start()
        threads.append(thread)
    
    print("All dummy hardware nodes started. Press Ctrl+C to stop.")
    
    try:
        # Keep main thread alive
        for thread in threads:
            thread.join()
    except KeyboardInterrupt:
        print("Shutting down dummy hardware nodes...")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()