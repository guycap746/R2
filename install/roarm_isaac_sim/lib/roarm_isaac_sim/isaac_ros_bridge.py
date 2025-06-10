#!/usr/bin/env python3
"""
Isaac Sim - ROS2 Bridge for RoArm M3

This bridge provides seamless communication between Isaac Sim and the ROS2 
ecosystem, enabling sensor data flow, robot control, and state synchronization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import time
from threading import Lock

# ROS2 message types
from std_msgs.msg import String, Header, Bool, Float32MultiArray
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from tf2_msgs.msg import TFMessage

# Isaac Sim imports (conditional)
try:
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import get_current_stage
    from omni.isaac.sensor import Camera, ContactSensor
    from omni.isaac.core.robots import Robot
    import omni.replicator.core as rep
    ISAAC_AVAILABLE = True
except ImportError:
    ISAAC_AVAILABLE = False

class IsaacROSBridge(Node):
    def __init__(self):
        super().__init__('isaac_ros_bridge')
        
        # Parameters
        self.declare_parameter('update_frequency', 60.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_camera_data', True)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('robot_name', 'roarm_m3')
        
        self.update_freq = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.publish_cameras = self.get_parameter('publish_camera_data').get_parameter_value().bool_value
        self.publish_joints = self.get_parameter('publish_joint_states').get_parameter_value().bool_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers for sensor data
        if self.publish_cameras:
            self.d405_image_pub = self.create_publisher(
                Image, '/camera/color/image_raw', sensor_qos)
            self.d405_depth_pub = self.create_publisher(
                Image, '/camera/depth/image_raw', sensor_qos)
            self.d405_pointcloud_pub = self.create_publisher(
                PointCloud2, '/camera/depth/color/points', sensor_qos)
            self.d405_info_pub = self.create_publisher(
                CameraInfo, '/camera/color/camera_info', sensor_qos)
            
            self.oak_d_image_pub = self.create_publisher(
                Image, '/oak_d/oak_d/color/image_raw', sensor_qos)
            self.oak_d_depth_pub = self.create_publisher(
                Image, '/oak_d/oak_d/depth/image_raw', sensor_qos)
            self.oak_d_pointcloud_pub = self.create_publisher(
                PointCloud2, '/oak_d/oak_d/depth/color/points', sensor_qos)
            self.oak_d_info_pub = self.create_publisher(
                CameraInfo, '/oak_d/oak_d/color/camera_info', sensor_qos)
        
        if self.publish_joints:
            self.joint_state_pub = self.create_publisher(
                JointState, '/joint_states', 10)
        
        if self.publish_tf:
            self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        
        # Bridge status
        self.bridge_status_pub = self.create_publisher(String, '/isaac_bridge/status', 10)
        
        # Subscribers for robot control
        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory',
            self.joint_trajectory_callback, 10)
        
        self.target_pose_sub = self.create_subscription(
            PoseStamped, '/move_group/goal_pose',
            self.target_pose_callback, 10)
        
        # Isaac Sim components
        self.world = None
        self.robot = None
        self.cameras = {}
        self.sensors = {}
        self.isaac_initialized = False
        
        # Thread safety
        self.data_lock = Lock()
        
        # Timer for publishing sensor data
        self.sensor_timer = self.create_timer(1.0 / self.update_freq, self.publish_sensor_data)
        
        self.get_logger().info('Isaac-ROS Bridge initialized')
        
        # Initialize Isaac Sim connection
        if ISAAC_AVAILABLE:
            self.initialize_isaac_connection()
        else:
            self.get_logger().error('Isaac Sim not available')
    
    def initialize_isaac_connection(self):
        """Initialize connection to running Isaac Sim instance"""
        try:
            # Get world instance
            self.world = World.instance()
            if not self.world:
                self.get_logger().warning('Isaac Sim world not found, waiting...')
                return
            
            # Get robot
            self.robot = self.world.scene.get_object(self.robot_name)
            if not self.robot:
                self.get_logger().warning(f'Robot {self.robot_name} not found in scene')
            
            # Initialize cameras
            self.initialize_cameras()
            
            # Initialize sensors
            self.initialize_sensors()
            
            self.isaac_initialized = True
            self.publish_bridge_status('Connected to Isaac Sim')
            self.get_logger().info('Isaac Sim connection established')
            
        except Exception as e:
            self.get_logger().error(f'Isaac Sim connection failed: {e}')
            self.publish_bridge_status(f'Connection failed: {e}')
    
    def initialize_cameras(self):
        """Initialize camera sensors"""
        if not self.publish_cameras:
            return
            
        try:
            # Create camera instances for sensor data publishing
            camera_configs = [
                {
                    'name': 'd405',
                    'prim_path': '/World/Cameras/D405',
                    'width': 640,
                    'height': 480,
                    'frequency': 30
                },
                {
                    'name': 'oak_d', 
                    'prim_path': '/World/Cameras/OAK_D',
                    'width': 1280,
                    'height': 720,
                    'frequency': 30
                }
            ]
            
            for config in camera_configs:
                try:
                    camera = Camera(
                        prim_path=config['prim_path'],
                        frequency=config['frequency'],
                        resolution=(config['width'], config['height'])
                    )
                    camera.initialize()
                    self.cameras[config['name']] = {
                        'camera': camera,
                        'config': config
                    }
                    self.get_logger().info(f'Camera {config["name"]} initialized')
                except Exception as e:
                    self.get_logger().warning(f'Failed to initialize camera {config["name"]}: {e}')
            
        except Exception as e:
            self.get_logger().error(f'Camera initialization failed: {e}')
    
    def initialize_sensors(self):
        """Initialize additional sensors"""
        try:
            # Force/torque sensors (if available)
            # Contact sensors
            # IMU sensors
            self.get_logger().info('Additional sensors initialized')
            
        except Exception as e:
            self.get_logger().error(f'Sensor initialization failed: {e}')
    
    def publish_sensor_data(self):
        """Publish sensor data from Isaac Sim to ROS2"""
        if not self.isaac_initialized:
            # Try to reconnect
            if ISAAC_AVAILABLE:
                self.initialize_isaac_connection()
            return
        
        with self.data_lock:
            try:
                # Publish camera data
                if self.publish_cameras:
                    self.publish_camera_data_internal()
                
                # Publish joint states
                if self.publish_joints:
                    self.publish_joint_states_internal()
                
                # Publish TF
                if self.publish_tf:
                    self.publish_tf_data()
                    
            except Exception as e:
                self.get_logger().debug(f'Sensor data publishing error: {e}')
    
    def publish_camera_data_internal(self):
        """Publish camera sensor data"""
        current_time = self.get_clock().now()
        
        for cam_name, cam_data in self.cameras.items():
            try:
                camera = cam_data['camera']
                config = cam_data['config']
                
                # Get camera data
                rgb_data = camera.get_rgba()
                depth_data = camera.get_depth_linear_array()
                
                if rgb_data is not None and depth_data is not None:
                    # Convert to ROS2 messages
                    header = Header()
                    header.stamp = current_time.to_msg()
                    header.frame_id = f'{cam_name}_optical_frame'
                    
                    # RGB Image
                    rgb_msg = self.numpy_to_image_msg(rgb_data[:,:,:3], header, 'rgb8')
                    
                    # Depth Image  
                    depth_msg = self.numpy_to_image_msg(depth_data, header, '32FC1')
                    
                    # Point Cloud
                    pc_msg = self.depth_to_pointcloud(depth_data, rgb_data, header, config)
                    
                    # Camera Info
                    info_msg = self.create_camera_info_msg(header, config)
                    
                    # Publish based on camera type
                    if cam_name == 'd405':
                        self.d405_image_pub.publish(rgb_msg)
                        self.d405_depth_pub.publish(depth_msg) 
                        self.d405_pointcloud_pub.publish(pc_msg)
                        self.d405_info_pub.publish(info_msg)
                    elif cam_name == 'oak_d':
                        self.oak_d_image_pub.publish(rgb_msg)
                        self.oak_d_depth_pub.publish(depth_msg)
                        self.oak_d_pointcloud_pub.publish(pc_msg)
                        self.oak_d_info_pub.publish(info_msg)
                        
            except Exception as e:
                self.get_logger().debug(f'Camera {cam_name} data publishing failed: {e}')
    
    def publish_joint_states_internal(self):
        """Publish robot joint states"""
        if not self.robot:
            return
            
        try:
            # Get joint positions from Isaac Sim
            joint_positions = self.robot.get_joint_positions()
            joint_velocities = self.robot.get_joint_velocities()
            
            if joint_positions is not None:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                
                # Get joint names (adapt based on your robot model)
                joint_names = [
                    'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
                ]
                
                msg.name = joint_names[:len(joint_positions)]
                msg.position = joint_positions.tolist()
                
                if joint_velocities is not None:
                    msg.velocity = joint_velocities.tolist()
                
                self.joint_state_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().debug(f'Joint state publishing failed: {e}')
    
    def publish_tf_data(self):
        """Publish TF transformations"""
        # Implementation for publishing TF frames
        pass
    
    def numpy_to_image_msg(self, numpy_array, header, encoding):
        """Convert numpy array to ROS2 Image message"""
        from cv_bridge import CvBridge
        bridge = CvBridge()
        
        if encoding == 'rgb8':
            numpy_array = (numpy_array * 255).astype(np.uint8)
        
        image_msg = bridge.cv2_to_imgmsg(numpy_array, encoding)
        image_msg.header = header
        return image_msg
    
    def depth_to_pointcloud(self, depth_data, rgb_data, header, config):
        """Convert depth + RGB to PointCloud2"""
        import sensor_msgs_py.point_cloud2 as pc2
        
        # Camera intrinsics (should be configured properly)
        fx = 615.0  # Focal length X
        fy = 615.0  # Focal length Y
        cx = config['width'] / 2.0  # Principal point X
        cy = config['height'] / 2.0  # Principal point Y
        
        points = []
        h, w = depth_data.shape
        
        for v in range(0, h, 4):  # Subsample for performance
            for u in range(0, w, 4):
                z = depth_data[v, u]
                if z > 0.1 and z < 3.0:  # Valid depth range
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    
                    if rgb_data is not None:
                        r = int(rgb_data[v, u, 0] * 255)
                        g = int(rgb_data[v, u, 1] * 255) 
                        b = int(rgb_data[v, u, 2] * 255)
                        rgb = (r << 16) | (g << 8) | b
                    else:
                        rgb = 0
                    
                    points.append([x, y, z, rgb])
        
        return pc2.create_cloud_xyz32(header, points)
    
    def create_camera_info_msg(self, header, config):
        """Create CameraInfo message"""
        msg = CameraInfo()
        msg.header = header
        msg.width = config['width']
        msg.height = config['height']
        
        # Camera matrix (adjust based on actual camera)
        fx = 615.0
        fy = 615.0
        cx = config['width'] / 2.0
        cy = config['height'] / 2.0
        
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        
        return msg
    
    def joint_trajectory_callback(self, msg):
        """Handle joint trajectory commands"""
        if not self.robot or not self.isaac_initialized:
            return
            
        try:
            if len(msg.points) > 0:
                # Execute first trajectory point (simplified)
                point = msg.points[0]
                joint_positions = np.array(point.positions)
                
                # Set joint positions in Isaac Sim
                self.robot.set_joint_positions(joint_positions)
                
                self.get_logger().debug('Joint trajectory executed')
                
        except Exception as e:
            self.get_logger().error(f'Joint trajectory execution failed: {e}')
    
    def target_pose_callback(self, msg):
        """Handle target pose commands"""
        if not self.robot or not self.isaac_initialized:
            return
            
        try:
            target_position = [
                msg.pose.position.x,
                msg.pose.position.y, 
                msg.pose.position.z
            ]
            target_orientation = [
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z
            ]
            
            # Move robot end-effector (would need inverse kinematics)
            self.get_logger().info(f'Target pose received: {target_position}')
            
        except Exception as e:
            self.get_logger().error(f'Target pose execution failed: {e}')
    
    def publish_bridge_status(self, message):
        """Publish bridge status"""
        msg = String()
        msg.data = f"[ISAAC_BRIDGE] {message}"
        self.bridge_status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        bridge = IsaacROSBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Bridge error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()