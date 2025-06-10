#!/usr/bin/env python3
"""
Multi-Camera Coordinator Node

Coordinates multiple camera systems for optimal robotic perception:
- Synchronizes data streams from different cameras
- Combines point clouds from multiple depth cameras
- Manages camera switching based on task requirements
- Provides unified camera interface for grasping pipeline
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf_transformations import quaternion_matrix

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header, String, Bool
import sensor_msgs_py.point_cloud2 as pc2

class MultiCameraCoordinator(Node):
    def __init__(self):
        super().__init__('multi_camera_coordinator')
        
        # Parameters
        self.declare_parameter('realsense_enabled', True)
        self.declare_parameter('oak_d_enabled', True)
        self.declare_parameter('oak_1_enabled', False)
        self.declare_parameter('synchronize_streams', True)
        self.declare_parameter('max_sync_delay', 0.1)
        self.declare_parameter('publish_combined_pointcloud', True)
        self.declare_parameter('combined_pointcloud_topic', '/cameras/combined_pointcloud')
        self.declare_parameter('base_frame', 'base_link')
        
        # Get parameters
        self.realsense_enabled = self.get_parameter('realsense_enabled').value
        self.oak_d_enabled = self.get_parameter('oak_d_enabled').value
        self.oak_1_enabled = self.get_parameter('oak_1_enabled').value
        self.synchronize_streams = self.get_parameter('synchronize_streams').value
        self.max_sync_delay = self.get_parameter('max_sync_delay').value
        self.publish_combined_pc = self.get_parameter('publish_combined_pointcloud').value
        self.combined_pc_topic = self.get_parameter('combined_pointcloud_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Data storage for synchronization
        self.latest_data = {
            'realsense': {'pointcloud': None, 'timestamp': None},
            'oak_d': {'pointcloud': None, 'timestamp': None},
            'oak_1': {'image': None, 'timestamp': None}
        }
        
        # Subscribers for each camera system
        if self.realsense_enabled:
            self.realsense_pc_sub = self.create_subscription(
                PointCloud2, '/realsense/d405/depth/color/points',
                self.realsense_pointcloud_callback, sensor_qos)
                
        if self.oak_d_enabled:
            self.oak_d_pc_sub = self.create_subscription(
                PointCloud2, '/oak_d/oak_d/depth/color/points',
                self.oak_d_pointcloud_callback, sensor_qos)
                
        if self.oak_1_enabled:
            self.oak_1_img_sub = self.create_subscription(
                Image, '/oak_1/oak_1/image_raw',
                self.oak_1_image_callback, sensor_qos)
        
        # Publishers
        if self.publish_combined_pc:
            self.combined_pc_pub = self.create_publisher(
                PointCloud2, self.combined_pc_topic, 10)
        
        self.camera_status_pub = self.create_publisher(
            String, '/cameras/status', 10)
        self.active_camera_pub = self.create_publisher(
            String, '/cameras/active_camera', 10)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Timer for coordination tasks
        self.coordination_timer = self.create_timer(0.1, self.coordination_callback)
        
        # State variables
        self.active_camera = 'realsense'  # Default active camera
        self.last_combined_pc_time = self.get_clock().now()
        
        self.get_logger().info('Multi-camera coordinator started')
        self.get_logger().info(f'Cameras enabled: RealSense={self.realsense_enabled}, '
                             f'OAK-D={self.oak_d_enabled}, OAK-1={self.oak_1_enabled}')
    
    def realsense_pointcloud_callback(self, msg):
        """Handle RealSense point cloud data"""
        self.latest_data['realsense']['pointcloud'] = msg
        self.latest_data['realsense']['timestamp'] = self.get_clock().now()
        
        if self.publish_combined_pc and self.active_camera == 'realsense':
            self.update_combined_pointcloud()
    
    def oak_d_pointcloud_callback(self, msg):
        """Handle OAK-D point cloud data"""
        self.latest_data['oak_d']['pointcloud'] = msg
        self.latest_data['oak_d']['timestamp'] = self.get_clock().now()
        
        if self.publish_combined_pc:
            self.update_combined_pointcloud()
    
    def oak_1_image_callback(self, msg):
        """Handle OAK-1 image data"""
        self.latest_data['oak_1']['image'] = msg
        self.latest_data['oak_1']['timestamp'] = self.get_clock().now()
    
    def coordination_callback(self):
        """Main coordination logic"""
        # Publish camera status
        self.publish_camera_status()
        
        # Manage active camera selection
        self.update_active_camera()
        
        # Check data freshness
        self.check_data_freshness()
    
    def publish_camera_status(self):
        """Publish current camera system status"""
        current_time = self.get_clock().now()
        status_parts = []
        
        for camera, data in self.latest_data.items():
            if data['timestamp'] is not None:
                age = (current_time - data['timestamp']).nanoseconds / 1e9
                status = f"{camera}:{'OK' if age < 1.0 else 'STALE'}"
                status_parts.append(status)
            else:
                status_parts.append(f"{camera}:NO_DATA")
        
        status_msg = String()
        status_msg.data = ' | '.join(status_parts)
        self.camera_status_pub.publish(status_msg)
        
        # Publish active camera
        active_msg = String()
        active_msg.data = self.active_camera
        self.active_camera_pub.publish(active_msg)
    
    def update_active_camera(self):
        """Update which camera is considered primary based on data quality"""
        current_time = self.get_clock().now()
        
        # Check data freshness for each camera
        camera_scores = {}
        
        for camera, data in self.latest_data.items():
            if data['timestamp'] is not None:
                age = (current_time - data['timestamp']).nanoseconds / 1e9
                if age < 0.5:  # Fresh data (< 500ms)
                    camera_scores[camera] = 1.0
                elif age < 1.0:  # Acceptable data (< 1s)
                    camera_scores[camera] = 0.5
                else:  # Stale data
                    camera_scores[camera] = 0.0
            else:
                camera_scores[camera] = 0.0
        
        # Priority: RealSense > OAK-D > OAK-1 (if all have fresh data)
        if camera_scores.get('realsense', 0) > 0.5 and self.realsense_enabled:
            self.active_camera = 'realsense'
        elif camera_scores.get('oak_d', 0) > 0.5 and self.oak_d_enabled:
            self.active_camera = 'oak_d'
        elif camera_scores.get('oak_1', 0) > 0.5 and self.oak_1_enabled:
            self.active_camera = 'oak_1'
        else:
            # Keep current active camera if no good options
            pass
    
    def update_combined_pointcloud(self):
        """Combine point clouds from multiple depth cameras"""
        if not self.publish_combined_pc:
            return
            
        current_time = self.get_clock().now()
        
        # Limit update frequency to reduce computation
        if (current_time - self.last_combined_pc_time).nanoseconds / 1e9 < 0.1:
            return
        
        self.last_combined_pc_time = current_time
        
        # Collect valid point clouds
        point_clouds = []
        
        # RealSense point cloud
        if (self.realsense_enabled and 
            self.latest_data['realsense']['pointcloud'] is not None):
            age = (current_time - self.latest_data['realsense']['timestamp']).nanoseconds / 1e9
            if age < self.max_sync_delay:
                point_clouds.append(('realsense', self.latest_data['realsense']['pointcloud']))
        
        # OAK-D point cloud  
        if (self.oak_d_enabled and 
            self.latest_data['oak_d']['pointcloud'] is not None):
            age = (current_time - self.latest_data['oak_d']['timestamp']).nanoseconds / 1e9
            if age < self.max_sync_delay:
                point_clouds.append(('oak_d', self.latest_data['oak_d']['pointcloud']))
        
        if len(point_clouds) == 0:
            return
        
        # Combine point clouds
        try:
            combined_pc = self.merge_pointclouds(point_clouds)
            if combined_pc is not None:
                self.combined_pc_pub.publish(combined_pc)
        except Exception as e:
            self.get_logger().debug(f'Point cloud merging failed: {e}')
    
    def merge_pointclouds(self, point_clouds):
        """Merge multiple point clouds into a single cloud"""
        if len(point_clouds) == 0:
            return None
        
        # If only one point cloud, transform to base frame and return
        if len(point_clouds) == 1:
            camera_name, pc_msg = point_clouds[0]
            return self.transform_pointcloud_to_base(pc_msg, camera_name)
        
        # Combine multiple point clouds
        all_points = []
        
        for camera_name, pc_msg in point_clouds:
            try:
                # Transform to base frame
                transformed_pc = self.transform_pointcloud_to_base(pc_msg, camera_name)
                if transformed_pc is None:
                    continue
                
                # Extract points
                points = list(pc2.read_points(transformed_pc, 
                                            field_names=['x', 'y', 'z', 'rgb'],
                                            skip_nans=True))
                all_points.extend(points)
                
            except Exception as e:
                self.get_logger().debug(f'Failed to process {camera_name} point cloud: {e}')
                continue
        
        if len(all_points) == 0:
            return None
        
        # Create combined point cloud message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.base_frame
        
        combined_pc = pc2.create_cloud_xyz32(header, 
                                           [(p[0], p[1], p[2]) for p in all_points])
        
        return combined_pc
    
    def transform_pointcloud_to_base(self, pc_msg, camera_name):
        """Transform point cloud to base frame"""
        try:
            # Get transform from camera frame to base frame
            if camera_name == 'realsense':
                source_frame = 'd405_color_optical_frame'
            elif camera_name == 'oak_d':
                source_frame = 'oak_d_color_optical_frame'
            else:
                return None
            
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            # For simplicity, return the original point cloud with updated frame
            # In a full implementation, you would apply the transform to each point
            transformed_pc = PointCloud2()
            transformed_pc.header = Header()
            transformed_pc.header.stamp = pc_msg.header.stamp
            transformed_pc.header.frame_id = self.base_frame
            transformed_pc.height = pc_msg.height
            transformed_pc.width = pc_msg.width
            transformed_pc.fields = pc_msg.fields
            transformed_pc.is_bigendian = pc_msg.is_bigendian
            transformed_pc.point_step = pc_msg.point_step
            transformed_pc.row_step = pc_msg.row_step
            transformed_pc.data = pc_msg.data
            transformed_pc.is_dense = pc_msg.is_dense
            
            return transformed_pc
            
        except Exception as e:
            self.get_logger().debug(f'Transform failed for {camera_name}: {e}')
            return None
    
    def check_data_freshness(self):
        """Check and warn about stale camera data"""
        current_time = self.get_clock().now()
        
        for camera, data in self.latest_data.items():
            if data['timestamp'] is not None:
                age = (current_time - data['timestamp']).nanoseconds / 1e9
                if age > 2.0:  # Warn if data is older than 2 seconds
                    self.get_logger().warn(f'{camera} camera data is stale ({age:.1f}s old)')


def main(args=None):
    rclpy.init(args=args)
    
    node = MultiCameraCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()