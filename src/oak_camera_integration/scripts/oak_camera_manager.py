#!/usr/bin/env python3
"""
OAK Camera Manager Node

Manages OAK camera operations and provides unified interface for:
- Camera switching between OAK-D and OAK-1
- Parameter management and configuration
- Health monitoring and error recovery
- Performance optimization
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import yaml
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from depthai_ros_msgs.msg import HandLandmarkArray

class OAKCameraManager(Node):
    def __init__(self):
        super().__init__('oak_camera_manager')
        
        # Parameters
        self.declare_parameter('oak_d_enabled', True)
        self.declare_parameter('oak_1_enabled', False) 
        self.declare_parameter('auto_switch_cameras', True)
        self.declare_parameter('health_check_interval', 5.0)
        self.declare_parameter('config_update_interval', 1.0)
        
        self.oak_d_enabled = self.get_parameter('oak_d_enabled').value
        self.oak_1_enabled = self.get_parameter('oak_1_enabled').value
        self.auto_switch = self.get_parameter('auto_switch_cameras').value
        self.health_interval = self.get_parameter('health_check_interval').value
        self.config_interval = self.get_parameter('config_update_interval').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Camera health monitoring
        self.camera_health = {
            'oak_d': {'active': False, 'last_data': None, 'fps': 0.0},
            'oak_1': {'active': False, 'last_data': None, 'fps': 0.0}
        }
        
        # Subscribers for health monitoring
        if self.oak_d_enabled:
            self.oak_d_health_sub = self.create_subscription(
                Image, '/oak_d/oak_d/color/image_raw',
                lambda msg: self.update_camera_health('oak_d', msg), 
                sensor_qos)
                
        if self.oak_1_enabled:
            self.oak_1_health_sub = self.create_subscription(
                Image, '/oak_1/oak_1/image_raw',
                lambda msg: self.update_camera_health('oak_1', msg),
                sensor_qos)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/oak_cameras/status', 10)
        self.active_camera_pub = self.create_publisher(String, '/oak_cameras/active', 10)
        self.health_pub = self.create_publisher(String, '/oak_cameras/health', 10)
        
        # Timers
        self.health_timer = self.create_timer(self.health_interval, self.health_check_callback)
        self.status_timer = self.create_timer(1.0, self.status_callback)
        
        # State
        self.active_camera = 'oak_d' if self.oak_d_enabled else 'oak_1'
        
        self.get_logger().info('OAK Camera Manager started')
        self.get_logger().info(f'Managing cameras: OAK-D={self.oak_d_enabled}, OAK-1={self.oak_1_enabled}')
        
    def update_camera_health(self, camera_name, msg):
        """Update health status for a camera"""
        current_time = self.get_clock().now()
        
        if self.camera_health[camera_name]['last_data'] is not None:
            # Calculate FPS
            time_diff = (current_time - self.camera_health[camera_name]['last_data']).nanoseconds / 1e9
            if time_diff > 0:
                self.camera_health[camera_name]['fps'] = 1.0 / time_diff
        
        self.camera_health[camera_name]['active'] = True
        self.camera_health[camera_name]['last_data'] = current_time
    
    def health_check_callback(self):
        """Periodic health check for all cameras"""
        current_time = self.get_clock().now()
        health_report = []
        
        for camera, health in self.camera_health.items():
            if health['last_data'] is not None:
                age = (current_time - health['last_data']).nanoseconds / 1e9
                if age > 5.0:  # No data for 5 seconds
                    health['active'] = False
                    health['fps'] = 0.0
                    health_report.append(f"{camera}:TIMEOUT")
                else:
                    health_report.append(f"{camera}:OK@{health['fps']:.1f}fps")
            else:
                health_report.append(f"{camera}:NO_DATA")
        
        # Publish health report
        health_msg = String()
        health_msg.data = ' | '.join(health_report)
        self.health_pub.publish(health_msg)
        
        # Auto-switch if needed
        if self.auto_switch:
            self.auto_switch_camera()
    
    def auto_switch_camera(self):
        """Automatically switch to best available camera"""
        # Check if current active camera is healthy
        current_health = self.camera_health.get(self.active_camera, {})
        
        if not current_health.get('active', False):
            # Current camera is not healthy, try to switch
            for camera, health in self.camera_health.items():
                if health.get('active', False) and camera != self.active_camera:
                    self.get_logger().info(f'Switching from {self.active_camera} to {camera}')
                    self.active_camera = camera
                    break
    
    def status_callback(self):
        """Publish current status"""
        # Status message
        status_parts = []
        for camera, health in self.camera_health.items():
            enabled = (camera == 'oak_d' and self.oak_d_enabled) or \
                     (camera == 'oak_1' and self.oak_1_enabled)
            if enabled:
                state = 'ACTIVE' if health.get('active', False) else 'INACTIVE'
                status_parts.append(f"{camera}:{state}")
        
        status_msg = String()
        status_msg.data = ' | '.join(status_parts)
        self.status_pub.publish(status_msg)
        
        # Active camera
        active_msg = String()
        active_msg.data = self.active_camera
        self.active_camera_pub.publish(active_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = OAKCameraManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()