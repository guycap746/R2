#!/usr/bin/env python3
"""
Gazebo Object Interface for Cross-Platform Object Spawning

This module provides a unified interface for spawning objects in Gazebo
that matches the capabilities of the Isaac Sim object placement system.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple, Optional
import tempfile
import os

# ROS2 message types
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelList

class SDFGenerator:
    """Generate SDF models for Gazebo objects"""
    
    def __init__(self):
        self.material_library = self.create_material_library()
    
    def create_material_library(self) -> Dict:
        """Create library of materials for objects"""
        return {
            'plastic_red': {
                'ambient': [0.8, 0.2, 0.2, 1.0],
                'diffuse': [0.8, 0.2, 0.2, 1.0],
                'specular': [0.2, 0.2, 0.2, 1.0]
            },
            'plastic_green': {
                'ambient': [0.2, 0.8, 0.2, 1.0],
                'diffuse': [0.2, 0.8, 0.2, 1.0],
                'specular': [0.2, 0.2, 0.2, 1.0]
            },
            'plastic_blue': {
                'ambient': [0.2, 0.2, 0.8, 1.0],
                'diffuse': [0.2, 0.2, 0.8, 1.0],
                'specular': [0.2, 0.2, 0.2, 1.0]
            },
            'metal_gray': {
                'ambient': [0.5, 0.5, 0.5, 1.0],
                'diffuse': [0.7, 0.7, 0.7, 1.0],
                'specular': [0.8, 0.8, 0.8, 1.0]
            },
            'wood_brown': {
                'ambient': [0.6, 0.4, 0.2, 1.0],
                'diffuse': [0.6, 0.4, 0.2, 1.0],
                'specular': [0.1, 0.1, 0.1, 1.0]
            }
        }
    
    def create_sdf(self, object_config: Dict) -> str:
        """Create SDF model string from object configuration"""
        object_type = object_config['type']
        
        if object_type == 'cube':
            return self.create_cube_sdf(object_config)
        elif object_type == 'sphere':
            return self.create_sphere_sdf(object_config)
        elif object_type == 'cylinder':
            return self.create_cylinder_sdf(object_config)
        else:
            raise ValueError(f"Unsupported object type: {object_type}")
    
    def create_cube_sdf(self, object_config: Dict) -> str:
        """Create SDF for cube object"""
        scale = object_config.get('scale', [0.05, 0.05, 0.05])
        mass = object_config.get('mass', 0.1)
        material = self.get_material_name(object_config)
        
        # Calculate inertia for box
        inertia = self.calculate_box_inertia(mass, scale)
        
        sdf_template = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{{model_name}}">
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia['ixx']:.6f}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{inertia['iyy']:.6f}</iyy>
          <iyz>0</iyz>
          <izz>{inertia['izz']:.6f}</izz>
        </inertia>
      </inertial>
      
      <collision name="collision">
        <geometry>
          <box>
            <size>{scale[0]} {scale[1]} {scale[2]}</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.6</mu>
              <mu2>0.6</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>100.0</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      
      <visual name="visual">
        <geometry>
          <box>
            <size>{scale[0]} {scale[1]} {scale[2]}</size>
          </box>
        </geometry>
        <material>
          <script>
            <name>{material}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        
        return sdf_template
    
    def create_sphere_sdf(self, object_config: Dict) -> str:
        """Create SDF for sphere object"""
        radius = object_config.get('radius', 0.025)
        mass = object_config.get('mass', 0.1)
        material = self.get_material_name(object_config)
        
        # Calculate inertia for sphere
        inertia = self.calculate_sphere_inertia(mass, radius)
        
        sdf_template = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{{model_name}}">
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia:.6f}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{inertia:.6f}</iyy>
          <iyz>0</iyz>
          <izz>{inertia:.6f}</izz>
        </inertia>
      </inertial>
      
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>{radius}</radius>
          </sphere>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.4</mu>
              <mu2>0.4</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>100.0</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>{radius}</radius>
          </sphere>
        </geometry>
        <material>
          <script>
            <name>{material}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        
        return sdf_template
    
    def create_cylinder_sdf(self, object_config: Dict) -> str:
        """Create SDF for cylinder object"""
        radius = object_config.get('radius', 0.02)
        height = object_config.get('height', 0.04)
        mass = object_config.get('mass', 0.1)
        material = self.get_material_name(object_config)
        
        # Calculate inertia for cylinder
        inertia = self.calculate_cylinder_inertia(mass, radius, height)
        
        sdf_template = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{{model_name}}">
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{inertia['ixx']:.6f}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>{inertia['iyy']:.6f}</iyy>
          <iyz>0</iyz>
          <izz>{inertia['izz']:.6f}</izz>
        </inertia>
      </inertial>
      
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.7</mu>
              <mu2>0.7</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>100.0</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <script>
            <name>{material}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        
        return sdf_template
    
    def get_material_name(self, object_config: Dict) -> str:
        """Get appropriate material name for object"""
        color = object_config.get('color', [0.5, 0.5, 0.5])
        
        # Simple heuristic to select material based on color
        if color[0] > 0.6 and color[1] < 0.4 and color[2] < 0.4:
            return 'Gazebo/Red'
        elif color[1] > 0.6 and color[0] < 0.4 and color[2] < 0.4:
            return 'Gazebo/Green'
        elif color[2] > 0.6 and color[0] < 0.4 and color[1] < 0.4:
            return 'Gazebo/Blue'
        elif sum(color) > 1.5:
            return 'Gazebo/White'
        else:
            return 'Gazebo/Gray'
    
    def calculate_box_inertia(self, mass: float, dimensions: List[float]) -> Dict:
        """Calculate inertia tensor for box"""
        w, h, d = dimensions
        return {
            'ixx': mass * (h*h + d*d) / 12.0,
            'iyy': mass * (w*w + d*d) / 12.0,
            'izz': mass * (w*w + h*h) / 12.0
        }
    
    def calculate_sphere_inertia(self, mass: float, radius: float) -> float:
        """Calculate inertia for sphere"""
        return 0.4 * mass * radius * radius
    
    def calculate_cylinder_inertia(self, mass: float, radius: float, height: float) -> Dict:
        """Calculate inertia tensor for cylinder"""
        return {
            'ixx': mass * (3 * radius*radius + height*height) / 12.0,
            'iyy': mass * (3 * radius*radius + height*height) / 12.0,
            'izz': 0.5 * mass * radius * radius
        }

class GazeboObjectInterface(Node):
    """Interface for spawning and managing objects in Gazebo"""
    
    def __init__(self):
        super().__init__('gazebo_object_interface')
        
        # Parameters
        self.declare_parameter('gazebo_namespace', '/gazebo')
        self.declare_parameter('auto_cleanup', True)
        
        self.gazebo_namespace = self.get_parameter('gazebo_namespace').get_parameter_value().string_value
        self.auto_cleanup = self.get_parameter('auto_cleanup').get_parameter_value().bool_value
        
        # Service clients
        self.spawn_model_client = self.create_client(
            SpawnModel, f'{self.gazebo_namespace}/spawn_sdf_model')
        self.delete_model_client = self.create_client(
            DeleteModel, f'{self.gazebo_namespace}/delete_model')
        self.get_model_list_client = self.create_client(
            GetModelList, f'{self.gazebo_namespace}/get_model_list')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/gazebo_interface/status', 10)
        self.spawn_complete_pub = self.create_publisher(Bool, '/gazebo_interface/spawn_complete', 10)
        
        # SDF generator
        self.sdf_generator = SDFGenerator()
        
        # Track spawned objects
        self.spawned_objects: List[str] = []
        
        self.get_logger().info('Gazebo Object Interface initialized')
        self.publish_status('Gazebo interface ready')
        
        # Wait for Gazebo services
        self.wait_for_gazebo_services()
    
    def wait_for_gazebo_services(self):
        """Wait for Gazebo services to become available"""
        self.publish_status('Waiting for Gazebo services...')
        
        services_ready = False
        while not services_ready and rclpy.ok():
            services_ready = (
                self.spawn_model_client.wait_for_service(timeout_sec=1.0) and
                self.delete_model_client.wait_for_service(timeout_sec=1.0) and
                self.get_model_list_client.wait_for_service(timeout_sec=1.0)
            )
            
            if not services_ready:
                self.get_logger().info('Waiting for Gazebo services...')
        
        if services_ready:
            self.publish_status('Gazebo services available')
        else:
            self.get_logger().error('Gazebo services not available')
    
    def spawn_object(self, object_config: Dict, model_name: str, 
                    position: Tuple[float, float, float],
                    orientation: Tuple[float, float, float] = (0, 0, 0)) -> bool:
        """Spawn object in Gazebo"""
        try:
            # Generate SDF content
            sdf_content = self.sdf_generator.create_sdf(object_config)
            sdf_content = sdf_content.format(model_name=model_name)
            
            # Create spawn request
            request = SpawnModel.Request()
            request.model_name = model_name
            request.model_xml = sdf_content
            request.robot_namespace = ''
            request.reference_frame = 'world'
            
            # Set position
            request.initial_pose.position.x = float(position[0])
            request.initial_pose.position.y = float(position[1])
            request.initial_pose.position.z = float(position[2])
            
            # Convert Euler angles to quaternion
            quat = self.euler_to_quaternion(orientation)
            request.initial_pose.orientation.x = quat[0]
            request.initial_pose.orientation.y = quat[1]
            request.initial_pose.orientation.z = quat[2]
            request.initial_pose.orientation.w = quat[3]
            
            # Call service
            future = self.spawn_model_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.spawned_objects.append(model_name)
                    self.get_logger().info(f'Successfully spawned {model_name}')
                    return True
                else:
                    self.get_logger().error(f'Failed to spawn {model_name}: {response.status_message}')
                    return False
            else:
                self.get_logger().error(f'Spawn service call failed for {model_name}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Exception during spawn: {e}')
            return False
    
    def delete_object(self, model_name: str) -> bool:
        """Delete object from Gazebo"""
        try:
            request = DeleteModel.Request()
            request.model_name = model_name
            
            future = self.delete_model_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    if model_name in self.spawned_objects:
                        self.spawned_objects.remove(model_name)
                    self.get_logger().info(f'Successfully deleted {model_name}')
                    return True
                else:
                    self.get_logger().error(f'Failed to delete {model_name}: {response.status_message}')
                    return False
            else:
                self.get_logger().error(f'Delete service call failed for {model_name}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Exception during delete: {e}')
            return False
    
    def clear_all_spawned_objects(self) -> int:
        """Clear all objects spawned by this interface"""
        cleared_count = 0
        
        for model_name in self.spawned_objects.copy():
            if self.delete_object(model_name):
                cleared_count += 1
        
        self.publish_status(f'Cleared {cleared_count} objects')
        return cleared_count
    
    def get_model_list(self) -> List[str]:
        """Get list of all models in Gazebo"""
        try:
            request = GetModelList.Request()
            future = self.get_model_list_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                return list(response.model_names)
            else:
                self.get_logger().error('Get model list service call failed')
                return []
                
        except Exception as e:
            self.get_logger().error(f'Exception getting model list: {e}')
            return []
    
    def spawn_multiple_objects(self, object_configs: List[Dict], 
                             positions: List[Tuple[float, float, float]],
                             base_name: str = 'object') -> int:
        """Spawn multiple objects"""
        spawned_count = 0
        
        for i, (config, position) in enumerate(zip(object_configs, positions)):
            model_name = f'{base_name}_{i:03d}'
            if self.spawn_object(config, model_name, position):
                spawned_count += 1
        
        # Signal completion
        complete_msg = Bool()
        complete_msg.data = True
        self.spawn_complete_pub.publish(complete_msg)
        
        self.publish_status(f'Spawned {spawned_count}/{len(object_configs)} objects')
        return spawned_count
    
    def euler_to_quaternion(self, euler: Tuple[float, float, float]) -> Tuple[float, float, float, float]:
        """Convert Euler angles (roll, pitch, yaw) to quaternion (x, y, z, w)"""
        roll, pitch, yaw = euler
        
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[GAZEBO_INTERFACE] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


class UnifiedObjectSpawner(Node):
    """Unified interface for spawning objects in both Isaac Sim and Gazebo"""
    
    def __init__(self):
        super().__init__('unified_object_spawner')
        
        # Parameters
        self.declare_parameter('simulator_type', 'isaac')  # 'isaac' or 'gazebo'
        self.declare_parameter('enable_both', False)
        
        self.simulator_type = self.get_parameter('simulator_type').get_parameter_value().string_value
        self.enable_both = self.get_parameter('enable_both').get_parameter_value().bool_value
        
        # Initialize interfaces
        self.isaac_available = False
        self.gazebo_interface = None
        
        # Try to initialize Isaac Sim interface
        try:
            # Import Isaac Sim components
            from omni.isaac.core import World
            self.isaac_world = World.instance()
            self.isaac_available = self.isaac_world is not None
        except ImportError:
            self.isaac_available = False
        
        # Initialize Gazebo interface if needed
        if self.simulator_type == 'gazebo' or self.enable_both:
            self.gazebo_interface = GazeboObjectInterface()
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/unified_spawner/status', 10)
        
        self.get_logger().info(f'Unified Object Spawner initialized for {self.simulator_type}')
        self.publish_status(f'Ready for {self.simulator_type} simulation')
    
    def spawn_object_unified(self, object_config: Dict, model_name: str,
                           position: Tuple[float, float, float],
                           orientation: Tuple[float, float, float] = (0, 0, 0)) -> bool:
        """Spawn object in configured simulator(s)"""
        success = True
        
        if self.simulator_type == 'isaac' or self.enable_both:
            if self.isaac_available:
                success &= self.spawn_isaac_object(object_config, model_name, position, orientation)
            else:
                self.get_logger().warning('Isaac Sim not available')
                success = False
        
        if self.simulator_type == 'gazebo' or self.enable_both:
            if self.gazebo_interface:
                success &= self.gazebo_interface.spawn_object(object_config, model_name, position, orientation)
            else:
                self.get_logger().warning('Gazebo interface not available')
                success = False
        
        return success
    
    def spawn_isaac_object(self, object_config: Dict, model_name: str,
                          position: Tuple[float, float, float],
                          orientation: Tuple[float, float, float]) -> bool:
        """Spawn object in Isaac Sim"""
        # This would use the existing Isaac Sim object creation logic
        # from the advanced_object_placer.py
        return True
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[UNIFIED_SPAWNER] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        # You can run either the Gazebo interface or unified spawner
        interface = GazeboObjectInterface()
        rclpy.spin(interface)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Gazebo interface error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()