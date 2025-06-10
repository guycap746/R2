#!/usr/bin/env python3
"""
Advanced Object Placement System for Isaac Sim & Gazebo

This module implements physics-based object placement with collision detection,
stacking validation, and stability analysis for realistic scene generation.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import random
import json
import time
from typing import List, Dict, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum

# ROS2 message types
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker

# Isaac Sim imports (conditional)
try:
    from omni.isaac.core import World
    from omni.isaac.core.objects import DynamicCuboid, DynamicSphere, DynamicCylinder
    from omni.isaac.core.utils.stage import get_current_stage
    from omni.isaac.core.utils.prims import create_prim
    from pxr import UsdGeom, Gf
    ISAAC_AVAILABLE = True
except ImportError:
    ISAAC_AVAILABLE = False

@dataclass
class ObjectBounds:
    """3D bounding box for objects"""
    min_x: float
    min_y: float
    min_z: float
    max_x: float
    max_y: float
    max_z: float
    
    @property
    def center(self) -> Tuple[float, float, float]:
        return (
            (self.min_x + self.max_x) / 2,
            (self.min_y + self.max_y) / 2,
            (self.min_z + self.max_z) / 2
        )
    
    @property
    def dimensions(self) -> Tuple[float, float, float]:
        return (
            self.max_x - self.min_x,
            self.max_y - self.min_y,
            self.max_z - self.min_z
        )
    
    def overlaps_with(self, other: 'ObjectBounds', margin: float = 0.0) -> bool:
        """Check if this bounding box overlaps with another"""
        return not (
            self.max_x + margin < other.min_x or
            self.min_x - margin > other.max_x or
            self.max_y + margin < other.min_y or
            self.min_y - margin > other.max_y or
            self.max_z + margin < other.min_z or
            self.min_z - margin > other.max_z
        )

@dataclass
class PlacedObject:
    """Information about a placed object"""
    id: str
    object_type: str
    position: Tuple[float, float, float]
    rotation: Tuple[float, float, float]
    bounds: ObjectBounds
    mass: float
    support_area: float
    stacked_on: Optional[str] = None
    supports: List[str] = None
    
    def __post_init__(self):
        if self.supports is None:
            self.supports = []

class PlacementStrategy(Enum):
    """Object placement strategies"""
    RANDOM = "random"
    CLUSTERED = "clustered"
    STACKED = "stacked"
    ORGANIZED = "organized"
    SCATTERED = "scattered"

class CollisionAwareObjectPlacer:
    """Collision-aware object placement system"""
    
    def __init__(self, workspace_bounds: ObjectBounds, collision_margin: float = 0.01):
        self.workspace_bounds = workspace_bounds
        self.collision_margin = collision_margin
        self.placed_objects: List[PlacedObject] = []
        self.occupied_spaces: List[ObjectBounds] = []
        
    def find_valid_placement(self, object_config: Dict, max_attempts: int = 50) -> Optional[Tuple[float, float, float]]:
        """Find collision-free placement location"""
        for attempt in range(max_attempts):
            candidate_pos = self.generate_candidate_position(object_config)
            object_bounds = self.create_object_bounds(candidate_pos, object_config)
            
            if self.is_valid_placement(object_bounds):
                return candidate_pos
                
        return None
    
    def generate_candidate_position(self, object_config: Dict) -> Tuple[float, float, float]:
        """Generate random candidate position within workspace"""
        # Get object dimensions to ensure it fits
        dims = self.get_object_dimensions(object_config)
        
        # Generate position within workspace bounds, accounting for object size
        x = random.uniform(
            self.workspace_bounds.min_x + dims[0]/2,
            self.workspace_bounds.max_x - dims[0]/2
        )
        y = random.uniform(
            self.workspace_bounds.min_y + dims[1]/2,
            self.workspace_bounds.max_y - dims[1]/2
        )
        z = self.workspace_bounds.min_z + dims[2]/2
        
        return (x, y, z)
    
    def create_object_bounds(self, position: Tuple[float, float, float], object_config: Dict) -> ObjectBounds:
        """Create bounding box for object at given position"""
        dims = self.get_object_dimensions(object_config)
        
        return ObjectBounds(
            min_x=position[0] - dims[0]/2,
            min_y=position[1] - dims[1]/2,
            min_z=position[2] - dims[2]/2,
            max_x=position[0] + dims[0]/2,
            max_y=position[1] + dims[1]/2,
            max_z=position[2] + dims[2]/2
        )
    
    def get_object_dimensions(self, object_config: Dict) -> Tuple[float, float, float]:
        """Get object dimensions from configuration"""
        if 'scale' in object_config:
            return tuple(object_config['scale'])
        elif 'radius' in object_config:
            r = object_config['radius']
            h = object_config.get('height', r * 2)
            return (r * 2, r * 2, h)
        else:
            # Default dimensions
            return (0.05, 0.05, 0.05)
    
    def is_valid_placement(self, object_bounds: ObjectBounds) -> bool:
        """Check if placement is valid (no collisions, within workspace)"""
        # Check workspace bounds
        if not self.within_workspace(object_bounds):
            return False
        
        # Check collisions with existing objects
        for occupied_space in self.occupied_spaces:
            if object_bounds.overlaps_with(occupied_space, self.collision_margin):
                return False
        
        return True
    
    def within_workspace(self, object_bounds: ObjectBounds) -> bool:
        """Check if object is within workspace bounds"""
        return (
            object_bounds.min_x >= self.workspace_bounds.min_x and
            object_bounds.max_x <= self.workspace_bounds.max_x and
            object_bounds.min_y >= self.workspace_bounds.min_y and
            object_bounds.max_y <= self.workspace_bounds.max_y and
            object_bounds.min_z >= self.workspace_bounds.min_z and
            object_bounds.max_z <= self.workspace_bounds.max_z
        )
    
    def place_object(self, object_config: Dict, placement_id: str) -> Optional[PlacedObject]:
        """Place object and track its position"""
        position = self.find_valid_placement(object_config)
        if position is None:
            return None
        
        # Create object bounds
        object_bounds = self.create_object_bounds(position, object_config)
        
        # Create placed object record
        placed_object = PlacedObject(
            id=placement_id,
            object_type=object_config['type'],
            position=position,
            rotation=object_config.get('rotation', (0, 0, 0)),
            bounds=object_bounds,
            mass=object_config.get('mass', 0.1),
            support_area=self.calculate_support_area(object_config)
        )
        
        # Add to tracking lists
        self.placed_objects.append(placed_object)
        self.occupied_spaces.append(object_bounds)
        
        return placed_object
    
    def calculate_support_area(self, object_config: Dict) -> float:
        """Calculate bottom surface area for stability calculations"""
        dims = self.get_object_dimensions(object_config)
        
        if object_config['type'] == 'sphere':
            # Contact point for sphere
            return 0.001
        elif object_config['type'] == 'cylinder':
            return np.pi * (dims[0]/2) ** 2
        else:  # cube and others
            return dims[0] * dims[1]
    
    def clear_objects(self):
        """Clear all placed objects"""
        self.placed_objects.clear()
        self.occupied_spaces.clear()
    
    def get_placement_density(self) -> float:
        """Calculate current workspace utilization"""
        if not self.occupied_spaces:
            return 0.0
        
        workspace_volume = (
            (self.workspace_bounds.max_x - self.workspace_bounds.min_x) *
            (self.workspace_bounds.max_y - self.workspace_bounds.min_y) *
            (self.workspace_bounds.max_z - self.workspace_bounds.min_z)
        )
        
        occupied_volume = sum([
            bounds.dimensions[0] * bounds.dimensions[1] * bounds.dimensions[2]
            for bounds in self.occupied_spaces
        ])
        
        return occupied_volume / workspace_volume

class StackingValidator:
    """Physics-based stacking validation system"""
    
    def __init__(self):
        self.max_stack_height = 5
        self.stability_threshold = 0.8
        self.gravity = 9.81
    
    def can_stack_on(self, base_object: PlacedObject, new_object_config: Dict) -> bool:
        """Determine if object can be stacked on base"""
        # Check geometric compatibility
        if not self.geometric_compatibility(base_object, new_object_config):
            return False
        
        # Check mass compatibility
        if not self.mass_compatibility(base_object, new_object_config):
            return False
        
        # Check stability prediction
        stability = self.predict_stability(base_object, new_object_config)
        return stability > self.stability_threshold
    
    def geometric_compatibility(self, base_object: PlacedObject, new_object_config: Dict) -> bool:
        """Check if objects are geometrically compatible for stacking"""
        new_dims = self.get_object_dimensions(new_object_config)
        base_dims = base_object.bounds.dimensions
        
        # Top object should be smaller or similar size to base
        if new_dims[0] > base_dims[0] * 1.2 or new_dims[1] > base_dims[1] * 1.2:
            return False
        
        # Check if base object is suitable for stacking
        if base_object.object_type == 'sphere':
            return False  # Can't stack on spheres reliably
        
        return True
    
    def mass_compatibility(self, base_object: PlacedObject, new_object_config: Dict) -> bool:
        """Check if base can support the mass of new object"""
        new_mass = new_object_config.get('mass', 0.1)
        
        # Simple heuristic: top object should be lighter than base
        return new_mass <= base_object.mass * 2.0
    
    def predict_stability(self, base_object: PlacedObject, new_object_config: Dict) -> float:
        """Predict stack stability using physics simulation"""
        # Center of mass analysis
        com_offset = self.calculate_com_offset(base_object, new_object_config)
        base_support_area = base_object.support_area
        
        # Stability decreases with COM offset from center
        if base_support_area > 0:
            stability = max(0, 1.0 - (com_offset / np.sqrt(base_support_area)))
        else:
            stability = 0.0
        
        # Adjust for object types
        if base_object.object_type == 'cylinder' and new_object_config['type'] == 'cube':
            stability *= 0.8  # Less stable combination
        elif base_object.object_type == 'cube' and new_object_config['type'] == 'cube':
            stability *= 1.1  # More stable combination
        
        return min(stability, 1.0)
    
    def calculate_com_offset(self, base_object: PlacedObject, new_object_config: Dict) -> float:
        """Calculate center of mass offset from base center"""
        # Simplified calculation - in practice would consider exact geometry
        new_dims = self.get_object_dimensions(new_object_config)
        base_dims = base_object.bounds.dimensions
        
        # Assume some random offset for realistic stacking
        max_offset = min(base_dims[0], base_dims[1]) * 0.2
        return random.uniform(0, max_offset)
    
    def get_object_dimensions(self, object_config: Dict) -> Tuple[float, float, float]:
        """Get object dimensions from configuration"""
        if 'scale' in object_config:
            return tuple(object_config['scale'])
        elif 'radius' in object_config:
            r = object_config['radius']
            h = object_config.get('height', r * 2)
            return (r * 2, r * 2, h)
        else:
            return (0.05, 0.05, 0.05)
    
    def find_stacking_position(self, base_object: PlacedObject, new_object_config: Dict) -> Optional[Tuple[float, float, float]]:
        """Find optimal position for stacking object on base"""
        if not self.can_stack_on(base_object, new_object_config):
            return None
        
        new_dims = self.get_object_dimensions(new_object_config)
        
        # Position on top of base object with slight randomness
        base_top_z = base_object.bounds.max_z
        new_z = base_top_z + new_dims[2] / 2
        
        # Small random offset for realism
        offset_x = random.uniform(-0.005, 0.005)
        offset_y = random.uniform(-0.005, 0.005)
        
        new_x = base_object.position[0] + offset_x
        new_y = base_object.position[1] + offset_y
        
        return (new_x, new_y, new_z)

class AdvancedObjectPlacer(Node):
    """ROS2 node for advanced object placement"""
    
    def __init__(self):
        super().__init__('advanced_object_placer')
        
        # Parameters
        self.declare_parameter('workspace_min', [-0.1, -0.3, 0.0])
        self.declare_parameter('workspace_max', [0.8, 0.3, 0.5])
        self.declare_parameter('collision_margin', 0.01)
        self.declare_parameter('max_objects', 20)
        self.declare_parameter('enable_stacking', True)
        
        workspace_min = self.get_parameter('workspace_min').get_parameter_value().double_array_value
        workspace_max = self.get_parameter('workspace_max').get_parameter_value().double_array_value
        self.collision_margin = self.get_parameter('collision_margin').get_parameter_value().double_value
        self.max_objects = self.get_parameter('max_objects').get_parameter_value().integer_value
        self.enable_stacking = self.get_parameter('enable_stacking').get_parameter_value().bool_value
        
        # Create workspace bounds
        self.workspace_bounds = ObjectBounds(
            min_x=workspace_min[0], min_y=workspace_min[1], min_z=workspace_min[2],
            max_x=workspace_max[0], max_y=workspace_max[1], max_z=workspace_max[2]
        )
        
        # Initialize placement systems
        self.object_placer = CollisionAwareObjectPlacer(self.workspace_bounds, self.collision_margin)
        self.stacking_validator = StackingValidator()
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/object_placer/status', 10)
        self.visualization_pub = self.create_publisher(MarkerArray, '/object_placer/visualization', 10)
        self.placement_complete_pub = self.create_publisher(Bool, '/object_placer/complete', 10)
        
        # Subscribers
        self.place_objects_sub = self.create_subscription(
            Int32, '/object_placer/place_objects', self.place_objects_callback, 10)
        self.clear_objects_sub = self.create_subscription(
            Bool, '/object_placer/clear', self.clear_objects_callback, 10)
        
        # Isaac Sim world reference
        self.world = None
        if ISAAC_AVAILABLE:
            self.world = World.instance()
        
        # Object library
        self.object_library = self.create_object_library()
        
        self.get_logger().info('Advanced Object Placer initialized')
        self.publish_status('Object placer ready')
    
    def create_object_library(self) -> List[Dict]:
        """Create library of objects for placement"""
        return [
            # Basic geometric objects
            {'type': 'cube', 'scale': [0.03, 0.03, 0.03], 'mass': 0.1, 'color': [0.8, 0.2, 0.2]},
            {'type': 'cube', 'scale': [0.025, 0.025, 0.025], 'mass': 0.08, 'color': [0.2, 0.8, 0.2]},
            {'type': 'sphere', 'radius': 0.02, 'mass': 0.06, 'color': [0.2, 0.2, 0.8]},
            {'type': 'sphere', 'radius': 0.015, 'mass': 0.04, 'color': [0.8, 0.8, 0.2]},
            {'type': 'cylinder', 'radius': 0.015, 'height': 0.04, 'mass': 0.07, 'color': [0.8, 0.2, 0.8]},
            {'type': 'cylinder', 'radius': 0.02, 'height': 0.03, 'mass': 0.09, 'color': [0.2, 0.8, 0.8]},
            
            # Varied sizes for testing
            {'type': 'cube', 'scale': [0.04, 0.04, 0.02], 'mass': 0.12, 'color': [0.5, 0.5, 0.5]},
            {'type': 'cube', 'scale': [0.02, 0.02, 0.05], 'mass': 0.08, 'color': [0.9, 0.5, 0.1]},
        ]
    
    def place_objects_callback(self, msg):
        """Handle object placement request"""
        num_objects = msg.data if msg.data > 0 else random.randint(3, 8)
        self.place_multiple_objects(num_objects)
    
    def clear_objects_callback(self, msg):
        """Handle clear objects request"""
        if msg.data:
            self.clear_all_objects()
    
    def place_multiple_objects(self, num_objects: int, strategy: PlacementStrategy = PlacementStrategy.RANDOM):
        """Place multiple objects using specified strategy"""
        self.publish_status(f'Placing {num_objects} objects using {strategy.value} strategy')
        
        # Clear existing objects
        self.clear_all_objects()
        
        placed_count = 0
        stacked_count = 0
        
        for i in range(min(num_objects, self.max_objects)):
            # Select random object from library
            object_config = random.choice(self.object_library).copy()
            object_id = f'object_{i:03d}'
            
            # Try stacking first if enabled and we have base objects
            placed_object = None
            if (self.enable_stacking and len(self.object_placer.placed_objects) > 0 and 
                random.random() < 0.3):  # 30% chance of stacking
                
                placed_object = self.try_stack_object(object_config, object_id)
                if placed_object:
                    stacked_count += 1
            
            # If stacking failed or not attempted, place normally
            if placed_object is None:
                placed_object = self.object_placer.place_object(object_config, object_id)
            
            if placed_object:
                # Create object in Isaac Sim
                if self.world:
                    self.create_isaac_object(placed_object, object_config)
                
                placed_count += 1
                self.get_logger().debug(f'Placed {object_id} at {placed_object.position}')
            else:
                self.get_logger().warning(f'Failed to place object {i}')
        
        # Publish results
        density = self.object_placer.get_placement_density()
        status_msg = f'Placed {placed_count} objects ({stacked_count} stacked), density: {density:.2f}'
        self.publish_status(status_msg)
        
        # Publish visualization
        self.publish_visualization()
        
        # Signal completion
        complete_msg = Bool()
        complete_msg.data = True
        self.placement_complete_pub.publish(complete_msg)
    
    def try_stack_object(self, object_config: Dict, object_id: str) -> Optional[PlacedObject]:
        """Try to stack object on existing objects"""
        # Find suitable base objects
        suitable_bases = []
        for placed_obj in self.object_placer.placed_objects:
            if self.stacking_validator.can_stack_on(placed_obj, object_config):
                suitable_bases.append(placed_obj)
        
        if not suitable_bases:
            return None
        
        # Select random base
        base_object = random.choice(suitable_bases)
        
        # Find stacking position
        stack_position = self.stacking_validator.find_stacking_position(base_object, object_config)
        if stack_position is None:
            return None
        
        # Create object bounds at stack position
        object_bounds = self.object_placer.create_object_bounds(stack_position, object_config)
        
        # Check for collisions with other objects
        for occupied_space in self.object_placer.occupied_spaces:
            if object_bounds.overlaps_with(occupied_space, self.collision_margin):
                return None
        
        # Create stacked object
        placed_object = PlacedObject(
            id=object_id,
            object_type=object_config['type'],
            position=stack_position,
            rotation=object_config.get('rotation', (0, 0, 0)),
            bounds=object_bounds,
            mass=object_config.get('mass', 0.1),
            support_area=self.object_placer.calculate_support_area(object_config),
            stacked_on=base_object.id
        )
        
        # Update base object's support list
        base_object.supports.append(object_id)
        
        # Add to tracking lists
        self.object_placer.placed_objects.append(placed_object)
        self.object_placer.occupied_spaces.append(object_bounds)
        
        return placed_object
    
    def create_isaac_object(self, placed_object: PlacedObject, object_config: Dict):
        """Create object in Isaac Sim"""
        if not ISAAC_AVAILABLE or not self.world:
            return
        
        try:
            prim_path = f'/World/AdvancedObjects/{placed_object.id}'
            position = np.array(placed_object.position)
            color = np.array(object_config.get('color', [0.5, 0.5, 0.5]))
            
            if placed_object.object_type == 'cube':
                scale = np.array(object_config['scale'])
                obj = DynamicCuboid(
                    prim_path=prim_path,
                    name=placed_object.id,
                    position=position,
                    scale=scale,
                    color=color
                )
            elif placed_object.object_type == 'sphere':
                radius = object_config['radius']
                obj = DynamicSphere(
                    prim_path=prim_path,
                    name=placed_object.id,
                    position=position,
                    radius=radius,
                    color=color
                )
            elif placed_object.object_type == 'cylinder':
                radius = object_config['radius']
                height = object_config['height']
                obj = DynamicCylinder(
                    prim_path=prim_path,
                    name=placed_object.id,
                    position=position,
                    radius=radius,
                    height=height,
                    color=color
                )
            
            # Set mass
            if hasattr(obj, 'set_mass'):
                obj.set_mass(placed_object.mass)
            
            # Add to world
            self.world.scene.add(obj)
            
        except Exception as e:
            self.get_logger().error(f'Failed to create Isaac object {placed_object.id}: {e}')
    
    def clear_all_objects(self):
        """Clear all placed objects"""
        self.object_placer.clear_objects()
        
        # Clear from Isaac Sim
        if ISAAC_AVAILABLE and self.world:
            try:
                stage = get_current_stage()
                objects_prim = stage.GetPrimAtPath('/World/AdvancedObjects')
                if objects_prim:
                    stage.RemovePrim('/World/AdvancedObjects')
                
                # Recreate objects container
                create_prim('/World/AdvancedObjects', 'Xform')
                
            except Exception as e:
                self.get_logger().error(f'Failed to clear Isaac objects: {e}')
        
        self.publish_status('All objects cleared')
    
    def publish_visualization(self):
        """Publish visualization markers for placed objects"""
        marker_array = MarkerArray()
        
        for i, placed_obj in enumerate(self.object_placer.placed_objects):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'placed_objects'
            marker.id = i
            marker.type = Marker.CUBE if placed_obj.object_type == 'cube' else Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = placed_obj.position[0]
            marker.pose.position.y = placed_obj.position[1]
            marker.pose.position.z = placed_obj.position[2]
            
            # Orientation (identity quaternion)
            marker.pose.orientation.w = 1.0
            
            # Scale
            dims = placed_obj.bounds.dimensions
            marker.scale.x = dims[0]
            marker.scale.y = dims[1]
            marker.scale.z = dims[2]
            
            # Color (green for base objects, blue for stacked)
            if placed_obj.stacked_on:
                marker.color.r = 0.2
                marker.color.g = 0.2
                marker.color.b = 0.8
            else:
                marker.color.r = 0.2
                marker.color.g = 0.8
                marker.color.b = 0.2
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
        
        self.visualization_pub.publish(marker_array)
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[OBJECT_PLACER] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        placer = AdvancedObjectPlacer()
        rclpy.spin(placer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Object placer error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()