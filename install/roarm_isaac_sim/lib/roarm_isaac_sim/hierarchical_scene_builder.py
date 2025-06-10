#!/usr/bin/env python3
"""
Hierarchical Scene Builder for Advanced Object Arrangements

This module implements intelligent scene generation with realistic object
placement patterns, procedural clutter generation, and context-aware arrangements.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import random
import json
import time
from typing import Dict, List, Tuple, Optional, Callable
from dataclasses import dataclass
from enum import Enum

# ROS2 message types
from std_msgs.msg import String, Bool, Int32, Float32
from geometry_msgs.msg import Pose

# Import our object placement components
from advanced_object_placer import CollisionAwareObjectPlacer, StackingValidator, PlacedObject, ObjectBounds, PlacementStrategy

class SceneComplexity(Enum):
    """Scene complexity levels"""
    SIMPLE = 1      # 3-5 objects, minimal stacking
    MODERATE = 2    # 6-10 objects, some stacking
    COMPLEX = 3     # 11-15 objects, multi-layer stacking
    CLUTTERED = 4   # 16+ objects, dense arrangements

class SceneType(Enum):
    """Predefined scene types"""
    KITCHEN_COUNTER = "kitchen_counter"
    OFFICE_DESK = "office_desk"
    WORKSHOP_TABLE = "workshop_table"
    TOY_PLAYAREA = "toy_playarea"
    RANDOM_CLUTTER = "random_clutter"
    PRECISION_TASK = "precision_task"

@dataclass
class SceneTemplate:
    """Template for scene generation"""
    name: str
    object_types: List[str]
    typical_arrangements: List[str]
    preferred_materials: List[str]
    complexity_weights: Dict[SceneComplexity, float]
    stacking_probability: float
    clustering_tendency: float

class ProceduralObjectLibrary:
    """Procedural object generation and management"""
    
    def __init__(self):
        self.object_templates = {
            'household': {
                'cup': {'base_scale': [0.03, 0.03, 0.04], 'type': 'cylinder', 'mass_density': 800},
                'bowl': {'base_scale': [0.04, 0.04, 0.02], 'type': 'cylinder', 'mass_density': 700},
                'plate': {'base_scale': [0.05, 0.05, 0.01], 'type': 'cylinder', 'mass_density': 900},
                'bottle': {'base_scale': [0.025, 0.025, 0.08], 'type': 'cylinder', 'mass_density': 300},
                'can': {'base_scale': [0.032, 0.032, 0.06], 'type': 'cylinder', 'mass_density': 400}
            },
            'tools': {
                'screwdriver': {'base_scale': [0.01, 0.01, 0.12], 'type': 'cylinder', 'mass_density': 2000},
                'wrench': {'base_scale': [0.03, 0.015, 0.08], 'type': 'cube', 'mass_density': 2200},
                'hammer': {'base_scale': [0.04, 0.02, 0.15], 'type': 'cube', 'mass_density': 1800},
                'pliers': {'base_scale': [0.02, 0.01, 0.10], 'type': 'cube', 'mass_density': 2100}
            },
            'toys': {
                'block': {'base_scale': [0.03, 0.03, 0.03], 'type': 'cube', 'mass_density': 600},
                'ball': {'base_scale': [0.025, 0.025, 0.025], 'type': 'sphere', 'mass_density': 500},
                'car': {'base_scale': [0.06, 0.03, 0.02], 'type': 'cube', 'mass_density': 800}
            },
            'office': {
                'stapler': {'base_scale': [0.08, 0.04, 0.03], 'type': 'cube', 'mass_density': 1200},
                'pen': {'base_scale': [0.008, 0.008, 0.12], 'type': 'cylinder', 'mass_density': 900},
                'book': {'base_scale': [0.15, 0.20, 0.03], 'type': 'cube', 'mass_density': 800},
                'folder': {'base_scale': [0.22, 0.30, 0.005], 'type': 'cube', 'mass_density': 400}
            }
        }
        
        self.material_properties = {
            'plastic': {'density_range': [800, 1200], 'colors': [[0.8,0.2,0.2], [0.2,0.8,0.2], [0.2,0.2,0.8]]},
            'metal': {'density_range': [2000, 8000], 'colors': [[0.7,0.7,0.7], [0.5,0.5,0.5], [0.3,0.3,0.3]]},
            'wood': {'density_range': [400, 800], 'colors': [[0.6,0.4,0.2], [0.8,0.6,0.3], [0.5,0.3,0.1]]},
            'ceramic': {'density_range': [1800, 2500], 'colors': [[0.9,0.9,0.9], [0.8,0.8,0.7], [0.7,0.7,0.6]]},
            'rubber': {'density_range': [900, 1400], 'colors': [[0.1,0.1,0.1], [0.3,0.3,0.3], [0.8,0.2,0.2]]}
        }
    
    def generate_object_variant(self, base_type: str, category: str, variation_level: float = 0.3) -> Dict:
        """Generate variant of base object with controlled randomization"""
        if category not in self.object_templates:
            raise ValueError(f"Unknown category: {category}")
        
        if base_type not in self.object_templates[category]:
            raise ValueError(f"Unknown object type: {base_type} in category {category}")
        
        base_config = self.object_templates[category][base_type]
        
        # Apply procedural variations
        variant = {
            'type': base_config['type'],
            'category': category,
            'base_type': base_type,
        }
        
        # Scale variation
        variant.update(self.vary_scale(base_config, variation_level))
        
        # Material assignment
        variant.update(self.select_realistic_material(category, base_type))
        
        # Physics properties
        variant.update(self.generate_physics_properties(variant))
        
        return variant
    
    def vary_scale(self, base_config: Dict, variation_level: float) -> Dict:
        """Apply scale variations"""
        base_scale = base_config['base_scale']
        scale_factor = 1.0 + random.uniform(-variation_level, variation_level)
        
        if base_config['type'] == 'sphere':
            radius = base_scale[0] / 2 * scale_factor
            return {'radius': radius}
        elif base_config['type'] == 'cylinder':
            radius = base_scale[0] / 2 * scale_factor
            height = base_scale[2] * scale_factor
            return {'radius': radius, 'height': height}
        else:  # cube
            scale = [dim * scale_factor for dim in base_scale]
            return {'scale': scale}
    
    def select_realistic_material(self, category: str, base_type: str) -> Dict:
        """Select realistic material based on object type"""
        material_mappings = {
            'household': {
                'cup': ['ceramic', 'plastic'],
                'bowl': ['ceramic', 'plastic', 'wood'],
                'plate': ['ceramic', 'plastic'],
                'bottle': ['plastic'],
                'can': ['metal']
            },
            'tools': {
                'screwdriver': ['metal', 'plastic'],
                'wrench': ['metal'],
                'hammer': ['metal', 'wood'],
                'pliers': ['metal']
            },
            'toys': {
                'block': ['wood', 'plastic'],
                'ball': ['rubber', 'plastic'],
                'car': ['plastic', 'metal']
            },
            'office': {
                'stapler': ['metal', 'plastic'],
                'pen': ['plastic', 'metal'],
                'book': ['paper'],  # Special case
                'folder': ['paper']  # Special case
            }
        }
        
        # Get possible materials for this object
        possible_materials = material_mappings.get(category, {}).get(base_type, ['plastic'])
        selected_material = random.choice(possible_materials)
        
        # Handle special cases
        if selected_material == 'paper':
            color = random.choice([[0.9,0.9,0.9], [0.8,0.8,0.7], [0.95,0.95,0.8]])
            return {'material': 'paper', 'color': color}
        
        # Get material properties
        material_props = self.material_properties.get(selected_material, self.material_properties['plastic'])
        color = random.choice(material_props['colors'])
        
        return {
            'material': selected_material,
            'color': color,
            'density_range': material_props['density_range']
        }
    
    def generate_physics_properties(self, variant: Dict) -> Dict:
        """Generate realistic physics properties"""
        # Calculate volume
        if variant['type'] == 'sphere':
            volume = (4/3) * np.pi * (variant['radius'] ** 3)
        elif variant['type'] == 'cylinder':
            volume = np.pi * (variant['radius'] ** 2) * variant['height']
        else:  # cube
            scale = variant['scale']
            volume = scale[0] * scale[1] * scale[2]
        
        # Calculate mass based on material density
        if 'density_range' in variant:
            density = random.uniform(*variant['density_range'])
        else:
            density = 1000  # Default density
        
        mass = volume * density
        
        return {
            'mass': mass,
            'volume': volume,
            'density': density
        }

class HierarchicalSceneBuilder(Node):
    """Hierarchical scene generation with intelligent placement"""
    
    def __init__(self):
        super().__init__('hierarchical_scene_builder')
        
        # Parameters
        self.declare_parameter('workspace_min', [-0.1, -0.3, 0.0])
        self.declare_parameter('workspace_max', [0.8, 0.3, 0.5])
        self.declare_parameter('default_complexity', 'MODERATE')
        self.declare_parameter('enable_clustering', True)
        self.declare_parameter('enable_stacking', True)
        
        workspace_min = self.get_parameter('workspace_min').get_parameter_value().double_array_value
        workspace_max = self.get_parameter('workspace_max').get_parameter_value().double_array_value
        complexity_str = self.get_parameter('default_complexity').get_parameter_value().string_value
        self.enable_clustering = self.get_parameter('enable_clustering').get_parameter_value().bool_value
        self.enable_stacking = self.get_parameter('enable_stacking').get_parameter_value().bool_value
        
        try:
            self.default_complexity = SceneComplexity[complexity_str]
        except KeyError:
            self.default_complexity = SceneComplexity.MODERATE
        
        # Create workspace bounds
        self.workspace_bounds = ObjectBounds(
            min_x=workspace_min[0], min_y=workspace_min[1], min_z=workspace_min[2],
            max_x=workspace_max[0], max_y=workspace_max[1], max_z=workspace_max[2]
        )
        
        # Initialize components
        self.object_placer = CollisionAwareObjectPlacer(self.workspace_bounds)
        self.stacking_validator = StackingValidator()
        self.object_library = ProceduralObjectLibrary()
        
        # Create scene templates
        self.scene_templates = self.create_scene_templates()
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/scene_builder/status', 10)
        self.scene_complete_pub = self.create_publisher(Bool, '/scene_builder/complete', 10)
        self.scene_stats_pub = self.create_publisher(String, '/scene_builder/stats', 10)
        
        # Subscribers
        self.build_scene_sub = self.create_subscription(
            String, '/scene_builder/build_scene', self.build_scene_callback, 10)
        self.set_complexity_sub = self.create_subscription(
            String, '/scene_builder/set_complexity', self.set_complexity_callback, 10)
        
        # Current scene state
        self.current_scene_type = None
        self.current_complexity = self.default_complexity
        
        self.get_logger().info('Hierarchical Scene Builder initialized')
        self.publish_status('Scene builder ready')
    
    def create_scene_templates(self) -> Dict[SceneType, SceneTemplate]:
        """Create predefined scene templates"""
        templates = {}
        
        # Kitchen counter scene
        templates[SceneType.KITCHEN_COUNTER] = SceneTemplate(
            name="Kitchen Counter",
            object_types=['household'],
            typical_arrangements=['clustered', 'stacked'],
            preferred_materials=['ceramic', 'plastic'],
            complexity_weights={
                SceneComplexity.SIMPLE: 0.3,
                SceneComplexity.MODERATE: 0.4,
                SceneComplexity.COMPLEX: 0.2,
                SceneComplexity.CLUTTERED: 0.1
            },
            stacking_probability=0.4,
            clustering_tendency=0.7
        )
        
        # Office desk scene
        templates[SceneType.OFFICE_DESK] = SceneTemplate(
            name="Office Desk",
            object_types=['office'],
            typical_arrangements=['organized', 'clustered'],
            preferred_materials=['plastic', 'metal', 'paper'],
            complexity_weights={
                SceneComplexity.SIMPLE: 0.2,
                SceneComplexity.MODERATE: 0.5,
                SceneComplexity.COMPLEX: 0.3,
                SceneComplexity.CLUTTERED: 0.0
            },
            stacking_probability=0.6,
            clustering_tendency=0.8
        )
        
        # Workshop table scene
        templates[SceneType.WORKSHOP_TABLE] = SceneTemplate(
            name="Workshop Table",
            object_types=['tools'],
            typical_arrangements=['scattered', 'clustered'],
            preferred_materials=['metal', 'plastic'],
            complexity_weights={
                SceneComplexity.SIMPLE: 0.1,
                SceneComplexity.MODERATE: 0.3,
                SceneComplexity.COMPLEX: 0.4,
                SceneComplexity.CLUTTERED: 0.2
            },
            stacking_probability=0.2,
            clustering_tendency=0.5
        )
        
        # Toy play area scene
        templates[SceneType.TOY_PLAYAREA] = SceneTemplate(
            name="Toy Play Area",
            object_types=['toys'],
            typical_arrangements=['scattered', 'stacked'],
            preferred_materials=['plastic', 'wood', 'rubber'],
            complexity_weights={
                SceneComplexity.SIMPLE: 0.2,
                SceneComplexity.MODERATE: 0.3,
                SceneComplexity.COMPLEX: 0.3,
                SceneComplexity.CLUTTERED: 0.2
            },
            stacking_probability=0.5,
            clustering_tendency=0.4
        )
        
        return templates
    
    def build_scene_callback(self, msg):
        """Handle scene building request"""
        scene_type_str = msg.data
        
        try:
            scene_type = SceneType(scene_type_str)
            self.build_scene(scene_type, self.current_complexity)
        except ValueError:
            self.get_logger().error(f'Invalid scene type: {scene_type_str}')
            self.publish_status(f'Invalid scene type: {scene_type_str}')
    
    def set_complexity_callback(self, msg):
        """Handle complexity setting"""
        complexity_str = msg.data
        
        try:
            self.current_complexity = SceneComplexity[complexity_str.upper()]
            self.publish_status(f'Complexity set to {self.current_complexity.name}')
        except KeyError:
            self.get_logger().error(f'Invalid complexity: {complexity_str}')
    
    def build_scene(self, scene_type: SceneType, complexity: SceneComplexity):
        """Build complete scene with specified type and complexity"""
        self.current_scene_type = scene_type
        template = self.scene_templates[scene_type]
        
        self.publish_status(f'Building {template.name} scene with {complexity.name} complexity')
        
        # Clear existing objects
        self.object_placer.clear_objects()
        
        # Determine number of objects based on complexity
        object_count = self.get_object_count_for_complexity(complexity)
        
        # Generate objects based on scene template
        scene_objects = self.generate_scene_objects(template, object_count)
        
        # Place objects using hierarchical strategy
        placed_objects = self.place_objects_hierarchically(scene_objects, template)
        
        # Generate scene statistics
        self.publish_scene_statistics(placed_objects, template)
        
        # Signal completion
        complete_msg = Bool()
        complete_msg.data = True
        self.scene_complete_pub.publish(complete_msg)
        
        self.publish_status(f'Scene generation complete: {len(placed_objects)} objects placed')
    
    def get_object_count_for_complexity(self, complexity: SceneComplexity) -> int:
        """Determine object count based on complexity level"""
        count_ranges = {
            SceneComplexity.SIMPLE: (3, 5),
            SceneComplexity.MODERATE: (6, 10),
            SceneComplexity.COMPLEX: (11, 15),
            SceneComplexity.CLUTTERED: (16, 20)
        }
        
        min_count, max_count = count_ranges[complexity]
        return random.randint(min_count, max_count)
    
    def generate_scene_objects(self, template: SceneTemplate, count: int) -> List[Dict]:
        """Generate objects for the scene based on template"""
        objects = []
        
        for i in range(count):
            # Select category based on template
            category = random.choice(template.object_types)
            
            # Select object type from category
            available_types = list(self.object_library.object_templates[category].keys())
            object_type = random.choice(available_types)
            
            # Generate variant
            variation_level = 0.2 + (0.3 * random.random())  # 0.2-0.5 variation
            object_config = self.object_library.generate_object_variant(
                object_type, category, variation_level)
            
            objects.append(object_config)
        
        return objects
    
    def place_objects_hierarchically(self, objects: List[Dict], template: SceneTemplate) -> List[PlacedObject]:
        """Place objects using hierarchical placement strategy"""
        placed_objects = []
        
        # Sort objects by size (largest first for better packing)
        objects.sort(key=lambda obj: self.get_object_volume(obj), reverse=True)
        
        # Phase 1: Place base layer objects
        base_objects = self.place_base_layer(objects[:len(objects)//2], template)
        placed_objects.extend(base_objects)
        
        # Phase 2: Add secondary objects with possible stacking
        remaining_objects = objects[len(objects)//2:]
        secondary_objects = self.place_secondary_layer(remaining_objects, template, base_objects)
        placed_objects.extend(secondary_objects)
        
        return placed_objects
    
    def place_base_layer(self, objects: List[Dict], template: SceneTemplate) -> List[PlacedObject]:
        """Place base layer objects"""
        placed_objects = []
        
        for i, obj_config in enumerate(objects):
            # Determine placement strategy
            if template.clustering_tendency > 0.5 and placed_objects:
                # Try clustered placement
                position = self.find_clustered_position(obj_config, placed_objects, template.clustering_tendency)
            else:
                # Random placement
                position = self.object_placer.find_valid_placement(obj_config)
            
            if position:
                object_id = f'{template.name.lower().replace(" ", "_")}_{i:03d}'
                placed_obj = self.object_placer.place_object(obj_config, object_id)
                if placed_obj:
                    placed_objects.append(placed_obj)
        
        return placed_objects
    
    def place_secondary_layer(self, objects: List[Dict], template: SceneTemplate, 
                            base_objects: List[PlacedObject]) -> List[PlacedObject]:
        """Place secondary layer with stacking opportunities"""
        placed_objects = []
        
        for i, obj_config in enumerate(objects):
            placed_obj = None
            object_id = f'{template.name.lower().replace(" ", "_")}_sec_{i:03d}'
            
            # Try stacking first if enabled
            if (self.enable_stacking and base_objects and 
                random.random() < template.stacking_probability):
                
                placed_obj = self.try_stacking_placement(obj_config, object_id, base_objects)
            
            # If stacking failed, try regular placement
            if placed_obj is None:
                if template.clustering_tendency > 0.5:
                    position = self.find_clustered_position(obj_config, 
                                                           base_objects + placed_objects, 
                                                           template.clustering_tendency)
                else:
                    position = self.object_placer.find_valid_placement(obj_config)
                
                if position:
                    placed_obj = self.object_placer.place_object(obj_config, object_id)
            
            if placed_obj:
                placed_objects.append(placed_obj)
        
        return placed_objects
    
    def find_clustered_position(self, obj_config: Dict, existing_objects: List[PlacedObject], 
                               clustering_tendency: float) -> Optional[Tuple[float, float, float]]:
        """Find position near existing objects for clustering"""
        if not existing_objects:
            return self.object_placer.find_valid_placement(obj_config)
        
        # Select cluster center (existing object)
        cluster_center = random.choice(existing_objects)
        
        # Generate position near cluster center
        cluster_radius = 0.1 * clustering_tendency  # Adjust based on tendency
        
        for attempt in range(20):
            # Random offset from cluster center
            angle = random.uniform(0, 2 * np.pi)
            distance = random.uniform(0.05, cluster_radius)
            
            offset_x = distance * np.cos(angle)
            offset_y = distance * np.sin(angle)
            
            candidate_pos = (
                cluster_center.position[0] + offset_x,
                cluster_center.position[1] + offset_y,
                cluster_center.position[2]
            )
            
            # Check if position is valid
            object_bounds = self.object_placer.create_object_bounds(candidate_pos, obj_config)
            if self.object_placer.is_valid_placement(object_bounds):
                return candidate_pos
        
        # Fall back to random placement
        return self.object_placer.find_valid_placement(obj_config)
    
    def try_stacking_placement(self, obj_config: Dict, object_id: str, 
                              base_objects: List[PlacedObject]) -> Optional[PlacedObject]:
        """Try to place object by stacking on existing objects"""
        # Find suitable base objects
        suitable_bases = []
        for base_obj in base_objects:
            if self.stacking_validator.can_stack_on(base_obj, obj_config):
                suitable_bases.append(base_obj)
        
        if not suitable_bases:
            return None
        
        # Try stacking on random suitable base
        for attempt in range(3):  # Multiple attempts
            base_obj = random.choice(suitable_bases)
            stack_position = self.stacking_validator.find_stacking_position(base_obj, obj_config)
            
            if stack_position:
                # Check for collisions
                object_bounds = self.object_placer.create_object_bounds(stack_position, obj_config)
                
                collision_detected = False
                for occupied_space in self.object_placer.occupied_spaces:
                    if object_bounds.overlaps_with(occupied_space, self.object_placer.collision_margin):
                        collision_detected = True
                        break
                
                if not collision_detected:
                    # Create stacked object
                    placed_object = PlacedObject(
                        id=object_id,
                        object_type=obj_config['type'],
                        position=stack_position,
                        rotation=obj_config.get('rotation', (0, 0, 0)),
                        bounds=object_bounds,
                        mass=obj_config.get('mass', 0.1),
                        support_area=self.object_placer.calculate_support_area(obj_config),
                        stacked_on=base_obj.id
                    )
                    
                    # Update tracking
                    base_obj.supports.append(object_id)
                    self.object_placer.placed_objects.append(placed_object)
                    self.object_placer.occupied_spaces.append(object_bounds)
                    
                    return placed_object
        
        return None
    
    def get_object_volume(self, obj_config: Dict) -> float:
        """Calculate object volume for sorting"""
        if obj_config['type'] == 'sphere':
            return (4/3) * np.pi * (obj_config['radius'] ** 3)
        elif obj_config['type'] == 'cylinder':
            return np.pi * (obj_config['radius'] ** 2) * obj_config['height']
        else:  # cube
            scale = obj_config['scale']
            return scale[0] * scale[1] * scale[2]
    
    def publish_scene_statistics(self, placed_objects: List[PlacedObject], template: SceneTemplate):
        """Publish statistics about the generated scene"""
        stats = {
            'scene_type': template.name,
            'total_objects': len(placed_objects),
            'stacked_objects': len([obj for obj in placed_objects if obj.stacked_on]),
            'workspace_density': self.object_placer.get_placement_density(),
            'object_types': {},
            'materials': {},
            'mass_distribution': []
        }
        
        # Analyze object composition
        for obj in placed_objects:
            obj_type = getattr(obj, 'base_type', obj.object_type)
            stats['object_types'][obj_type] = stats['object_types'].get(obj_type, 0) + 1
            stats['mass_distribution'].append(obj.mass)
        
        # Calculate statistics
        if stats['mass_distribution']:
            stats['average_mass'] = np.mean(stats['mass_distribution'])
            stats['total_mass'] = np.sum(stats['mass_distribution'])
        
        # Publish statistics
        stats_msg = String()
        stats_msg.data = json.dumps(stats, indent=2)
        self.scene_stats_pub.publish(stats_msg)
        
        self.get_logger().info(f'Scene statistics: {stats["total_objects"]} objects, '
                              f'{stats["stacked_objects"]} stacked, '
                              f'density: {stats["workspace_density"]:.2f}')
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[SCENE_BUILDER] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        builder = HierarchicalSceneBuilder()
        rclpy.spin(builder)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Scene builder error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()