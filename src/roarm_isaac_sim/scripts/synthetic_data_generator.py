#!/usr/bin/env python3
"""
Synthetic Data Generation Pipeline for RoArm M3

This script generates synthetic training data for AnyGrasp and other ML models
using Isaac Sim's domain randomization and photorealistic rendering capabilities.
"""

import os
import sys
import json
import random
import numpy as np
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseArray, Pose

# Isaac Sim imports
try:
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import get_current_stage
    from omni.isaac.core.utils.prims import create_prim, get_prim_at_path
    from omni.isaac.core.materials import PreviewSurface
    import omni.replicator.core as rep
    from pxr import UsdGeom, Gf, UsdShade
    ISAAC_AVAILABLE = True
except ImportError:
    ISAAC_AVAILABLE = False

class SyntheticDataGenerator(Node):
    def __init__(self):
        super().__init__('synthetic_data_generator')
        
        # Parameters
        self.declare_parameter('output_directory', '/tmp/synthetic_data')
        self.declare_parameter('num_scenes_per_batch', 50)
        self.declare_parameter('objects_per_scene_min', 3)
        self.declare_parameter('objects_per_scene_max', 8)
        self.declare_parameter('enable_domain_randomization', True)
        self.declare_parameter('generate_annotations', True)
        self.declare_parameter('image_resolution', [1280, 720])
        
        self.output_dir = Path(self.get_parameter('output_directory').get_parameter_value().string_value)
        self.scenes_per_batch = self.get_parameter('num_scenes_per_batch').get_parameter_value().integer_value
        self.objects_min = self.get_parameter('objects_per_scene_min').get_parameter_value().integer_value
        self.objects_max = self.get_parameter('objects_per_scene_max').get_parameter_value().integer_value
        self.domain_randomization = self.get_parameter('enable_domain_randomization').get_parameter_value().bool_value
        self.generate_annotations = self.get_parameter('generate_annotations').get_parameter_value().bool_value
        self.resolution = self.get_parameter('image_resolution').get_parameter_value().integer_array_value
        
        # Create output directories
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / 'images').mkdir(exist_ok=True)
        (self.output_dir / 'depth').mkdir(exist_ok=True)
        (self.output_dir / 'pointclouds').mkdir(exist_ok=True)
        (self.output_dir / 'annotations').mkdir(exist_ok=True)
        (self.output_dir / 'metadata').mkdir(exist_ok=True)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/synthetic_data/status', 10)
        self.progress_pub = self.create_publisher(Int32, '/synthetic_data/progress', 10)
        self.generated_scene_pub = self.create_publisher(Bool, '/synthetic_data/scene_ready', 10)
        
        # Subscribers  
        self.generate_batch_sub = self.create_subscription(
            Int32, '/synthetic_data/generate_batch', self.generate_batch_callback, 10)
        
        # Isaac Sim components
        self.world = None
        self.replicator_initialized = False
        self.object_library = []
        self.material_library = []
        self.lighting_configs = []
        
        # Generation state
        self.current_batch = 0
        self.scenes_generated = 0
        
        self.get_logger().info('Synthetic Data Generator initialized')
        
        if ISAAC_AVAILABLE:
            self.initialize_isaac_components()
        else:
            self.get_logger().error('Isaac Sim not available')
    
    def initialize_isaac_components(self):
        """Initialize Isaac Sim components for data generation"""
        try:
            # Get world instance
            self.world = World.instance()
            if not self.world:
                self.get_logger().error('Isaac Sim world not found')
                return
            
            # Initialize Replicator
            self.initialize_replicator()
            
            # Create object library
            self.create_object_library()
            
            # Create material library
            self.create_material_library()
            
            # Setup lighting configurations
            self.setup_lighting_configs()
            
            self.publish_status('Isaac Sim components initialized')
            self.get_logger().info('Isaac Sim initialization complete')
            
        except Exception as e:
            self.get_logger().error(f'Isaac Sim initialization failed: {e}')
    
    def initialize_replicator(self):
        """Initialize Omniverse Replicator for domain randomization"""
        try:
            # Create replicator cameras
            self.setup_replicator_cameras()
            
            # Setup domain randomization
            if self.domain_randomization:
                self.setup_domain_randomization()
            
            self.replicator_initialized = True
            self.get_logger().info('Replicator initialized')
            
        except Exception as e:
            self.get_logger().error(f'Replicator initialization failed: {e}')
    
    def setup_replicator_cameras(self):
        """Setup cameras for synthetic data capture"""
        try:
            # Main data collection camera (overhead view)
            self.main_camera = rep.create.camera(
                position=(0.0, 0.0, 0.8),
                rotation=(-90, 0, 0),
                resolution=self.resolution
            )
            
            # Side view camera
            self.side_camera = rep.create.camera(
                position=(0.6, 0.3, 0.3),
                rotation=(-30, -30, 0),
                resolution=self.resolution
            )
            
            # Wrist-mounted camera (robot perspective)
            self.wrist_camera = rep.create.camera(
                position=(0.05, 0.0, 0.15),
                rotation=(-90, 0, 0),
                resolution=[640, 480]
            )
            
            self.get_logger().info('Replicator cameras setup complete')
            
        except Exception as e:
            self.get_logger().error(f'Camera setup failed: {e}')
    
    def setup_domain_randomization(self):
        """Setup domain randomization parameters"""
        try:
            # Lighting randomization
            self.light_randomizer = rep.randomizer.light(
                rotation=rep.distribution.uniform((-180, -180, -180), (180, 180, 180)),
                intensity=rep.distribution.uniform(500, 2000),
                temperature=rep.distribution.uniform(3000, 6500)
            )
            
            # Texture randomization  
            self.texture_randomizer = rep.randomizer.materials(
                materials=self.material_library,
                input_prims=rep.utils.get_bounds('/World/Objects'),
            )
            
            # Camera pose randomization
            self.camera_randomizer = rep.randomizer.pose(
                position=rep.distribution.uniform((-0.2, -0.2, 0.5), (0.2, 0.2, 1.0)),
                rotation=rep.distribution.uniform((-30, -30, -30), (30, 30, 30))
            )
            
            self.get_logger().info('Domain randomization setup complete')
            
        except Exception as e:
            self.get_logger().error(f'Domain randomization setup failed: {e}')
    
    def create_object_library(self):
        """Create library of objects for scene generation"""
        try:
            # Basic geometric objects
            basic_objects = [
                {'type': 'cube', 'scale': [0.03, 0.03, 0.03]},
                {'type': 'sphere', 'scale': [0.025, 0.025, 0.025]},
                {'type': 'cylinder', 'scale': [0.02, 0.02, 0.04]},
                {'type': 'cone', 'scale': [0.025, 0.025, 0.04]},
            ]
            
            # Common manipulation objects
            manipulation_objects = [
                {'type': 'bottle', 'scale': [0.03, 0.03, 0.08]},
                {'type': 'can', 'scale': [0.032, 0.032, 0.06]},
                {'type': 'box', 'scale': [0.04, 0.03, 0.02]},
                {'type': 'tool', 'scale': [0.02, 0.01, 0.12]},
            ]
            
            self.object_library = basic_objects + manipulation_objects
            self.get_logger().info(f'Object library created with {len(self.object_library)} objects')
            
        except Exception as e:
            self.get_logger().error(f'Object library creation failed: {e}')
    
    def create_material_library(self):
        """Create library of materials for domain randomization"""
        try:
            # Create diverse materials
            material_configs = [
                {'name': 'plastic_red', 'diffuse': [0.8, 0.2, 0.2], 'roughness': 0.3},
                {'name': 'metal_blue', 'diffuse': [0.2, 0.2, 0.8], 'metallic': 0.8, 'roughness': 0.1},
                {'name': 'wood_brown', 'diffuse': [0.6, 0.4, 0.2], 'roughness': 0.7},
                {'name': 'ceramic_white', 'diffuse': [0.9, 0.9, 0.9], 'roughness': 0.2},
                {'name': 'rubber_black', 'diffuse': [0.1, 0.1, 0.1], 'roughness': 0.9},
                {'name': 'glass_clear', 'diffuse': [0.9, 0.9, 0.9], 'transmission': 0.8},
            ]
            
            for config in material_configs:
                material = PreviewSurface(
                    prim_path=f"/World/Materials/{config['name']}",
                    name=config['name']
                )
                material.set_color(config['diffuse'])
                material.set_roughness(config.get('roughness', 0.5))
                material.set_metallic(config.get('metallic', 0.0))
                
                self.material_library.append(material)
            
            self.get_logger().info(f'Material library created with {len(self.material_library)} materials')
            
        except Exception as e:
            self.get_logger().error(f'Material library creation failed: {e}')
    
    def setup_lighting_configs(self):
        """Setup different lighting configurations"""
        self.lighting_configs = [
            {'type': 'studio', 'intensity': 1000, 'temperature': 5500},
            {'type': 'indoor', 'intensity': 500, 'temperature': 3200},
            {'type': 'outdoor', 'intensity': 2000, 'temperature': 6500},
            {'type': 'harsh', 'intensity': 1500, 'temperature': 4000},
            {'type': 'soft', 'intensity': 300, 'temperature': 5000},
        ]
    
    def generate_batch_callback(self, msg):
        """Handle batch generation request"""
        batch_size = msg.data if msg.data > 0 else self.scenes_per_batch
        self.generate_data_batch(batch_size)
    
    def generate_data_batch(self, batch_size):
        """Generate a batch of synthetic training data"""
        if not self.replicator_initialized:
            self.publish_status('Replicator not initialized')
            return
        
        self.current_batch += 1
        self.publish_status(f'Starting batch {self.current_batch} with {batch_size} scenes')
        
        batch_start_time = datetime.now()
        
        for scene_idx in range(batch_size):
            try:
                # Generate scene
                scene_data = self.generate_single_scene(scene_idx)
                
                # Save data
                if scene_data:
                    self.save_scene_data(scene_data, self.current_batch, scene_idx)
                    self.scenes_generated += 1
                
                # Publish progress
                progress_msg = Int32()
                progress_msg.data = scene_idx + 1
                self.progress_pub.publish(progress_msg)
                
                # Publish scene ready signal
                ready_msg = Bool()
                ready_msg.data = True
                self.generated_scene_pub.publish(ready_msg)
                
            except Exception as e:
                self.get_logger().error(f'Scene {scene_idx} generation failed: {e}')
        
        batch_duration = datetime.now() - batch_start_time
        self.publish_status(f'Batch {self.current_batch} completed in {batch_duration.total_seconds():.1f}s')
    
    def generate_single_scene(self, scene_idx):
        """Generate a single synthetic scene"""
        try:
            # Clear previous objects
            self.clear_scene_objects()
            
            # Randomize scene parameters
            num_objects = random.randint(self.objects_min, self.objects_max)
            lighting_config = random.choice(self.lighting_configs)
            
            # Setup lighting
            self.setup_scene_lighting(lighting_config)
            
            # Generate objects
            objects_data = []
            for obj_idx in range(num_objects):
                obj_data = self.generate_random_object(obj_idx)
                if obj_data:
                    objects_data.append(obj_data)
            
            # Apply domain randomization
            if self.domain_randomization:
                self.apply_domain_randomization()
            
            # Step simulation to settle physics
            for _ in range(10):
                self.world.step(render=True)
            
            # Capture images and annotations
            scene_data = {
                'scene_id': f'batch_{self.current_batch:04d}_scene_{scene_idx:04d}',
                'timestamp': datetime.now().isoformat(),
                'objects': objects_data,
                'lighting': lighting_config,
                'images': self.capture_scene_images(),
                'annotations': self.generate_scene_annotations(objects_data) if self.generate_annotations else None
            }
            
            return scene_data
            
        except Exception as e:
            self.get_logger().error(f'Single scene generation failed: {e}')
            return None
    
    def clear_scene_objects(self):
        """Clear objects from previous scene"""
        try:
            stage = get_current_stage()
            objects_prim = stage.GetPrimAtPath('/World/Objects')
            if objects_prim:
                stage.RemovePrim('/World/Objects')
            
            # Recreate objects container
            create_prim('/World/Objects', 'Xform')
            
        except Exception as e:
            self.get_logger().debug(f'Scene clearing failed: {e}')
    
    def setup_scene_lighting(self, lighting_config):
        """Setup lighting for the scene"""
        try:
            # Implementation would setup lights based on config
            self.get_logger().debug(f'Setting up {lighting_config["type"]} lighting')
            
        except Exception as e:
            self.get_logger().debug(f'Lighting setup failed: {e}')
    
    def generate_random_object(self, obj_idx):
        """Generate a random object in the scene"""
        try:
            # Select random object type
            obj_config = random.choice(self.object_library)
            
            # Random position on table
            x = random.uniform(0.15, 0.45)
            y = random.uniform(-0.2, 0.2)
            z = 0.05  # On table surface
            
            # Random orientation
            rotation = [0, 0, random.uniform(0, 360)]
            
            # Create object
            obj_path = f'/World/Objects/Object_{obj_idx}'
            obj_prim = create_prim(obj_path, obj_config['type'].title())
            
            if obj_prim:
                obj_geom = UsdGeom.Cube(obj_prim)  # Simplified - should match obj type
                obj_geom.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
                obj_geom.AddRotateXYZOp().Set(Gf.Vec3d(*rotation))
                obj_geom.AddScaleOp().Set(Gf.Vec3d(*obj_config['scale']))
                
                # Apply random material
                if self.material_library:
                    material = random.choice(self.material_library)
                    # Apply material to object (simplified)
                
                return {
                    'id': obj_idx,
                    'type': obj_config['type'],
                    'position': [x, y, z],
                    'rotation': rotation,
                    'scale': obj_config['scale']
                }
            
        except Exception as e:
            self.get_logger().debug(f'Object generation failed: {e}')
        
        return None
    
    def apply_domain_randomization(self):
        """Apply domain randomization to the scene"""
        try:
            # Trigger replicator randomizers
            if hasattr(self, 'light_randomizer'):
                # Apply randomization (simplified - actual implementation would vary)
                pass
                
        except Exception as e:
            self.get_logger().debug(f'Domain randomization failed: {e}')
    
    def capture_scene_images(self):
        """Capture images from all cameras"""
        images = {}
        
        try:
            # Capture from main camera
            if hasattr(self, 'main_camera'):
                rgb_data = rep.utils.capture_image(self.main_camera)
                depth_data = rep.utils.capture_depth(self.main_camera)
                
                images['main'] = {
                    'rgb': rgb_data,
                    'depth': depth_data,
                    'camera_params': self.get_camera_params('main')
                }
            
            # Capture from side camera
            if hasattr(self, 'side_camera'):
                rgb_data = rep.utils.capture_image(self.side_camera)
                depth_data = rep.utils.capture_depth(self.side_camera)
                
                images['side'] = {
                    'rgb': rgb_data,
                    'depth': depth_data,
                    'camera_params': self.get_camera_params('side')
                }
            
            # Capture from wrist camera
            if hasattr(self, 'wrist_camera'):
                rgb_data = rep.utils.capture_image(self.wrist_camera)
                depth_data = rep.utils.capture_depth(self.wrist_camera)
                
                images['wrist'] = {
                    'rgb': rgb_data,
                    'depth': depth_data,
                    'camera_params': self.get_camera_params('wrist')
                }
            
        except Exception as e:
            self.get_logger().error(f'Image capture failed: {e}')
        
        return images
    
    def get_camera_params(self, camera_name):
        """Get camera parameters for calibration"""
        # Return camera intrinsics and extrinsics
        return {
            'intrinsics': {
                'fx': 615.0, 'fy': 615.0,
                'cx': 320.0, 'cy': 240.0
            },
            'extrinsics': {
                'position': [0, 0, 0],
                'rotation': [0, 0, 0]
            }
        }
    
    def generate_scene_annotations(self, objects_data):
        """Generate annotations for the scene"""
        annotations = {
            'objects': [],
            'grasps': []
        }
        
        try:
            # Generate object annotations
            for obj in objects_data:
                obj_annotation = {
                    'id': obj['id'],
                    'type': obj['type'],
                    'bbox_3d': self.calculate_3d_bbox(obj),
                    'bbox_2d': self.project_to_2d_bbox(obj),
                    'pose': {
                        'position': obj['position'],
                        'rotation': obj['rotation']
                    }
                }
                annotations['objects'].append(obj_annotation)
                
                # Generate grasp annotations for this object
                grasp_poses = self.generate_grasp_poses(obj)
                annotations['grasps'].extend(grasp_poses)
            
        except Exception as e:
            self.get_logger().error(f'Annotation generation failed: {e}')
        
        return annotations
    
    def calculate_3d_bbox(self, obj_data):
        """Calculate 3D bounding box for object"""
        # Simplified implementation
        pos = obj_data['position']
        scale = obj_data['scale']
        
        return {
            'min': [pos[0] - scale[0]/2, pos[1] - scale[1]/2, pos[2] - scale[2]/2],
            'max': [pos[0] + scale[0]/2, pos[1] + scale[1]/2, pos[2] + scale[2]/2]
        }
    
    def project_to_2d_bbox(self, obj_data):
        """Project 3D object to 2D bounding box"""
        # Simplified projection - would need actual camera parameters
        return {
            'x': 320, 'y': 240, 'width': 50, 'height': 50
        }
    
    def generate_grasp_poses(self, obj_data):
        """Generate grasp poses for an object"""
        grasps = []
        
        # Generate multiple grasp candidates per object
        for i in range(3):  # 3 grasps per object
            grasp = {
                'object_id': obj_data['id'],
                'position': [
                    obj_data['position'][0] + random.uniform(-0.01, 0.01),
                    obj_data['position'][1] + random.uniform(-0.01, 0.01),
                    obj_data['position'][2] + 0.02  # Slightly above object
                ],
                'orientation': [0, 0, 0, 1],  # Simplified orientation
                'width': random.uniform(0.02, 0.08),
                'confidence': random.uniform(0.6, 0.95)
            }
            grasps.append(grasp)
        
        return grasps
    
    def save_scene_data(self, scene_data, batch_id, scene_id):
        """Save generated scene data to disk"""
        try:
            scene_dir = self.output_dir / f'batch_{batch_id:04d}' / f'scene_{scene_id:04d}'
            scene_dir.mkdir(parents=True, exist_ok=True)
            
            # Save images
            if 'images' in scene_data:
                for cam_name, cam_data in scene_data['images'].items():
                    if 'rgb' in cam_data:
                        rgb_path = scene_dir / f'{cam_name}_rgb.png'
                        # Save RGB image (implementation depends on data format)
                    
                    if 'depth' in cam_data:
                        depth_path = scene_dir / f'{cam_name}_depth.npy'
                        # Save depth data
                        np.save(depth_path, cam_data['depth'])
            
            # Save annotations
            if scene_data.get('annotations'):
                annotation_path = scene_dir / 'annotations.json'
                with open(annotation_path, 'w') as f:
                    json.dump(scene_data['annotations'], f, indent=2)
            
            # Save metadata
            metadata_path = scene_dir / 'metadata.json'
            metadata = {
                'scene_id': scene_data['scene_id'],
                'timestamp': scene_data['timestamp'],
                'objects': scene_data['objects'],
                'lighting': scene_data['lighting']
            }
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            self.get_logger().debug(f'Scene data saved to {scene_dir}')
            
        except Exception as e:
            self.get_logger().error(f'Scene data saving failed: {e}')
    
    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = f"[SYNTHETIC_DATA] {message}"
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        generator = SyntheticDataGenerator()
        rclpy.spin(generator)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Generator error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()