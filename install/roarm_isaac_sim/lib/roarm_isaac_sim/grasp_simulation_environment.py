#!/usr/bin/env python3
"""
Grasp Simulation Environment for RoArm M3

This script creates and manages specialized simulation environments for testing
and validating grasp algorithms in Isaac Sim.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import random
import time
from enum import Enum

# ROS2 message types
from std_msgs.msg import String, Bool, Int32, Float32
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import PointCloud2, Image
from roarm_anygrasp_integration.srv import GetGraspCandidates, SelectGrasp

# Isaac Sim imports (conditional)
try:
    from omni.isaac.core import World
    from omni.isaac.core.utils.stage import get_current_stage
    from omni.isaac.core.utils.prims import create_prim
    from omni.isaac.core.objects import DynamicCuboid, DynamicSphere, DynamicCylinder
    from pxr import UsdGeom, Gf
    ISAAC_AVAILABLE = True
except ImportError:
    ISAAC_AVAILABLE = False

class GraspTestType(Enum):
    SINGLE_OBJECT = "single_object"
    MULTI_OBJECT = "multi_object"
    CLUTTERED_SCENE = "cluttered_scene"
    MOVING_OBJECT = "moving_object"
    PRECISION_GRASP = "precision_grasp"

class GraspSimulationEnvironment(Node):
    def __init__(self):
        super().__init__('grasp_simulation_environment')
        
        # Parameters
        self.declare_parameter('auto_test_mode', False)
        self.declare_parameter('test_duration', 60.0)  # seconds
        self.declare_parameter('success_threshold', 0.7)
        self.declare_parameter('num_test_objects', 10)
        
        self.auto_test = self.get_parameter('auto_test_mode').get_parameter_value().bool_value
        self.test_duration = self.get_parameter('test_duration').get_parameter_value().double_value
        self.success_threshold = self.get_parameter('success_threshold').get_parameter_value().double_value
        self.num_test_objects = self.get_parameter('num_test_objects').get_parameter_value().integer_value
        
        # Publishers
        self.environment_status_pub = self.create_publisher(String, '/grasp_sim_env/status', 10)
        self.test_results_pub = self.create_publisher(String, '/grasp_sim_env/test_results', 10)
        self.scene_ready_pub = self.create_publisher(Bool, '/grasp_sim_env/scene_ready', 10)
        self.success_rate_pub = self.create_publisher(Float32, '/grasp_sim_env/success_rate', 10)
        
        # Subscribers
        self.start_test_sub = self.create_subscription(
            String, '/grasp_sim_env/start_test', self.start_test_callback, 10)
        self.reset_environment_sub = self.create_subscription(
            Bool, '/grasp_sim_env/reset', self.reset_environment_callback, 10)
        
        # Service clients
        self.grasp_candidates_client = self.create_client(GetGraspCandidates, '/anygrasp/get_candidates')
        self.execute_grasp_client = self.create_client(SelectGrasp, '/anygrasp/select_grasp')
        
        # Isaac Sim components
        self.world = None
        self.test_objects = []
        self.current_test_type = None
        self.test_active = False
        
        # Test statistics
        self.test_stats = {
            'total_grasps': 0,
            'successful_grasps': 0,
            'failed_grasps': 0,
            'test_start_time': None,
            'current_test_type': None
        }
        
        self.get_logger().info('Grasp Simulation Environment initialized')
        
        if ISAAC_AVAILABLE:
            self.initialize_isaac_environment()
        else:
            self.get_logger().error('Isaac Sim not available')
            
        # Start auto test if enabled
        if self.auto_test:
            self.create_timer(5.0, self.start_auto_test)
    
    def initialize_isaac_environment(self):
        """Initialize Isaac Sim environment for grasp testing"""
        try:
            # Get world instance
            self.world = World.instance()
            if not self.world:
                self.get_logger().error('Isaac Sim world not found')
                return
            
            # Create basic environment
            self.create_base_environment()
            
            self.publish_status('Isaac Sim grasp environment initialized')
            self.get_logger().info('Grasp simulation environment ready')
            
        except Exception as e:
            self.get_logger().error(f'Isaac Sim environment initialization failed: {e}')
    
    def create_base_environment(self):
        """Create base manipulation environment"""
        try:
            # Clear existing test objects
            self.clear_test_objects()
            
            # Create workspace table
            table = DynamicCuboid(
                prim_path="/World/Table",
                name="workspace_table",
                position=np.array([0.4, 0.0, -0.01]),
                scale=np.array([0.8, 0.6, 0.02]),
                color=np.array([0.7, 0.7, 0.7])
            )
            self.world.scene.add(table)
            
            # Create boundaries
            self.create_workspace_boundaries()
            
            self.get_logger().info('Base environment created')
            
        except Exception as e:
            self.get_logger().error(f'Base environment creation failed: {e}')
    
    def create_workspace_boundaries(self):
        """Create workspace boundaries for safety"""
        try:
            boundary_configs = [
                {'name': 'left_wall', 'position': [0.1, 0.35, 0.1], 'scale': [0.02, 0.1, 0.2]},
                {'name': 'right_wall', 'position': [0.1, -0.35, 0.1], 'scale': [0.02, 0.1, 0.2]},
                {'name': 'back_wall', 'position': [0.8, 0.0, 0.1], 'scale': [0.02, 0.7, 0.2]},
            ]
            
            for config in boundary_configs:
                boundary = DynamicCuboid(
                    prim_path=f"/World/Boundaries/{config['name']}",
                    name=config['name'],
                    position=np.array(config['position']),
                    scale=np.array(config['scale']),
                    color=np.array([0.3, 0.3, 0.3])
                )
                self.world.scene.add(boundary)
            
        except Exception as e:
            self.get_logger().error(f'Boundary creation failed: {e}')
    
    def start_test_callback(self, msg):
        """Handle test start request"""
        test_type_str = msg.data
        
        try:
            test_type = GraspTestType(test_type_str)
            self.start_grasp_test(test_type)
        except ValueError:
            self.get_logger().error(f'Invalid test type: {test_type_str}')
            self.publish_status(f'Invalid test type: {test_type_str}')
    
    def reset_environment_callback(self, msg):
        """Handle environment reset request"""
        if msg.data:
            self.reset_environment()
    
    def start_auto_test(self):
        """Start automatic testing sequence"""
        if self.test_active:
            return
            
        self.publish_status('Starting automatic grasp testing')
        
        # Run different test types in sequence
        test_sequence = [
            GraspTestType.SINGLE_OBJECT,
            GraspTestType.MULTI_OBJECT,
            GraspTestType.CLUTTERED_SCENE,
            GraspTestType.PRECISION_GRASP
        ]
        
        for test_type in test_sequence:
            self.start_grasp_test(test_type)
            time.sleep(self.test_duration / len(test_sequence))
    
    def start_grasp_test(self, test_type):
        """Start specific grasp test"""
        if self.test_active:
            self.publish_status('Test already active')
            return
        
        self.test_active = True
        self.current_test_type = test_type
        
        # Reset statistics
        self.test_stats = {
            'total_grasps': 0,
            'successful_grasps': 0,
            'failed_grasps': 0,
            'test_start_time': time.time(),
            'current_test_type': test_type.value
        }
        
        self.publish_status(f'Starting {test_type.value} test')
        
        # Create test scene
        self.create_test_scene(test_type)
        
        # Start test execution
        self.execute_test_sequence(test_type)
    
    def create_test_scene(self, test_type):
        """Create scene for specific test type"""
        self.clear_test_objects()
        
        if test_type == GraspTestType.SINGLE_OBJECT:
            self.create_single_object_scene()
        elif test_type == GraspTestType.MULTI_OBJECT:
            self.create_multi_object_scene()
        elif test_type == GraspTestType.CLUTTERED_SCENE:
            self.create_cluttered_scene()
        elif test_type == GraspTestType.MOVING_OBJECT:
            self.create_moving_object_scene()
        elif test_type == GraspTestType.PRECISION_GRASP:
            self.create_precision_grasp_scene()
        
        # Allow physics to settle
        for _ in range(10):
            self.world.step(render=True)
        
        # Signal scene is ready
        scene_ready_msg = Bool()
        scene_ready_msg.data = True
        self.scene_ready_pub.publish(scene_ready_msg)
    
    def create_single_object_scene(self):
        """Create scene with single object for basic grasp testing"""
        try:
            # Create single test object
            object_types = ['cube', 'sphere', 'cylinder']
            object_type = random.choice(object_types)
            
            position = np.array([0.3, 0.0, 0.05])
            
            if object_type == 'cube':
                test_obj = DynamicCuboid(
                    prim_path="/World/TestObjects/SingleObject",
                    name="single_test_object",
                    position=position,
                    scale=np.array([0.04, 0.04, 0.04]),
                    color=np.array([0.8, 0.2, 0.2])
                )
            elif object_type == 'sphere':
                test_obj = DynamicSphere(
                    prim_path="/World/TestObjects/SingleObject",
                    name="single_test_object",
                    position=position,
                    radius=0.025,
                    color=np.array([0.2, 0.8, 0.2])
                )
            else:  # cylinder
                test_obj = DynamicCylinder(
                    prim_path="/World/TestObjects/SingleObject",
                    name="single_test_object",
                    position=position,
                    radius=0.02,
                    height=0.06,
                    color=np.array([0.2, 0.2, 0.8])
                )
            
            self.world.scene.add(test_obj)
            self.test_objects.append(test_obj)
            
            self.get_logger().info(f'Created single {object_type} object scene')
            
        except Exception as e:
            self.get_logger().error(f'Single object scene creation failed: {e}')
    
    def create_multi_object_scene(self):
        """Create scene with multiple objects"""
        try:
            object_positions = [
                [0.25, 0.1, 0.05],
                [0.35, -0.05, 0.05],
                [0.3, 0.15, 0.05],
                [0.4, 0.0, 0.05]
            ]
            
            object_types = ['cube', 'sphere', 'cylinder', 'cube']
            colors = [
                [0.8, 0.2, 0.2],
                [0.2, 0.8, 0.2], 
                [0.2, 0.2, 0.8],
                [0.8, 0.8, 0.2]
            ]
            
            for i, (pos, obj_type, color) in enumerate(zip(object_positions, object_types, colors)):
                prim_path = f"/World/TestObjects/Object_{i}"
                
                if obj_type == 'cube':
                    test_obj = DynamicCuboid(
                        prim_path=prim_path,
                        name=f"test_object_{i}",
                        position=np.array(pos),
                        scale=np.array([0.03, 0.03, 0.03]),
                        color=np.array(color)
                    )
                elif obj_type == 'sphere':
                    test_obj = DynamicSphere(
                        prim_path=prim_path,
                        name=f"test_object_{i}",
                        position=np.array(pos),
                        radius=0.02,
                        color=np.array(color)
                    )
                else:  # cylinder
                    test_obj = DynamicCylinder(
                        prim_path=prim_path,
                        name=f"test_object_{i}",
                        position=np.array(pos),
                        radius=0.015,
                        height=0.04,
                        color=np.array(color)
                    )
                
                self.world.scene.add(test_obj)
                self.test_objects.append(test_obj)
            
            self.get_logger().info(f'Created multi-object scene with {len(self.test_objects)} objects')
            
        except Exception as e:
            self.get_logger().error(f'Multi-object scene creation failed: {e}')
    
    def create_cluttered_scene(self):
        """Create cluttered scene for challenging grasp testing"""
        try:
            # Create many small objects in close proximity
            num_objects = random.randint(8, 12)
            
            for i in range(num_objects):
                # Random position in workspace
                x = random.uniform(0.2, 0.45)
                y = random.uniform(-0.15, 0.15)
                z = 0.03
                
                # Random object type
                obj_type = random.choice(['cube', 'sphere', 'cylinder'])
                color = np.array([random.random(), random.random(), random.random()])
                
                prim_path = f"/World/TestObjects/ClutteredObject_{i}"
                
                if obj_type == 'cube':
                    size = random.uniform(0.02, 0.035)
                    test_obj = DynamicCuboid(
                        prim_path=prim_path,
                        name=f"cluttered_object_{i}",
                        position=np.array([x, y, z]),
                        scale=np.array([size, size, size]),
                        color=color
                    )
                elif obj_type == 'sphere':
                    radius = random.uniform(0.015, 0.025)
                    test_obj = DynamicSphere(
                        prim_path=prim_path,
                        name=f"cluttered_object_{i}",
                        position=np.array([x, y, z]),
                        radius=radius,
                        color=color
                    )
                else:  # cylinder
                    radius = random.uniform(0.01, 0.02)
                    height = random.uniform(0.03, 0.05)
                    test_obj = DynamicCylinder(
                        prim_path=prim_path,
                        name=f"cluttered_object_{i}",
                        position=np.array([x, y, z]),
                        radius=radius,
                        height=height,
                        color=color
                    )
                
                self.world.scene.add(test_obj)
                self.test_objects.append(test_obj)
            
            self.get_logger().info(f'Created cluttered scene with {len(self.test_objects)} objects')
            
        except Exception as e:
            self.get_logger().error(f'Cluttered scene creation failed: {e}')
    
    def create_moving_object_scene(self):
        """Create scene with moving objects"""
        # For now, create static objects - motion would require more complex physics
        self.create_single_object_scene()
        self.get_logger().info('Created moving object scene (static for now)')
    
    def create_precision_grasp_scene(self):
        """Create scene requiring precision grasping"""
        try:
            # Create small, delicate objects
            precision_objects = [
                {'type': 'cube', 'scale': [0.015, 0.015, 0.015], 'pos': [0.28, 0.05, 0.04]},
                {'type': 'cylinder', 'radius': 0.008, 'height': 0.03, 'pos': [0.32, -0.03, 0.04]},
                {'type': 'sphere', 'radius': 0.012, 'pos': [0.35, 0.08, 0.04]}
            ]
            
            for i, obj_config in enumerate(precision_objects):
                prim_path = f"/World/TestObjects/PrecisionObject_{i}"
                color = np.array([0.9, 0.9, 0.1])  # Yellow for precision objects
                
                if obj_config['type'] == 'cube':
                    test_obj = DynamicCuboid(
                        prim_path=prim_path,
                        name=f"precision_object_{i}",
                        position=np.array(obj_config['pos']),
                        scale=np.array(obj_config['scale']),
                        color=color
                    )
                elif obj_config['type'] == 'sphere':
                    test_obj = DynamicSphere(
                        prim_path=prim_path,
                        name=f"precision_object_{i}",
                        position=np.array(obj_config['pos']),
                        radius=obj_config['radius'],
                        color=color
                    )
                else:  # cylinder
                    test_obj = DynamicCylinder(
                        prim_path=prim_path,
                        name=f"precision_object_{i}",
                        position=np.array(obj_config['pos']),
                        radius=obj_config['radius'],
                        height=obj_config['height'],
                        color=color
                    )
                
                self.world.scene.add(test_obj)
                self.test_objects.append(test_obj)
            
            self.get_logger().info(f'Created precision grasp scene with {len(self.test_objects)} objects')
            
        except Exception as e:
            self.get_logger().error(f'Precision grasp scene creation failed: {e}')
    
    def execute_test_sequence(self, test_type):
        """Execute automated test sequence"""
        self.publish_status(f'Executing {test_type.value} test sequence')
        
        # Run test for specified duration or number of attempts
        test_attempts = 5 if test_type == GraspTestType.PRECISION_GRASP else 10
        
        for attempt in range(test_attempts):
            if not self.test_active:
                break
                
            self.publish_status(f'Test attempt {attempt + 1}/{test_attempts}')
            
            # Run single grasp test
            success = self.execute_single_grasp_test()
            
            # Update statistics
            self.test_stats['total_grasps'] += 1
            if success:
                self.test_stats['successful_grasps'] += 1
            else:
                self.test_stats['failed_grasps'] += 1
            
            # Publish current success rate
            success_rate = self.test_stats['successful_grasps'] / self.test_stats['total_grasps']
            success_rate_msg = Float32()
            success_rate_msg.data = success_rate
            self.success_rate_pub.publish(success_rate_msg)
            
            # Small delay between attempts
            time.sleep(2.0)
        
        # Complete test
        self.complete_test()
    
    def execute_single_grasp_test(self):
        """Execute a single grasp test and return success status"""
        try:
            # Request grasp candidates
            if not self.grasp_candidates_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('Grasp candidates service not available')
                return False
            
            request = GetGraspCandidates.Request()
            request.num_candidates = 3
            request.min_confidence = 0.5
            
            future = self.grasp_candidates_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is None:
                self.get_logger().error('Failed to get grasp candidates')
                return False
            
            candidates_response = future.result()
            
            if len(candidates_response.grasp_poses) == 0:
                self.get_logger().warning('No grasp candidates found')
                return False
            
            # Select best candidate (first one - highest confidence)
            if not self.execute_grasp_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('Execute grasp service not available')
                return False
            
            execute_request = SelectGrasp.Request()
            execute_request.selected_grasp_index = 0
            
            execute_future = self.execute_grasp_client.call_async(execute_request)
            rclpy.spin_until_future_complete(self, execute_future, timeout_sec=30.0)
            
            if execute_future.result() is None:
                self.get_logger().error('Grasp execution failed')
                return False
            
            execute_response = execute_future.result()
            
            # Evaluate grasp success
            success = self.evaluate_grasp_success(execute_response)
            
            return success
            
        except Exception as e:
            self.get_logger().error(f'Single grasp test failed: {e}')
            return False
    
    def evaluate_grasp_success(self, execute_response):
        """Evaluate if grasp was successful"""
        # Basic success evaluation based on response
        if not execute_response.success:
            return False
        
        # Additional success criteria could include:
        # - Object lifted to minimum height
        # - Object held for minimum duration
        # - No collisions during execution
        
        # For simulation, we'll use a probabilistic model
        confidence = execute_response.confidence_score if hasattr(execute_response, 'confidence_score') else 0.7
        
        # Higher confidence grasps are more likely to succeed
        success_probability = min(confidence * 1.2, 0.95)  # Cap at 95%
        
        return random.random() < success_probability
    
    def complete_test(self):
        """Complete current test and publish results"""
        if not self.test_active:
            return
        
        self.test_active = False
        
        # Calculate final statistics
        total_grasps = self.test_stats['total_grasps']
        successful_grasps = self.test_stats['successful_grasps']
        
        if total_grasps > 0:
            success_rate = successful_grasps / total_grasps
            test_duration = time.time() - self.test_stats['test_start_time']
            
            # Create results summary
            results = {
                'test_type': self.test_stats['current_test_type'],
                'total_grasps': total_grasps,
                'successful_grasps': successful_grasps,
                'failed_grasps': self.test_stats['failed_grasps'],
                'success_rate': success_rate,
                'test_duration': test_duration,
                'passed': success_rate >= self.success_threshold
            }
            
            # Publish results
            results_msg = String()
            results_msg.data = json.dumps(results, indent=2)
            self.test_results_pub.publish(results_msg)
            
            status_message = f'{self.current_test_type.value} test completed: {success_rate:.2f} success rate'
            if results['passed']:
                status_message += ' (PASSED)'
            else:
                status_message += ' (FAILED)'
            
            self.publish_status(status_message)
            self.get_logger().info(f'Test completed: {results}')
        
        self.current_test_type = None
    
    def clear_test_objects(self):
        """Clear all test objects from scene"""
        try:
            for obj in self.test_objects:
                if obj in self.world.scene.object_list:
                    self.world.scene.remove_object(obj)
            
            self.test_objects.clear()
            
            # Remove test objects prim path
            stage = get_current_stage()
            test_objects_prim = stage.GetPrimAtPath('/World/TestObjects')
            if test_objects_prim:
                stage.RemovePrim('/World/TestObjects')
            
            # Recreate test objects container
            create_prim('/World/TestObjects', 'Xform')
            
            self.get_logger().debug('Test objects cleared')
            
        except Exception as e:
            self.get_logger().error(f'Failed to clear test objects: {e}')
    
    def reset_environment(self):
        """Reset entire test environment"""
        self.publish_status('Resetting test environment')
        
        # Stop any active test
        self.test_active = False
        
        # Clear test objects
        self.clear_test_objects()
        
        # Reset statistics
        self.test_stats = {
            'total_grasps': 0,
            'successful_grasps': 0,
            'failed_grasps': 0,
            'test_start_time': None,
            'current_test_type': None
        }
        
        self.publish_status('Environment reset complete')
        self.get_logger().info('Test environment reset')
    
    def publish_status(self, message):
        """Publish environment status"""
        msg = String()
        msg.data = f"[GRASP_SIM_ENV] {message}"
        self.environment_status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        environment = GraspSimulationEnvironment()
        rclpy.spin(environment)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Environment error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()