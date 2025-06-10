#!/usr/bin/env python3
"""
NVIDIA Isaac Sim Launcher for RoArm M3

This script launches Isaac Sim with the RoArm M3 robot model and integrates
with the ROS2 workspace for robotic manipulation simulation.
"""

import os
import sys
import argparse
import subprocess
import time
from pathlib import Path

# Isaac Sim imports (will be available when Isaac Sim is installed)
try:
    from omni.isaac.kit import SimulationApp
    ISAAC_AVAILABLE = True
except ImportError:
    ISAAC_AVAILABLE = False
    print("Isaac Sim not found. Please install NVIDIA Isaac Sim.")

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

class IsaacSimLauncher(Node):
    def __init__(self):
        super().__init__('isaac_sim_launcher')
        
        self.declare_parameter('headless', False)
        self.declare_parameter('scene_path', '')
        self.declare_parameter('robot_model', 'roarm_m3')
        self.declare_parameter('enable_cameras', True)
        self.declare_parameter('enable_physics', True)
        self.declare_parameter('real_time_factor', 1.0)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/isaac_sim/status', 10)
        self.scene_ready_pub = self.create_publisher(Bool, '/isaac_sim/scene_ready', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.target_pose_sub = self.create_subscription(
            PoseStamped, '/isaac_sim/target_pose', self.target_pose_callback, 10)
        
        # Isaac Sim application
        self.simulation_app = None
        self.world = None
        self.robot = None
        self.scene_loaded = False
        
        self.get_logger().info('Isaac Sim Launcher initialized')
        
    def launch_isaac_sim(self):
        """Launch Isaac Sim with appropriate configuration"""
        if not ISAAC_AVAILABLE:
            self.get_logger().error('Isaac Sim not available')
            return False
            
        try:
            # Isaac Sim configuration
            headless = self.get_parameter('headless').get_parameter_value().bool_value
            
            config = {
                "headless": headless,
                "physics_dt": 1.0 / 60.0,  # 60 FPS physics
                "rendering_dt": 1.0 / 60.0,  # 60 FPS rendering
            }
            
            # Launch Isaac Sim
            self.simulation_app = SimulationApp(config)
            
            # Import Omniverse modules after SimulationApp initialization
            from omni.isaac.core import World
            from omni.isaac.core.robots import Robot
            from omni.isaac.core.utils.stage import add_reference_to_stage
            from omni.isaac.core.utils.nucleus import get_assets_root_path
            
            # Create world
            self.world = World(stage_units_in_meters=1.0)
            
            self.get_logger().info('Isaac Sim launched successfully')
            self.publish_status('Isaac Sim launched')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to launch Isaac Sim: {e}')
            self.publish_status(f'Launch failed: {e}')
            return False
    
    def load_roarm_scene(self):
        """Load RoArm M3 robot and scene"""
        if not self.world:
            return False
            
        try:
            from omni.isaac.core.robots import Robot
            from omni.isaac.core.utils.stage import add_reference_to_stage
            from omni.isaac.core.utils.prims import create_prim
            from omni.isaac.core.materials import PreviewSurface
            
            # Load RoArm M3 robot
            robot_asset_path = self.get_robot_asset_path()
            if robot_asset_path:
                # Add robot to stage
                robot_prim_path = "/World/RoArm_M3"
                add_reference_to_stage(usd_path=robot_asset_path, prim_path=robot_prim_path)
                
                # Create robot object
                self.robot = Robot(
                    prim_path=robot_prim_path,
                    name="roarm_m3",
                    position=[0.0, 0.0, 0.0],
                    orientation=[1.0, 0.0, 0.0, 0.0]
                )
                self.world.scene.add(self.robot)
                
            # Create workspace environment
            self.create_workspace_environment()
            
            # Create cameras matching real setup
            if self.get_parameter('enable_cameras').get_parameter_value().bool_value:
                self.setup_cameras()
            
            # Reset world
            self.world.reset()
            
            self.scene_loaded = True
            self.publish_status('Scene loaded successfully')
            
            scene_ready_msg = Bool()
            scene_ready_msg.data = True
            self.scene_ready_pub.publish(scene_ready_msg)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to load scene: {e}')
            self.publish_status(f'Scene loading failed: {e}')
            return False
    
    def get_robot_asset_path(self):
        """Get path to RoArm M3 USD asset"""
        # Try to find robot asset in various locations
        possible_paths = [
            "/root/ros2_workspace/src/roarm_isaac_sim/models/roarm_m3.usd",
            "/root/ros2_workspace/src/roarm_description/meshes/roarm_m3.usd",
            "omniverse://localhost/Projects/RoArm/roarm_m3.usd"
        ]
        
        for path in possible_paths:
            if os.path.exists(path) or path.startswith("omniverse://"):
                self.get_logger().info(f'Using robot asset: {path}')
                return path
        
        # If no custom asset found, create a basic robot representation
        self.get_logger().warning('No robot USD asset found, will create basic representation')
        return None
    
    def create_workspace_environment(self):
        """Create manipulation workspace with objects"""
        try:
            from omni.isaac.core.utils.prims import create_prim
            from omni.isaac.core.utils.stage import get_current_stage
            from pxr import UsdGeom, Gf
            
            stage = get_current_stage()
            
            # Create ground plane
            ground_path = "/World/GroundPlane"
            ground_prim = create_prim(ground_path, "Cube")
            ground_geom = UsdGeom.Cube(ground_prim)
            ground_geom.CreateSizeAttr(1.0)
            ground_geom.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.5))
            ground_geom.AddScaleOp().Set(Gf.Vec3d(2.0, 2.0, 0.1))
            
            # Create table
            table_path = "/World/Table"
            table_prim = create_prim(table_path, "Cube")
            table_geom = UsdGeom.Cube(table_prim)
            table_geom.CreateSizeAttr(1.0)
            table_geom.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 0.0))
            table_geom.AddScaleOp().Set(Gf.Vec3d(0.8, 0.6, 0.02))
            
            # Create manipulation objects
            object_positions = [
                (0.3, 0.0, 0.05),    # Center
                (0.4, 0.15, 0.05),   # Right
                (0.25, -0.12, 0.05), # Left
            ]
            
            for i, (x, y, z) in enumerate(object_positions):
                obj_path = f"/World/Object_{i}"
                obj_prim = create_prim(obj_path, "Cube")
                obj_geom = UsdGeom.Cube(obj_prim)
                obj_geom.CreateSizeAttr(1.0)
                obj_geom.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
                obj_geom.AddScaleOp().Set(Gf.Vec3d(0.04, 0.04, 0.04))
            
            self.get_logger().info('Workspace environment created')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create environment: {e}')
    
    def setup_cameras(self):
        """Setup cameras matching real robot configuration"""
        try:
            from omni.isaac.core.utils.prims import create_prim
            from omni.isaac.sensor import Camera
            from pxr import UsdGeom, Gf
            
            # RealSense D405 (wrist-mounted)
            d405_path = "/World/Cameras/D405"
            d405_prim = create_prim(d405_path, "Camera")
            d405_geom = UsdGeom.Camera(d405_prim)
            
            # Position relative to robot end-effector
            d405_geom.AddTranslateOp().Set(Gf.Vec3d(0.05, 0.0, 0.15))
            d405_geom.AddRotateXYZOp().Set(Gf.Vec3d(-90, 0, 0))
            
            # OAK-D (workspace overview)
            oak_d_path = "/World/Cameras/OAK_D"
            oak_d_prim = create_prim(oak_d_path, "Camera")
            oak_d_geom = UsdGeom.Camera(oak_d_prim)
            
            # Position for workspace overview
            oak_d_geom.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.3, 0.4))
            oak_d_geom.AddRotateXYZOp().Set(Gf.Vec3d(-45, 0, 0))
            
            self.get_logger().info('Cameras setup complete')
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup cameras: {e}')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates from ROS2"""
        if self.robot and self.scene_loaded:
            try:
                # Update robot joint positions in Isaac Sim
                joint_positions = {}
                for name, position in zip(msg.name, msg.position):
                    joint_positions[name] = position
                
                # Apply joint positions to robot
                self.robot.set_joint_positions(joint_positions)
                
            except Exception as e:
                self.get_logger().debug(f'Joint state update failed: {e}')
    
    def target_pose_callback(self, msg):
        """Handle target pose commands for robot"""
        if self.robot and self.scene_loaded:
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
                
                # Move robot to target pose (simplified)
                self.get_logger().info(f'Moving to target pose: {target_position}')
                
            except Exception as e:
                self.get_logger().error(f'Target pose execution failed: {e}')
    
    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = f"[ISAAC_SIM] {message}"
        self.status_pub.publish(msg)
        self.get_logger().info(message)
    
    def run_simulation(self):
        """Main simulation loop"""
        if not self.simulation_app:
            return
            
        try:
            while self.simulation_app.is_running():
                # Step simulation
                if self.world:
                    self.world.step(render=True)
                
                # Process ROS2 callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except KeyboardInterrupt:
            self.get_logger().info('Simulation interrupted by user')
        except Exception as e:
            self.get_logger().error(f'Simulation error: {e}')
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup Isaac Sim resources"""
        try:
            if self.world:
                self.world.stop()
            if self.simulation_app:
                self.simulation_app.close()
            self.get_logger().info('Isaac Sim cleanup complete')
        except Exception as e:
            self.get_logger().error(f'Cleanup error: {e}')


def main():
    parser = argparse.ArgumentParser(description='Launch Isaac Sim for RoArm M3')
    parser.add_argument('--headless', action='store_true', help='Run in headless mode')
    parser.add_argument('--scene', type=str, help='Scene file to load')
    args = parser.parse_args()
    
    if not ISAAC_AVAILABLE:
        print("ERROR: Isaac Sim not found. Please install NVIDIA Isaac Sim.")
        print("Visit: https://developer.nvidia.com/isaac-sim")
        return 1
    
    rclpy.init()
    
    try:
        launcher = IsaacSimLauncher()
        
        # Set parameters from command line
        if args.headless:
            launcher.set_parameters([rclpy.parameter.Parameter('headless', rclpy.Parameter.Type.BOOL, True)])
        if args.scene:
            launcher.set_parameters([rclpy.parameter.Parameter('scene_path', rclpy.Parameter.Type.STRING, args.scene)])
        
        # Launch Isaac Sim
        if launcher.launch_isaac_sim():
            # Load scene
            if launcher.load_roarm_scene():
                # Run simulation
                launcher.run_simulation()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'launcher' in locals():
            launcher.cleanup()
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())