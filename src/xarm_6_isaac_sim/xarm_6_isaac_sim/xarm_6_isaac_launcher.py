#!/usr/bin/env python3
"""
NVIDIA Isaac Sim Launcher for xArm 6

This script launches Isaac Sim with the xArm 6 robot model and integrates
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
from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class XArm6IsaacSimLauncher(Node):
    def __init__(self):
        super().__init__('xarm_6_isaac_sim_launcher')
        
        self.declare_parameter('headless', False)
        self.declare_parameter('scene_path', '')
        self.declare_parameter('robot_model', 'xarm_6')
        self.declare_parameter('enable_cameras', True)
        self.declare_parameter('enable_physics', True)
        self.declare_parameter('real_time_factor', 1.0)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/isaac_sim/status', 10)
        self.scene_ready_pub = self.create_publisher(Bool, '/isaac_sim/scene_ready', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers
        self.joint_trajectory_sub = self.create_subscription(
            JointTrajectory, '/xarm_6_joint_trajectory_controller/joint_trajectory', 
            self.joint_trajectory_callback, 10)
        self.target_pose_sub = self.create_subscription(
            PoseStamped, '/isaac_sim/target_pose', self.target_pose_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/isaac_sim/cmd_vel', self.cmd_vel_callback, 10)
        
        # Isaac Sim application
        self.simulation_app = None
        self.world = None
        self.robot = None
        self.scene_loaded = False
        
        # Robot state
        self.current_joint_positions = [0.0] * 6
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # Timer for publishing joint states
        self.joint_state_timer = self.create_timer(0.02, self.publish_joint_states)  # 50 Hz
        
        self.get_logger().info('xArm 6 Isaac Sim Launcher initialized')
        
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
    
    def load_xarm_6_scene(self):
        """Load xArm 6 robot and scene"""
        if not self.world:
            return False
            
        try:
            from omni.isaac.core.robots import Robot
            from omni.isaac.core.utils.stage import add_reference_to_stage
            from omni.isaac.core.utils.prims import create_prim
            from omni.isaac.core.materials import PreviewSurface
            
            # Load xArm 6 robot
            robot_asset_path = self.get_robot_asset_path()
            if robot_asset_path:
                # Add robot to stage
                robot_prim_path = "/World/XArm_6"
                add_reference_to_stage(usd_path=robot_asset_path, prim_path=robot_prim_path)
                
                # Create robot object
                self.robot = Robot(
                    prim_path=robot_prim_path,
                    name="xarm_6",
                    position=[0.0, 0.0, 0.0],
                    orientation=[1.0, 0.0, 0.0, 0.0]
                )
                self.world.scene.add(self.robot)
                
            else:
                # Create basic robot representation if no USD available
                self.create_basic_robot_representation()
            
            # Create workspace environment
            self.create_workspace_environment()
            
            # Create cameras matching real setup
            if self.get_parameter('enable_cameras').get_parameter_value().bool_value:
                self.setup_cameras()
            
            # Reset world
            self.world.reset()
            
            self.scene_loaded = True
            self.publish_status('xArm 6 scene loaded successfully')
            
            scene_ready_msg = Bool()
            scene_ready_msg.data = True
            self.scene_ready_pub.publish(scene_ready_msg)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to load scene: {e}')
            self.publish_status(f'Scene loading failed: {e}')
            return False
    
    def get_robot_asset_path(self):
        """Get path to xArm 6 USD asset"""
        # Try to find robot asset in various locations
        possible_paths = [
            "/root/ros2_workspace/src/xarm_6_isaac_sim/models/xarm_6.usd",
            "/root/ros2_workspace/src/xarm_6_description/meshes/xarm_6.usd",
            "omniverse://localhost/Projects/XArm/xarm_6.usd"
        ]
        
        for path in possible_paths:
            if os.path.exists(path) or path.startswith("omniverse://"):
                self.get_logger().info(f'Using robot asset: {path}')
                return path
        
        # If no custom asset found, create a basic robot representation
        self.get_logger().warning('No robot USD asset found, will create basic representation')
        return None
    
    def create_basic_robot_representation(self):
        """Create basic robot representation using primitives"""
        try:
            from omni.isaac.core.utils.prims import create_prim
            from omni.isaac.core.utils.stage import get_current_stage
            from pxr import UsdGeom, Gf
            
            stage = get_current_stage()
            
            # Create robot base
            base_path = "/World/XArm_6_Basic/base_link"
            base_prim = create_prim(base_path, "Cylinder")
            base_geom = UsdGeom.Cylinder(base_prim)
            base_geom.CreateRadiusAttr(0.08)
            base_geom.CreateHeightAttr(0.1)
            base_geom.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.05))
            
            # Create simplified joint links
            link_positions = [
                (0.0, 0.0, 0.2),   # Link 1
                (0.15, 0.0, 0.2),  # Link 2
                (0.4, 0.0, 0.2),   # Link 3
                (0.65, 0.0, 0.28), # Link 4
                (0.65, 0.0, 0.4),  # Link 5
                (0.65, 0.0, 0.48)  # Link 6
            ]
            
            for i, (x, y, z) in enumerate(link_positions, 1):
                link_path = f"/World/XArm_6_Basic/link{i}"
                link_prim = create_prim(link_path, "Cylinder")
                link_geom = UsdGeom.Cylinder(link_prim)
                link_geom.CreateRadiusAttr(0.04)
                link_geom.CreateHeightAttr(0.08)
                link_geom.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
            
            self.get_logger().info('Basic robot representation created')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create basic robot: {e}')
    
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
            ground_geom.AddScaleOp().Set(Gf.Vec3d(3.0, 3.0, 0.1))
            
            # Create table
            table_path = "/World/Table"
            table_prim = create_prim(table_path, "Cube")
            table_geom = UsdGeom.Cube(table_prim)
            table_geom.CreateSizeAttr(1.0)
            table_geom.AddTranslateOp().Set(Gf.Vec3d(0.7, 0.0, 0.0))
            table_geom.AddScaleOp().Set(Gf.Vec3d(1.0, 0.8, 0.02))
            
            # Create manipulation objects
            object_positions = [
                (0.5, 0.0, 0.05),    # Center
                (0.6, 0.2, 0.05),    # Right
                (0.4, -0.15, 0.05),  # Left
                (0.7, 0.1, 0.05),    # Far right
                (0.8, -0.1, 0.05),   # Far left
            ]
            
            for i, (x, y, z) in enumerate(object_positions):
                obj_path = f"/World/Object_{i}"
                obj_prim = create_prim(obj_path, "Cube")
                obj_geom = UsdGeom.Cube(obj_prim)
                obj_geom.CreateSizeAttr(1.0)
                obj_geom.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
                obj_geom.AddScaleOp().Set(Gf.Vec3d(0.05, 0.05, 0.05))
            
            self.get_logger().info('Workspace environment created')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create environment: {e}')
    
    def setup_cameras(self):
        """Setup cameras for xArm 6 simulation"""
        try:
            from omni.isaac.core.utils.prims import create_prim
            from omni.isaac.sensor import Camera
            from pxr import UsdGeom, Gf
            
            # End-effector camera
            ee_camera_path = "/World/Cameras/EndEffectorCamera"
            ee_camera_prim = create_prim(ee_camera_path, "Camera")
            ee_camera_geom = UsdGeom.Camera(ee_camera_prim)
            
            # Position relative to robot end-effector
            ee_camera_geom.AddTranslateOp().Set(Gf.Vec3d(0.7, 0.0, 0.6))
            ee_camera_geom.AddRotateXYZOp().Set(Gf.Vec3d(-45, 0, 0))
            
            # Workspace overview camera
            overview_camera_path = "/World/Cameras/OverviewCamera"
            overview_camera_prim = create_prim(overview_camera_path, "Camera")
            overview_camera_geom = UsdGeom.Camera(overview_camera_prim)
            
            # Position for workspace overview
            overview_camera_geom.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.5, 0.8))
            overview_camera_geom.AddRotateXYZOp().Set(Gf.Vec3d(-60, 0, 0))
            
            self.get_logger().info('Cameras setup complete')
            
        except Exception as e:
            self.get_logger().error(f'Failed to setup cameras: {e}')
    
    def joint_trajectory_callback(self, msg):
        """Handle joint trajectory commands from MoveIt"""
        if self.robot and self.scene_loaded and msg.points:
            try:
                # Use the last point in the trajectory as target
                target_point = msg.points[-1]
                joint_positions = {}
                
                for name, position in zip(msg.joint_names, target_point.positions):
                    joint_positions[name] = position
                    # Update current positions for state publishing
                    if name in self.joint_names:
                        idx = self.joint_names.index(name)
                        self.current_joint_positions[idx] = position
                
                # Apply joint positions to robot
                if hasattr(self.robot, 'set_joint_positions'):
                    self.robot.set_joint_positions(joint_positions)
                
                self.get_logger().debug(f'Applied joint trajectory: {joint_positions}')
                
            except Exception as e:
                self.get_logger().debug(f'Joint trajectory execution failed: {e}')
    
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
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands for robot"""
        if self.robot and self.scene_loaded:
            try:
                # Apply velocity commands (simplified implementation)
                self.get_logger().debug(f'Applying velocity: linear={msg.linear}, angular={msg.angular}')
                
            except Exception as e:
                self.get_logger().debug(f'Velocity command failed: {e}')
    
    def publish_joint_states(self):
        """Publish current joint states"""
        if self.scene_loaded:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = self.current_joint_positions
            msg.velocity = [0.0] * len(self.joint_names)
            msg.effort = [0.0] * len(self.joint_names)
            
            self.joint_state_pub.publish(msg)
    
    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = f"[XARM_6_ISAAC_SIM] {message}"
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
    parser = argparse.ArgumentParser(description='Launch Isaac Sim for xArm 6')
    parser.add_argument('--headless', action='store_true', help='Run in headless mode')
    parser.add_argument('--scene', type=str, help='Scene file to load')
    args = parser.parse_args()
    
    if not ISAAC_AVAILABLE:
        print("ERROR: Isaac Sim not found. Please install NVIDIA Isaac Sim.")
        print("Visit: https://developer.nvidia.com/isaac-sim")
        return 1
    
    rclpy.init()
    
    try:
        launcher = XArm6IsaacSimLauncher()
        
        # Set parameters from command line
        if args.headless:
            launcher.set_parameters([rclpy.parameter.Parameter('headless', rclpy.Parameter.Type.BOOL, True)])
        if args.scene:
            launcher.set_parameters([rclpy.parameter.Parameter('scene_path', rclpy.Parameter.Type.STRING, args.scene)])
        
        # Launch Isaac Sim
        if launcher.launch_isaac_sim():
            # Load scene
            if launcher.load_xarm_6_scene():
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