#!/usr/bin/env python3
"""
xArm 6 Simulation Controller

This node provides high-level control interface for the xArm 6 robot
in Isaac Sim simulation, including motion planning integration and
task execution capabilities.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveGroupAction, MoveItErrorCodes

class XArm6SimulationController(Node):
    def __init__(self):
        super().__init__('xarm_6_simulation_controller')
        
        # Publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/xarm_6_joint_trajectory_controller/joint_trajectory', 10)
        self.target_pose_pub = self.create_publisher(
            PoseStamped, '/isaac_sim/target_pose', 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/isaac_sim/cmd_vel', 10)
        self.status_pub = self.create_publisher(
            String, '/xarm_6_controller/status', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.isaac_status_sub = self.create_subscription(
            String, '/isaac_sim/status', self.isaac_status_callback, 10)
        self.scene_ready_sub = self.create_subscription(
            Bool, '/isaac_sim/scene_ready', self.scene_ready_callback, 10)
        
        # Action clients
        self.move_group_client = ActionClient(self, MoveGroup, '/move_group')
        
        # Robot state
        self.current_joint_state = None
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.scene_ready = False
        
        # Predefined poses
        self.predefined_poses = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, -0.785, 0.0, 0.0, 0.785, 0.0],
            'vertical': [0.0, -1.5708, 0.0, 0.0, 1.5708, 0.0],
            'pick_pose': [0.0, -0.5, 0.5, 0.0, 1.0, 0.0],
            'place_pose': [1.57, -0.5, 0.5, 0.0, 1.0, 0.0]
        }
        
        # Timer for demonstration
        self.demo_timer = self.create_timer(10.0, self.run_demo_sequence)
        self.demo_step = 0
        
        self.get_logger().info('xArm 6 Simulation Controller initialized')
        
    def joint_state_callback(self, msg):
        """Update current joint state"""
        self.current_joint_state = msg
        
    def isaac_status_callback(self, msg):
        """Handle Isaac Sim status updates"""
        self.get_logger().debug(f'Isaac Sim status: {msg.data}')
        
    def scene_ready_callback(self, msg):
        """Handle scene ready notification"""
        self.scene_ready = msg.data
        if self.scene_ready:
            self.get_logger().info('Isaac Sim scene is ready')
            self.publish_status('Scene ready, starting demo sequence')
            # Move to home position
            self.move_to_joint_position('home')
        
    def move_to_joint_position(self, pose_name):
        """Move robot to predefined joint position"""
        if pose_name not in self.predefined_poses:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            return False
            
        joint_positions = self.predefined_poses[pose_name]
        return self.execute_joint_trajectory(joint_positions, duration=3.0)
        
    def execute_joint_trajectory(self, target_positions, duration=2.0):
        """Execute joint trajectory"""
        if not self.scene_ready:
            self.get_logger().warning('Scene not ready for trajectory execution')
            return False
            
        try:
            # Create trajectory message
            trajectory = JointTrajectory()
            trajectory.header.stamp = self.get_clock().now().to_msg()
            trajectory.joint_names = self.joint_names
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = target_positions
            point.velocities = [0.0] * len(self.joint_names)
            point.accelerations = [0.0] * len(self.joint_names)
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration % 1) * 1e9)
            
            trajectory.points = [point]
            
            # Publish trajectory
            self.joint_trajectory_pub.publish(trajectory)
            
            self.get_logger().info(f'Executing trajectory to: {target_positions}')
            self.publish_status(f'Moving to joint positions: {target_positions}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to execute trajectory: {e}')
            return False
    
    def move_to_cartesian_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """Move robot to cartesian pose"""
        try:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            
            # Convert RPY to quaternion (simplified)
            pose_msg.pose.orientation.w = 1.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            
            self.target_pose_pub.publish(pose_msg)
            
            self.get_logger().info(f'Moving to cartesian pose: ({x}, {y}, {z})')
            self.publish_status(f'Moving to cartesian pose: ({x}, {y}, {z})')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to move to cartesian pose: {e}')
            return False
    
    def run_demo_sequence(self):
        """Run demonstration sequence"""
        if not self.scene_ready:
            return
            
        demo_sequence = [
            ('home', 'Moving to home position'),
            ('ready', 'Moving to ready position'),
            ('vertical', 'Moving to vertical position'),
            ('pick_pose', 'Moving to pick pose'),
            ('place_pose', 'Moving to place pose'),
            ('home', 'Returning to home position')
        ]
        
        if self.demo_step < len(demo_sequence):
            pose_name, description = demo_sequence[self.demo_step]
            self.get_logger().info(f'Demo step {self.demo_step + 1}: {description}')
            self.move_to_joint_position(pose_name)
            self.demo_step += 1
        else:
            # Reset demo sequence
            self.demo_step = 0
            self.get_logger().info('Demo sequence completed, restarting...')
    
    def execute_pick_and_place(self, pick_pose, place_pose):
        """Execute pick and place operation"""
        try:
            self.get_logger().info('Starting pick and place operation')
            
            # Move to approach pose above pick
            approach_pose = pick_pose.copy()
            approach_pose[2] += 0.1  # 10cm above
            self.execute_joint_trajectory(approach_pose, duration=3.0)
            
            # Wait for movement to complete
            self.create_timer(4.0, lambda: self.execute_joint_trajectory(pick_pose, duration=2.0))
            
            # Simulate gripper close
            self.create_timer(7.0, lambda: self.publish_status('Gripper closed'))
            
            # Move to place position
            self.create_timer(8.0, lambda: self.execute_joint_trajectory(place_pose, duration=3.0))
            
            # Simulate gripper open
            self.create_timer(12.0, lambda: self.publish_status('Gripper opened'))
            
            # Return to home
            self.create_timer(13.0, lambda: self.move_to_joint_position('home'))
            
        except Exception as e:
            self.get_logger().error(f'Pick and place operation failed: {e}')
    
    def execute_velocity_control(self, linear_vel, angular_vel, duration=1.0):
        """Execute velocity control"""
        try:
            cmd_vel = Twist()
            cmd_vel.linear.x = linear_vel[0]
            cmd_vel.linear.y = linear_vel[1]
            cmd_vel.linear.z = linear_vel[2]
            cmd_vel.angular.x = angular_vel[0]
            cmd_vel.angular.y = angular_vel[1]
            cmd_vel.angular.z = angular_vel[2]
            
            # Publish velocity command for specified duration
            end_time = self.get_clock().now() + rclpy.duration.Duration(seconds=duration)
            
            def velocity_timer_callback():
                if self.get_clock().now() < end_time:
                    self.cmd_vel_pub.publish(cmd_vel)
                else:
                    # Stop motion
                    stop_cmd = Twist()
                    self.cmd_vel_pub.publish(stop_cmd)
                    timer.cancel()
            
            timer = self.create_timer(0.1, velocity_timer_callback)
            
            self.get_logger().info(f'Executing velocity control: linear={linear_vel}, angular={angular_vel}')
            
        except Exception as e:
            self.get_logger().error(f'Velocity control failed: {e}')
    
    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = f"[XARM_6_CONTROLLER] {message}"
        self.status_pub.publish(msg)
        self.get_logger().info(message)
    
    def get_current_joint_positions(self):
        """Get current joint positions"""
        if self.current_joint_state:
            positions = []
            for joint_name in self.joint_names:
                if joint_name in self.current_joint_state.name:
                    idx = self.current_joint_state.name.index(joint_name)
                    positions.append(self.current_joint_state.position[idx])
                else:
                    positions.append(0.0)
            return positions
        return [0.0] * 6
    
    def calculate_joint_interpolation(self, start_pos, end_pos, steps=10):
        """Calculate joint interpolation between two positions"""
        trajectory_points = []
        for i in range(steps + 1):
            alpha = i / steps
            interpolated_pos = [
                start + alpha * (end - start) 
                for start, end in zip(start_pos, end_pos)
            ]
            trajectory_points.append(interpolated_pos)
        return trajectory_points


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = XArm6SimulationController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()