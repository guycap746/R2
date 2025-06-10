#!/usr/bin/env python3
"""
LeRobot Teleoperation Interface

This module provides human teleoperation interfaces for data collection,
including joystick, keyboard, and gamepad control for demonstration recording.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import json
import time
from typing import Dict, List, Optional
import threading

# ROS2 message types
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Try to import keyboard library
try:
    import keyboard
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False

class TeleopMode:
    """Teleoperation modes"""
    JOINT_CONTROL = "joint_control"
    CARTESIAN_CONTROL = "cartesian_control"
    VELOCITY_CONTROL = "velocity_control"
    POSITION_CONTROL = "position_control"

class LeRobotTeleopInterface(Node):
    """Teleoperation interface for LeRobot data collection"""
    
    def __init__(self):
        super().__init__('lerobot_teleop_interface')
        
        # Parameters
        self.declare_parameter('joystick_device', '/dev/input/js0')
        self.declare_parameter('enable_keyboard', True)
        self.declare_parameter('enable_gamepad', True)
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('position_scaling', 0.01)
        self.declare_parameter('control_mode', 'joint_control')
        self.declare_parameter('publish_rate', 30)
        
        # Get parameters
        self.joystick_device = self.get_parameter('joystick_device').get_parameter_value().string_value
        self.enable_keyboard = self.get_parameter('enable_keyboard').get_parameter_value().bool_value
        self.enable_gamepad = self.get_parameter('enable_gamepad').get_parameter_value().bool_value
        self.velocity_scaling = self.get_parameter('velocity_scaling').get_parameter_value().double_value
        self.position_scaling = self.get_parameter('position_scaling').get_parameter_value().double_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value
        
        # State variables
        self.current_joint_state = None
        self.target_joint_positions = [0.0] * 6
        self.joint_velocities = [0.0] * 6
        self.teleop_active = False
        self.emergency_stop = False
        
        # Control input state
        self.joystick_state = None
        self.keyboard_state = {
            'joint_velocities': [0.0] * 6,
            'buttons': {}
        }
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/lerobot/teleop_status', 10)
        self.joint_command_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.cartesian_command_pub = self.create_publisher(Twist, '/cartesian_velocity_controller/cmd_vel', 10)
        
        # Data collection control publishers
        self.start_collection_pub = self.create_publisher(Bool, '/lerobot/start_collection', 10)
        self.stop_collection_pub = self.create_publisher(Bool, '/lerobot/stop_collection', 10)
        self.save_episode_pub = self.create_publisher(Bool, '/lerobot/save_episode', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10) if self.enable_gamepad else None
        
        # Control subscribers
        self.activate_teleop_sub = self.create_subscription(
            Bool, '/lerobot/activate_teleop', self.activate_teleop_callback, 10)
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10)
        
        # Control timer
        self.control_timer = self.create_timer(1.0 / self.publish_rate, self.control_loop)
        
        # Keyboard thread
        self.keyboard_thread = None
        if self.enable_keyboard and KEYBOARD_AVAILABLE:
            self.start_keyboard_interface()
        
        self.get_logger().info('LeRobot Teleoperation Interface initialized')
        self.publish_status('Teleop interface ready')
        self.print_usage_instructions()
    
    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        self.current_joint_state = msg
        if not self.target_joint_positions or len(self.target_joint_positions) != len(msg.position):
            self.target_joint_positions = list(msg.position)
    
    def joy_callback(self, msg):
        """Handle joystick/gamepad input"""
        self.joystick_state = msg
        
        # Process joystick commands
        if len(msg.axes) >= 6:
            # Map axes to joint velocities
            self.joint_velocities = [axis * self.velocity_scaling for axis in msg.axes[:6]]
        
        # Process buttons for data collection control
        if len(msg.buttons) > 0:
            # Button mapping for PS4/Xbox controller
            if len(msg.buttons) > 3:
                if msg.buttons[0] == 1:  # X/A button - start collection
                    self.trigger_start_collection()
                elif msg.buttons[1] == 1:  # Circle/B button - stop collection
                    self.trigger_stop_collection()
                elif msg.buttons[2] == 1:  # Triangle/Y button - save episode
                    self.trigger_save_episode()
                elif msg.buttons[3] == 1:  # Square/X button - emergency stop
                    self.trigger_emergency_stop()
    
    def activate_teleop_callback(self, msg):
        """Handle teleoperation activation"""
        self.teleop_active = msg.data
        status = "activated" if self.teleop_active else "deactivated"
        self.publish_status(f'Teleoperation {status}')
    
    def emergency_stop_callback(self, msg):
        """Handle emergency stop"""
        if msg.data:
            self.emergency_stop = True
            self.teleop_active = False
            self.joint_velocities = [0.0] * 6
            self.publish_status('EMERGENCY STOP ACTIVATED')
    
    def start_keyboard_interface(self):
        """Start keyboard input thread"""
        if not KEYBOARD_AVAILABLE:
            self.get_logger().warning('Keyboard library not available')
            return
        
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        self.get_logger().info('Keyboard interface started')
    
    def keyboard_loop(self):
        """Keyboard input processing loop"""
        try:
            while rclpy.ok():
                # Joint control keys
                velocity_map = {
                    'q': (0, self.velocity_scaling),   # Joint 1 positive
                    'a': (0, -self.velocity_scaling),  # Joint 1 negative
                    'w': (1, self.velocity_scaling),   # Joint 2 positive
                    's': (1, -self.velocity_scaling),  # Joint 2 negative
                    'e': (2, self.velocity_scaling),   # Joint 3 positive
                    'd': (2, -self.velocity_scaling),  # Joint 3 negative
                    'r': (3, self.velocity_scaling),   # Joint 4 positive
                    'f': (3, -self.velocity_scaling),  # Joint 4 negative
                    't': (4, self.velocity_scaling),   # Joint 5 positive
                    'g': (4, -self.velocity_scaling),  # Joint 5 negative
                    'y': (5, self.velocity_scaling),   # Joint 6 positive
                    'h': (5, -self.velocity_scaling),  # Joint 6 negative
                }
                
                # Reset velocities
                self.keyboard_state['joint_velocities'] = [0.0] * 6
                
                # Check for pressed keys
                for key, (joint_idx, velocity) in velocity_map.items():
                    if keyboard.is_pressed(key):
                        self.keyboard_state['joint_velocities'][joint_idx] = velocity
                
                # Control keys
                if keyboard.is_pressed('space'):
                    self.trigger_start_collection()
                    time.sleep(0.5)  # Debounce
                elif keyboard.is_pressed('enter'):
                    self.trigger_stop_collection()
                    time.sleep(0.5)  # Debounce
                elif keyboard.is_pressed('backspace'):
                    self.trigger_save_episode()
                    time.sleep(0.5)  # Debounce
                elif keyboard.is_pressed('esc'):
                    self.trigger_emergency_stop()
                    time.sleep(0.5)  # Debounce
                
                time.sleep(0.01)  # 100 Hz keyboard polling
                
        except Exception as e:
            self.get_logger().error(f'Keyboard loop error: {e}')
    
    def control_loop(self):
        """Main control loop"""
        if not self.teleop_active or self.emergency_stop:
            return
        
        try:
            # Determine control input source
            if self.control_mode == TeleopMode.JOINT_CONTROL:
                self.process_joint_control()
            elif self.control_mode == TeleopMode.CARTESIAN_CONTROL:
                self.process_cartesian_control()
            else:
                self.process_joint_control()  # Default
                
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}')
    
    def process_joint_control(self):
        """Process joint space teleoperation"""
        if not self.current_joint_state:
            return
        
        # Get velocity commands from active input source
        if self.joystick_state and self.enable_gamepad:
            velocities = self.joint_velocities
        elif self.enable_keyboard and KEYBOARD_AVAILABLE:
            velocities = self.keyboard_state['joint_velocities']
        else:
            velocities = [0.0] * 6
        
        # Integrate velocities to get target positions
        dt = 1.0 / self.publish_rate
        current_positions = list(self.current_joint_state.position)
        
        for i in range(min(len(velocities), len(current_positions))):
            self.target_joint_positions[i] = current_positions[i] + velocities[i] * dt
        
        # Apply joint limits (simplified)
        for i in range(len(self.target_joint_positions)):
            self.target_joint_positions[i] = np.clip(self.target_joint_positions[i], -3.14159, 3.14159)
        
        # Publish joint command
        self.publish_joint_command(self.target_joint_positions)
    
    def process_cartesian_control(self):
        """Process Cartesian space teleoperation"""
        if not self.joystick_state:
            return
        
        # Map joystick to Cartesian velocities
        if len(self.joystick_state.axes) >= 6:
            cartesian_cmd = Twist()
            cartesian_cmd.linear.x = self.joystick_state.axes[0] * self.velocity_scaling
            cartesian_cmd.linear.y = self.joystick_state.axes[1] * self.velocity_scaling
            cartesian_cmd.linear.z = self.joystick_state.axes[2] * self.velocity_scaling
            cartesian_cmd.angular.x = self.joystick_state.axes[3] * self.velocity_scaling
            cartesian_cmd.angular.y = self.joystick_state.axes[4] * self.velocity_scaling
            cartesian_cmd.angular.z = self.joystick_state.axes[5] * self.velocity_scaling
            
            self.cartesian_command_pub.publish(cartesian_cmd)
    
    def publish_joint_command(self, positions):
        """Publish joint trajectory command"""
        if not self.current_joint_state:
            return
        
        try:
            trajectory_msg = JointTrajectory()
            trajectory_msg.header.stamp = self.get_clock().now().to_msg()
            trajectory_msg.joint_names = list(self.current_joint_state.name)
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = positions[:len(trajectory_msg.joint_names)]
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int((1.0 / self.publish_rate) * 1e9)
            
            trajectory_msg.points = [point]
            
            # Publish command
            self.joint_command_pub.publish(trajectory_msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish joint command: {e}')
    
    def trigger_start_collection(self):
        """Trigger start of data collection"""
        msg = Bool()
        msg.data = True
        self.start_collection_pub.publish(msg)
        self.publish_status('Data collection start triggered')
    
    def trigger_stop_collection(self):
        """Trigger stop of data collection"""
        msg = Bool()
        msg.data = True
        self.stop_collection_pub.publish(msg)
        self.publish_status('Data collection stop triggered')
    
    def trigger_save_episode(self):
        """Trigger episode save"""
        msg = Bool()
        msg.data = True
        self.save_episode_pub.publish(msg)
        self.publish_status('Episode save triggered')
    
    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        self.emergency_stop = True
        self.teleop_active = False
        self.joint_velocities = [0.0] * 6
        self.publish_status('EMERGENCY STOP TRIGGERED')
    
    def print_usage_instructions(self):
        """Print usage instructions"""
        instructions = """
╔══════════════════════════════════════════════════════════════╗
║                   LeRobot Teleoperation Controls             ║
╠══════════════════════════════════════════════════════════════╣
║ KEYBOARD CONTROLS:                                           ║
║   Joint 1: Q (pos) / A (neg)                                ║
║   Joint 2: W (pos) / S (neg)                                ║
║   Joint 3: E (pos) / D (neg)                                ║
║   Joint 4: R (pos) / F (neg)                                ║
║   Joint 5: T (pos) / G (neg)                                ║
║   Joint 6: Y (pos) / H (neg)                                ║
║                                                              ║
║ DATA COLLECTION:                                             ║
║   Space     - Start recording                                ║
║   Enter     - Stop recording                                 ║
║   Backspace - Save episode                                   ║
║   Escape    - Emergency stop                                 ║
║                                                              ║
║ GAMEPAD CONTROLS:                                            ║
║   Left stick / Right stick - Joint control                  ║
║   X/A button - Start collection                             ║
║   Circle/B button - Stop collection                         ║
║   Triangle/Y button - Save episode                          ║
║   Square/X button - Emergency stop                          ║
║                                                              ║
║ ROS2 TOPICS:                                                ║
║   /lerobot/activate_teleop - Enable/disable teleop          ║
║   /lerobot/teleop_status - Status messages                  ║
╚══════════════════════════════════════════════════════════════╝
        """
        self.get_logger().info(instructions)
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f'[LEROBOT_TELEOP] {message}'
        self.status_pub.publish(msg)
        self.get_logger().info(message)


def main():
    rclpy.init()
    
    try:
        teleop = LeRobotTeleopInterface()
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Teleop interface error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()