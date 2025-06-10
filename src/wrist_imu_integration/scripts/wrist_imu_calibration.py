#!/usr/bin/env python3
"""
Wrist IMU Calibration Node

Provides automated calibration workflow for the BNO055 IMU:
- Automatic calibration detection and guidance
- Calibration status monitoring
- Offset storage and loading
- Interactive calibration prompts
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import json
import os
import time
from datetime import datetime

from sensor_msgs.msg import Imu
from std_msgs.msg import String, UInt8
from geometry_msgs.msg import Vector3

class WristIMUCalibration(Node):
    def __init__(self):
        super().__init__('wrist_imu_calibration')
        
        # Parameters
        self.declare_parameter('auto_calibration', True)
        self.declare_parameter('calibration_timeout', 300.0)  # 5 minutes
        self.declare_parameter('save_calibration', True)
        self.declare_parameter('calibration_file', '~/.ros2/wrist_imu_calibration.json')
        self.declare_parameter('min_calibration_level', 2)  # 0-3 scale
        
        self.auto_calibration = self.get_parameter('auto_calibration').value
        self.calibration_timeout = self.get_parameter('calibration_timeout').value
        self.save_calibration = self.get_parameter('save_calibration').value
        self.calibration_file = os.path.expanduser(
            self.get_parameter('calibration_file').value)
        self.min_calibration_level = self.get_parameter('min_calibration_level').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers
        self.calib_status_sub = self.create_subscription(
            UInt8, '/wrist_imu/calib_status', self.calibration_status_callback, sensor_qos)
        self.imu_sub = self.create_subscription(
            Imu, '/wrist_imu/imu', self.imu_callback, sensor_qos)
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/wrist_imu/calibration_guidance', 10)
        self.calibration_complete_pub = self.create_publisher(
            String, '/wrist_imu/calibration_complete', 10)
        
        # State variables
        self.calibration_status = [0, 0, 0, 0]  # [sys, gyro, accel, mag]
        self.last_imu_data = None
        self.calibration_start_time = None
        self.is_calibrating = False
        self.calibration_complete = False
        
        # Calibration guidance state
        self.current_step = 0
        self.step_start_time = None
        self.steps_completed = set()
        
        # Load previous calibration if available
        self.load_calibration()
        
        # Timer for calibration guidance
        self.guidance_timer = self.create_timer(2.0, self.calibration_guidance_callback)
        
        self.get_logger().info('Wrist IMU calibration node started')
        if self.auto_calibration:
            self.get_logger().info('Automatic calibration enabled')
        
    def calibration_status_callback(self, msg):
        """Process calibration status from BNO055"""
        # Decode calibration status (4 2-bit values packed in uint8)
        status_byte = msg.data
        self.calibration_status = [
            (status_byte >> 6) & 0x03,  # System
            (status_byte >> 4) & 0x03,  # Gyroscope
            (status_byte >> 2) & 0x03,  # Accelerometer
            status_byte & 0x03          # Magnetometer
        ]
        
        # Check if calibration is complete
        all_calibrated = all(status >= self.min_calibration_level 
                           for status in self.calibration_status)
        
        if all_calibrated and not self.calibration_complete:
            self.calibration_complete = True
            self.on_calibration_complete()
        elif not all_calibrated and self.calibration_complete:
            self.calibration_complete = False
            if self.auto_calibration:
                self.start_calibration()
    
    def imu_callback(self, msg):
        """Store latest IMU data"""
        self.last_imu_data = msg
    
    def start_calibration(self):
        """Start calibration process"""
        if self.is_calibrating:
            return
            
        self.is_calibrating = True
        self.calibration_start_time = time.time()
        self.current_step = 0
        self.step_start_time = time.time()
        self.steps_completed.clear()
        
        self.get_logger().info('Starting IMU calibration process')
        self.publish_guidance('Starting IMU calibration. Please follow the guidance messages.')
    
    def calibration_guidance_callback(self):
        """Provide calibration guidance"""
        if not self.auto_calibration or not self.is_calibrating:
            return
            
        if self.calibration_complete:
            return
            
        # Check timeout
        if (time.time() - self.calibration_start_time) > self.calibration_timeout:
            self.get_logger().warn('Calibration timeout reached')
            self.is_calibrating = False
            return
        
        # Provide step-by-step guidance
        self.provide_calibration_step()
    
    def provide_calibration_step(self):
        """Provide current calibration step guidance"""
        current_time = time.time()
        step_duration = current_time - self.step_start_time
        
        sys_cal, gyro_cal, accel_cal, mag_cal = self.calibration_status
        
        # Step 0: Keep IMU still for gyroscope calibration
        if self.current_step == 0:
            if gyro_cal >= self.min_calibration_level:
                self.steps_completed.add(0)
                self.advance_to_next_step()
                return
            
            if step_duration < 10:  # Give 10 seconds for this step
                self.publish_guidance(
                    f'Step 1/4: Keep IMU perfectly still for gyroscope calibration. '
                    f'Gyro: {gyro_cal}/3 - {10-int(step_duration)}s remaining'
                )
            else:
                self.advance_to_next_step()
        
        # Step 1: Accelerometer calibration - various orientations
        elif self.current_step == 1:
            if accel_cal >= self.min_calibration_level:
                self.steps_completed.add(1)
                self.advance_to_next_step()
                return
                
            positions = [
                'flat on table (Z up)',
                'on side (Y up)', 
                'on end (X up)',
                'upside down (Z down)',
                'on other side (Y down)',
                'on other end (X down)'
            ]
            
            position_index = min(int(step_duration / 5), len(positions) - 1)
            self.publish_guidance(
                f'Step 2/4: Accelerometer calibration. '
                f'Place IMU {positions[position_index]}. '
                f'Accel: {accel_cal}/3'
            )
            
            if step_duration > 30:  # Move to next step after 30 seconds
                self.advance_to_next_step()
        
        # Step 2: Magnetometer calibration - figure-8 movements
        elif self.current_step == 2:
            if mag_cal >= self.min_calibration_level:
                self.steps_completed.add(2)
                self.advance_to_next_step()
                return
                
            self.publish_guidance(
                f'Step 3/4: Magnetometer calibration. '
                f'Move IMU in figure-8 patterns around all axes. '
                f'Mag: {mag_cal}/3'
            )
            
            if step_duration > 20:  # Move to next step after 20 seconds
                self.advance_to_next_step()
        
        # Step 3: System calibration - combined movements
        elif self.current_step == 3:
            if sys_cal >= self.min_calibration_level:
                self.steps_completed.add(3)
                self.on_calibration_complete()
                return
                
            self.publish_guidance(
                f'Step 4/4: System calibration. '
                f'Perform slow, smooth movements with the IMU. '
                f'System: {sys_cal}/3'
            )
            
            if step_duration > 15:  # Final step timeout
                if sys_cal >= 1:  # Accept lower system calibration
                    self.on_calibration_complete()
                else:
                    self.current_step = 1  # Restart from accelerometer
                    self.step_start_time = time.time()
    
    def advance_to_next_step(self):
        """Move to next calibration step"""
        self.current_step += 1
        self.step_start_time = time.time()
        
        if self.current_step > 3:
            self.on_calibration_complete()
    
    def on_calibration_complete(self):
        """Handle calibration completion"""
        self.is_calibrating = False
        self.calibration_complete = True
        
        sys_cal, gyro_cal, accel_cal, mag_cal = self.calibration_status
        
        completion_msg = (
            f'IMU Calibration Complete! '
            f'System: {sys_cal}/3, Gyro: {gyro_cal}/3, '
            f'Accel: {accel_cal}/3, Mag: {mag_cal}/3'
        )
        
        self.get_logger().info(completion_msg)
        self.publish_guidance(completion_msg)
        
        # Publish completion message
        complete_msg = String()
        complete_msg.data = f'COMPLETE:{sys_cal}:{gyro_cal}:{accel_cal}:{mag_cal}'
        self.calibration_complete_pub.publish(complete_msg)
        
        # Save calibration if enabled
        if self.save_calibration:
            self.save_calibration_data()
    
    def publish_guidance(self, message):
        """Publish calibration guidance message"""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f'Calibration: {message}')
    
    def save_calibration_data(self):
        """Save current calibration data to file"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.calibration_file), exist_ok=True)
            
            calibration_data = {
                'timestamp': datetime.now().isoformat(),
                'calibration_status': {
                    'system': self.calibration_status[0],
                    'gyroscope': self.calibration_status[1],
                    'accelerometer': self.calibration_status[2],
                    'magnetometer': self.calibration_status[3]
                },
                'imu_data': None
            }
            
            # Add IMU data if available
            if self.last_imu_data:
                calibration_data['imu_data'] = {
                    'orientation': {
                        'x': self.last_imu_data.orientation.x,
                        'y': self.last_imu_data.orientation.y,
                        'z': self.last_imu_data.orientation.z,
                        'w': self.last_imu_data.orientation.w
                    },
                    'angular_velocity': {
                        'x': self.last_imu_data.angular_velocity.x,
                        'y': self.last_imu_data.angular_velocity.y,
                        'z': self.last_imu_data.angular_velocity.z
                    },
                    'linear_acceleration': {
                        'x': self.last_imu_data.linear_acceleration.x,
                        'y': self.last_imu_data.linear_acceleration.y,
                        'z': self.last_imu_data.linear_acceleration.z
                    }
                }
            
            with open(self.calibration_file, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            self.get_logger().info(f'Calibration data saved to {self.calibration_file}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration: {e}')
    
    def load_calibration(self):
        """Load previous calibration data"""
        if not os.path.exists(self.calibration_file):
            self.get_logger().info('No previous calibration file found')
            return
            
        try:
            with open(self.calibration_file, 'r') as f:
                data = json.load(f)
            
            timestamp = data.get('timestamp', 'unknown')
            status = data.get('calibration_status', {})
            
            self.get_logger().info(f'Loaded calibration from {timestamp}')
            self.get_logger().info(
                f'Previous calibration: System: {status.get("system", 0)}/3, '
                f'Gyro: {status.get("gyroscope", 0)}/3, '
                f'Accel: {status.get("accelerometer", 0)}/3, '
                f'Mag: {status.get("magnetometer", 0)}/3'
            )
            
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = WristIMUCalibration()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()