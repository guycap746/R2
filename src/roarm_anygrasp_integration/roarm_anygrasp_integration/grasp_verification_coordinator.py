#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from roarm_moveit.srv import MovePointCmd
import time
import threading

class GraspVerificationCoordinator(Node):
    def __init__(self):
        super().__init__('grasp_verification_coordinator')
        
        # State tracking
        self.verification_in_progress = False
        self.last_grasp_pose = None
        self.verification_lock = threading.Lock()
        
        # Verification poses (adjust based on your camera positioning)
        self.verification_poses = {
            'display_to_side_camera': {
                'x': 0.15,   # Position in front of side camera
                'y': -0.25,  # Towards the left side camera
                'z': 0.25,   # At good viewing height
                'description': 'Show grasp result to side camera'
            },
            'pre_display': {
                'x': 0.1,
                'y': -0.1, 
                'z': 0.3,
                'description': 'Intermediate position before display'
            },
            'return_home': {
                'x': 0.2,
                'y': 0.0,
                'z': 0.2,
                'description': 'Return to home position'
            }
        }
        
        # Parameters
        self.declare_parameter('auto_verification', True)
        self.declare_parameter('verification_display_time', 3.0)
        self.declare_parameter('verification_timeout', 15.0)
        
        # Subscribers
        self.coordinator_status_sub = self.create_subscription(
            String, '/grasp_coordinator/status', self.coordinator_status_callback, 10)
        
        # Publishers
        self.verification_status_pub = self.create_publisher(
            String, '/grasp_verification/status', 10)
        
        # Service clients
        self.move_point_client = self.create_client(MovePointCmd, '/move_point_cmd')
        
        # Services
        self.verify_grasp_service = self.create_service(
            String, '/grasp_verification/verify_grasp', self.verify_grasp_callback)
        
        # Wait for services
        self.get_logger().info("Waiting for move_point service...")
        self.move_point_client.wait_for_service(timeout_sec=10.0)
        
        self.get_logger().info('Grasp Verification Coordinator initialized')
        self.publish_status("Verification coordinator ready")
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f"[VERIFICATION] {message}"
        self.verification_status_pub.publish(msg)
        self.get_logger().info(message)
    
    def coordinator_status_callback(self, msg: String):
        """Monitor grasp coordinator for completion"""
        status = msg.data
        
        # Auto-trigger verification on grasp completion
        if (self.get_parameter('auto_verification').get_parameter_value().bool_value and
            "[COMPLETED]" in status and not self.verification_in_progress):
            
            self.get_logger().info("Grasp completed - starting verification display")
            threading.Thread(target=self.execute_verification_sequence, daemon=True).start()
    
    def execute_verification_sequence(self):
        """Execute the complete verification display sequence"""
        with self.verification_lock:
            if self.verification_in_progress:
                self.get_logger().warning("Verification already in progress")
                return
            
            self.verification_in_progress = True
        
        try:
            self.publish_status("Starting verification display sequence")
            
            # Step 1: Move to pre-display position
            self.publish_status("Moving to pre-display position")
            success = self.move_to_verification_pose('pre_display')
            if not success:
                self.publish_status("Failed to reach pre-display position")
                return
            
            time.sleep(1.0)  # Brief pause
            
            # Step 2: Move to display position (in front of side camera)
            self.publish_status("Positioning for side camera view")
            success = self.move_to_verification_pose('display_to_side_camera')
            if not success:
                self.publish_status("Failed to reach display position")
                return
            
            # Step 3: Hold position for verification capture
            display_time = self.get_parameter('verification_display_time').get_parameter_value().double_value
            self.publish_status(f"Displaying grasp result for {display_time} seconds")
            time.sleep(display_time)
            
            # Step 4: Return to home position
            self.publish_status("Returning to home position")
            success = self.move_to_verification_pose('return_home')
            if not success:
                self.publish_status("Failed to return to home position")
                return
            
            self.publish_status("Verification display sequence completed")
            
        except Exception as e:
            self.get_logger().error(f"Error in verification sequence: {e}")
            self.publish_status(f"Verification sequence failed: {e}")
        
        finally:
            with self.verification_lock:
                self.verification_in_progress = False
    
    def move_to_verification_pose(self, pose_name: str) -> bool:
        """Move robot to a specific verification pose"""
        if pose_name not in self.verification_poses:
            self.get_logger().error(f"Unknown verification pose: {pose_name}")
            return False
        
        pose_config = self.verification_poses[pose_name]
        
        try:
            # Create move request
            move_request = MovePointCmd.Request()
            move_request.x = pose_config['x']
            move_request.y = pose_config['y'] 
            move_request.z = pose_config['z']
            
            self.get_logger().info(f"Moving to {pose_name}: {pose_config['description']}")
            
            # Call move service
            future = self.move_point_client.call_async(move_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                self.get_logger().info(f"Successfully moved to {pose_name}")
                return True
            else:
                self.get_logger().error(f"Move service call failed for {pose_name}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error moving to {pose_name}: {e}")
            return False
    
    def verify_grasp_callback(self, request, response):
        """Service callback for manual verification"""
        try:
            if self.verification_in_progress:
                response.data = "Verification already in progress"
                return response
            
            self.get_logger().info("Manual verification requested")
            
            # Start verification in background thread
            threading.Thread(target=self.execute_verification_sequence, daemon=True).start()
            
            response.data = "Verification sequence started"
            
        except Exception as e:
            response.data = f"Verification service error: {e}"
            self.get_logger().error(f"Verification service error: {e}")
        
        return response
    
    def set_verification_pose(self, pose_name: str, x: float, y: float, z: float, description: str = ""):
        """Update or add a verification pose"""
        self.verification_poses[pose_name] = {
            'x': x,
            'y': y,
            'z': z,
            'description': description or f"Custom pose {pose_name}"
        }
        self.get_logger().info(f"Updated verification pose {pose_name}: ({x}, {y}, {z})")

def main(args=None):
    rclpy.init(args=args)
    node = GraspVerificationCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()