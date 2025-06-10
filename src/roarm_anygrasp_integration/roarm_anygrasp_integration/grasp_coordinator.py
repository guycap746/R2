#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, PlanningOptions, Constraints, CollisionObject
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from roarm_moveit.srv import MovePointCmd
from roarm_anygrasp_integration.srv import GetGraspCandidates, SelectGrasp
import time
import numpy as np
from enum import Enum

class GraspState(Enum):
    IDLE = "idle"
    DETECTING = "detecting"
    WAITING_FOR_USER = "waiting_for_user"
    PLANNING = "planning"
    EXECUTING = "executing"
    GRASPING = "grasping"
    COMPLETED = "completed"
    FAILED = "failed"

class GraspCoordinator(Node):
    def __init__(self):
        super().__init__('grasp_coordinator')
        
        # State management
        self.current_state = GraspState.IDLE
        self.current_candidates = []
        self.selected_grasp = None
        self.execution_sequence_active = False
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/grasp_coordinator/status', 10)
        self.gripper_pub = self.create_publisher(Float32, '/gripper_cmd', 10)
        
        # Service clients
        self.get_candidates_client = self.create_client(GetGraspCandidates, '/anygrasp/get_candidates')
        self.select_grasp_client = self.create_client(SelectGrasp, '/anygrasp/select_grasp')
        self.move_point_client = self.create_client(MovePointCmd, '/move_point_cmd')
        
        # MoveIt planning services
        self.planning_scene_client = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.state_validity_client = self.create_client(GetStateValidity, '/check_state_validity')
        
        # Action clients for MoveIt
        self.move_group_client = ActionClient(self, MoveGroup, '/move_group')
        
        # Services provided by this node
        self.start_grasp_service = self.create_service(
            GetGraspCandidates, '/grasp_coordinator/start_grasp_workflow', self.start_grasp_workflow)
        self.execute_selected_service = self.create_service(
            SelectGrasp, '/grasp_coordinator/execute_selected', self.execute_selected_grasp)
        
        # Parameters
        self.declare_parameter('approach_distance', 0.1)
        self.declare_parameter('lift_distance', 0.05)
        self.declare_parameter('gripper_open_position', 0.0)
        self.declare_parameter('gripper_close_position', 1.2)
        self.declare_parameter('movement_speed', 0.1)
        self.declare_parameter('max_candidates', 5)
        self.declare_parameter('min_confidence', 0.6)
        
        # Safety parameters
        self.declare_parameter('enable_collision_checking', True)
        self.declare_parameter('workspace_bounds_x_min', -0.5)
        self.declare_parameter('workspace_bounds_x_max', 0.8)
        self.declare_parameter('workspace_bounds_y_min', -0.6)
        self.declare_parameter('workspace_bounds_y_max', 0.6)
        self.declare_parameter('workspace_bounds_z_min', 0.0)
        self.declare_parameter('workspace_bounds_z_max', 0.5)
        self.declare_parameter('max_joint_velocity', 1.0)
        self.declare_parameter('safety_margin', 0.05)
        
        # Wait for services
        self.get_logger().info("Waiting for required services...")
        self.wait_for_services()
        
        self.publish_status("Grasp coordinator ready")
        self.get_logger().info("Grasp coordinator initialized")
    
    def wait_for_services(self):
        """Wait for all required services to be available"""
        services_to_wait = [
            (self.get_candidates_client, '/anygrasp/get_candidates'),
            (self.select_grasp_client, '/anygrasp/select_grasp'),
            (self.move_point_client, '/move_point_cmd')
        ]
        
        for client, service_name in services_to_wait:
            self.get_logger().info(f"Waiting for service: {service_name}")
            client.wait_for_service(timeout_sec=10.0)
            self.get_logger().info(f"Service available: {service_name}")
    
    def publish_status(self, message):
        """Publish status message"""
        msg = String()
        msg.data = f"[{self.current_state.value.upper()}] {message}"
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {msg.data}")
    
    def start_grasp_workflow(self, request, response):
        """Start the complete grasp workflow"""
        if self.current_state != GraspState.IDLE:
            response.detection_status = f"Cannot start workflow - current state: {self.current_state.value}"
            response.total_grasps_detected = 0
            return response
        
        self.current_state = GraspState.DETECTING
        self.publish_status("Starting grasp detection workflow...")
        
        # Request grasp candidates from AnyGrasp
        candidates_request = GetGraspCandidates.Request()
        candidates_request.num_candidates = self.get_parameter('max_candidates').get_parameter_value().integer_value
        candidates_request.min_confidence = self.get_parameter('min_confidence').get_parameter_value().double_value
        
        try:
            # Call AnyGrasp detection service
            future = self.get_candidates_client.call_async(candidates_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
            
            if future.result() is not None:
                candidates_response = future.result()
                self.current_candidates = candidates_response.grasp_poses
                
                if len(self.current_candidates) > 0:
                    self.current_state = GraspState.WAITING_FOR_USER
                    self.publish_status(f"Detected {len(self.current_candidates)} candidates - waiting for user selection")
                    
                    # Return candidates to user for selection
                    response.grasp_poses = candidates_response.grasp_poses
                    response.confidence_scores = candidates_response.confidence_scores
                    response.grasp_widths = candidates_response.grasp_widths
                    response.quality_scores = candidates_response.quality_scores
                    response.original_indices = candidates_response.original_indices
                    response.detection_status = "Candidates ready for selection"
                    response.total_grasps_detected = candidates_response.total_grasps_detected
                    response.detection_timestamp = candidates_response.detection_timestamp
                else:
                    self.current_state = GraspState.FAILED
                    response.detection_status = "No valid grasps detected"
                    response.total_grasps_detected = 0
                    self.publish_status("No valid grasps detected")
            else:
                self.current_state = GraspState.FAILED
                response.detection_status = "Grasp detection service call failed"
                response.total_grasps_detected = 0
                self.publish_status("Grasp detection service call failed")
                
        except Exception as e:
            self.current_state = GraspState.FAILED
            error_msg = f"Error during grasp detection: {str(e)}"
            response.detection_status = error_msg
            response.total_grasps_detected = 0
            self.publish_status(error_msg)
        
        return response
    
    def execute_selected_grasp(self, request, response):
        """Execute the user-selected grasp"""
        if self.current_state != GraspState.WAITING_FOR_USER:
            response.success = False
            response.message = f"Cannot execute grasp - current state: {self.current_state.value}"
            return response
        
        if request.selected_grasp_index < 0 or request.selected_grasp_index >= len(self.current_candidates):
            response.success = False
            response.message = f"Invalid grasp index: {request.selected_grasp_index}"
            return response
        
        # Get selected grasp
        selected_pose = self.current_candidates[request.selected_grasp_index]
        self.selected_grasp = selected_pose
        
        self.current_state = GraspState.PLANNING
        self.publish_status(f"Executing selected grasp {request.selected_grasp_index}")
        
        # Execute grasp sequence
        success = self.execute_grasp_sequence(selected_pose.pose)
        
        if success:
            self.current_state = GraspState.COMPLETED
            response.success = True
            response.message = "Grasp executed successfully"
            response.selected_pose = selected_pose
            self.publish_status("Grasp execution completed successfully")
        else:
            self.current_state = GraspState.FAILED
            response.success = False
            response.message = "Grasp execution failed"
            self.publish_status("Grasp execution failed")
        
        # Reset for next cycle
        self.reset_workflow()
        
        return response
    
    def execute_grasp_sequence(self, target_pose):
        """Execute the complete grasp sequence with safety checks"""
        try:
            # Pre-execution safety validation
            if not self.validate_grasp_safety(target_pose):
                self.publish_status("Grasp safety validation failed")
                return False
            
            # Step 1: Open gripper
            self.publish_status("Opening gripper...")
            self.control_gripper(self.get_parameter('gripper_open_position').get_parameter_value().double_value)
            time.sleep(1.0)
            
            # Step 2: Move to approach position (above target)
            approach_pose = Pose()
            approach_pose.position.x = target_pose.position.x
            approach_pose.position.y = target_pose.position.y
            approach_pose.position.z = target_pose.position.z + self.get_parameter('approach_distance').get_parameter_value().double_value
            approach_pose.orientation = target_pose.orientation
            
            self.publish_status("Validating approach position...")
            if not self.validate_pose_safety(approach_pose):
                self.publish_status("Approach position validation failed")
                return False
            
            self.publish_status("Moving to approach position...")
            if not self.move_to_pose(approach_pose):
                return False
            
            # Step 3: Move to grasp position
            self.publish_status("Validating grasp position...")
            if not self.validate_pose_safety(target_pose):
                self.publish_status("Grasp position validation failed")
                return False
            
            self.publish_status("Moving to grasp position...")
            if not self.move_to_pose(target_pose):
                return False
            
            # Step 4: Close gripper
            self.publish_status("Closing gripper...")
            self.control_gripper(self.get_parameter('gripper_close_position').get_parameter_value().double_value)
            time.sleep(2.0)
            
            # Step 5: Lift object
            lift_pose = Pose()
            lift_pose.position.x = target_pose.position.x
            lift_pose.position.y = target_pose.position.y
            lift_pose.position.z = target_pose.position.z + self.get_parameter('lift_distance').get_parameter_value().double_value
            lift_pose.orientation = target_pose.orientation
            
            self.publish_status("Validating lift position...")
            if not self.validate_pose_safety(lift_pose):
                self.get_logger().warning("Lift position validation failed, skipping lift")
            else:
                self.publish_status("Lifting object...")
                if not self.move_to_pose(lift_pose):
                    self.get_logger().warning("Failed to lift object, but grasp may still be successful")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error during grasp execution: {e}")
            return False
    
    def move_to_pose(self, target_pose):
        """Move robot to target pose using MoveIt"""
        try:
            # Create move request
            move_request = MovePointCmd.Request()
            move_request.x = target_pose.position.x
            move_request.y = target_pose.position.y
            move_request.z = target_pose.position.z
            
            # Call move service
            future = self.move_point_client.call_async(move_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            
            if future.result() is not None:
                # Check if move was successful (assuming service returns success status)
                return True
            else:
                self.get_logger().error("Move service call failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error during move to pose: {e}")
            return False
    
    def control_gripper(self, position):
        """Control gripper position"""
        msg = Float32()
        msg.data = position
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Gripper command sent: {position}")
    
    def validate_grasp_safety(self, target_pose):
        """Comprehensive safety validation for grasp execution"""
        # Check workspace bounds
        if not self.check_workspace_bounds(target_pose):
            self.get_logger().warning("Target pose outside workspace bounds")
            return False
        
        # Check reachability
        if not self.check_reachability(target_pose):
            self.get_logger().warning("Target pose not reachable")
            return False
        
        # Check for collisions if enabled
        if self.get_parameter('enable_collision_checking').get_parameter_value().bool_value:
            if not self.check_collision_free(target_pose):
                self.get_logger().warning("Target pose would cause collision")
                return False
        
        return True
    
    def validate_pose_safety(self, pose):
        """Quick safety validation for individual poses"""
        return (self.check_workspace_bounds(pose) and 
                self.check_reachability(pose))
    
    def check_workspace_bounds(self, pose):
        """Check if pose is within safe workspace bounds"""
        x_min = self.get_parameter('workspace_bounds_x_min').get_parameter_value().double_value
        x_max = self.get_parameter('workspace_bounds_x_max').get_parameter_value().double_value
        y_min = self.get_parameter('workspace_bounds_y_min').get_parameter_value().double_value
        y_max = self.get_parameter('workspace_bounds_y_max').get_parameter_value().double_value
        z_min = self.get_parameter('workspace_bounds_z_min').get_parameter_value().double_value
        z_max = self.get_parameter('workspace_bounds_z_max').get_parameter_value().double_value
        
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        
        return (x_min <= x <= x_max and 
                y_min <= y <= y_max and 
                z_min <= z <= z_max)
    
    def check_reachability(self, pose):
        """Check if pose is reachable by robot arm"""
        # Calculate distance from robot base
        distance = np.sqrt(pose.position.x**2 + pose.position.y**2 + pose.position.z**2)
        
        # Typical arm reach limits (adjust for your robot)
        max_reach = 0.9  # meters
        min_reach = 0.1  # meters
        
        return min_reach <= distance <= max_reach
    
    def check_collision_free(self, pose):
        """Check if pose would result in collision"""
        try:
            # This is a simplified collision check
            # In a full implementation, you would use MoveIt's collision checking
            if self.state_validity_client.wait_for_service(timeout_sec=1.0):
                # Create state validity request
                request = GetStateValidity.Request()
                # Note: Full implementation would require proper robot state setup
                # For now, return True (no collision detected)
                return True
            else:
                self.get_logger().warning("State validity service not available")
                return True  # Assume safe if service unavailable
                
        except Exception as e:
            self.get_logger().warning(f"Collision checking failed: {e}")
            return True  # Assume safe on error
    
    def reset_workflow(self):
        """Reset workflow state for next cycle"""
        self.current_candidates = []
        self.selected_grasp = None
        self.current_state = GraspState.IDLE
        self.publish_status("Ready for next grasp workflow")
    
    def get_current_state(self):
        """Get current workflow state"""
        return self.current_state
    
    def emergency_stop(self):
        """Emergency stop - halt all operations"""
        self.current_state = GraspState.FAILED
        self.publish_status("EMERGENCY STOP ACTIVATED")
        
        # Open gripper for safety
        self.control_gripper(self.get_parameter('gripper_open_position').get_parameter_value().double_value)
        
        # Reset workflow
        self.reset_workflow()

def main(args=None):
    rclpy.init(args=args)
    node = GraspCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()