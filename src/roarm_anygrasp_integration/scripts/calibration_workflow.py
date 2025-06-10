#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from roarm_anygrasp_integration.srv import CalibrateCamera
import time
import sys
import os

class CalibrationWorkflow(Node):
    def __init__(self):
        super().__init__('calibration_workflow')
        
        # Service clients
        self.charuco_collect_client = self.create_client(String, '/charuco_calibration/collect_sample')
        self.charuco_calibrate_client = self.create_client(CalibrateCamera, '/charuco_calibration/calibrate_cameras')
        self.charuco_generate_client = self.create_client(String, '/charuco_calibration/generate_board')
        self.hand_eye_start_client = self.create_client(String, '/hand_eye_calibration/start_calibration')
        self.hand_eye_collect_client = self.create_client(String, '/hand_eye_calibration/collect_pose')
        self.hand_eye_compute_client = self.create_client(String, '/hand_eye_calibration/compute_calibration')
        
        # Status subscribers
        self.charuco_status_sub = self.create_subscription(
            String, '/charuco_calibration/status', self.charuco_status_callback, 10)
        self.hand_eye_status_sub = self.create_subscription(
            String, '/hand_eye_calibration/status', self.hand_eye_status_callback, 10)
        
        self.latest_charuco_status = ""
        self.latest_hand_eye_status = ""
        
        self.get_logger().info('Calibration Workflow initialized')
    
    def charuco_status_callback(self, msg):
        self.latest_charuco_status = msg.data
    
    def hand_eye_status_callback(self, msg):
        self.latest_hand_eye_status = msg.data
    
    def wait_for_services(self):
        """Wait for all calibration services to be available"""
        print("Waiting for calibration services...")
        
        services = [
            (self.charuco_collect_client, '/charuco_calibration/collect_sample'),
            (self.charuco_calibrate_client, '/charuco_calibration/calibrate_cameras'),
            (self.charuco_generate_client, '/charuco_calibration/generate_board'),
        ]
        
        for client, name in services:
            print(f"  Waiting for {name}...")
            if not client.wait_for_service(timeout_sec=10.0):
                print(f"  ❌ Service {name} not available")
                return False
            print(f"  ✅ Service {name} ready")
        
        print("All services ready!")
        return True
    
    def generate_calibration_board(self):
        """Generate ChArUco calibration board"""
        print("\n🎯 Generating ChArUco calibration board...")
        
        request = String.Request()
        future = self.charuco_generate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            print(f"✅ {future.result().data}")
            print("\n📋 INSTRUCTIONS:")
            print("1. Check the generated board file: /tmp/charuco_calibration/charuco_board.png")
            print("2. Print this board on A4 paper at actual size (no scaling)")
            print("3. Mount the board on a flat, rigid surface")
            print("4. Ensure good lighting and avoid reflections")
            return True
        else:
            print("❌ Failed to generate calibration board")
            return False
    
    def collect_charuco_samples(self, target_samples=15):
        """Collect ChArUco calibration samples"""
        print(f"\n📸 Collecting {target_samples} ChArUco calibration samples...")
        print("\n📋 INSTRUCTIONS:")
        print("1. Hold the printed ChArUco board in view of the camera")
        print("2. Move the board to different positions and orientations")
        print("3. Ensure the board is fully visible and well-lit")
        print("4. Press Enter to collect each sample")
        print("5. Type 'done' when you have enough samples")
        
        collected = 0
        while collected < target_samples:
            try:
                user_input = input(f"\nPress Enter to collect sample {collected + 1}/{target_samples} (or 'done' to finish): ").strip()
                
                if user_input.lower() == 'done':
                    break
                
                print("Collecting sample...")
                request = String.Request()
                future = self.charuco_collect_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    print(f"✅ {future.result().data}")
                    collected += 1
                else:
                    print("❌ Failed to collect sample")
                    
            except KeyboardInterrupt:
                print("\nCalibration collection interrupted")
                break
        
        print(f"\n📊 Collected {collected} samples")
        return collected >= 3  # Minimum samples needed
    
    def perform_charuco_calibration(self):
        """Perform ChArUco camera calibration"""
        print("\n🔬 Performing camera calibration...")
        
        request = CalibrateCamera.Request()
        future = self.charuco_calibrate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            result = future.result()
            if result.success:
                print(f"✅ {result.message}")
                print(f"📊 Primary camera error: {result.primary_reprojection_error:.4f}")
                print(f"📊 Side camera error: {result.side_reprojection_error:.4f}")
                return True
            else:
                print(f"❌ Calibration failed: {result.message}")
                return False
        else:
            print("❌ Calibration service call failed")
            return False
    
    def perform_hand_eye_calibration(self):
        """Perform hand-eye calibration workflow"""
        print("\n🤖 Starting hand-eye calibration...")
        print("\n📋 INSTRUCTIONS:")
        print("1. Ensure the robot is connected and MoveIt is running")
        print("2. Place the ChArUco board in the robot workspace")
        print("3. The robot will automatically move to different poses")
        print("4. Keep the board visible to the camera during movement")
        
        proceed = input("\nReady to start automatic hand-eye calibration? (y/n): ").strip().lower()
        if proceed != 'y':
            print("Hand-eye calibration skipped")
            return False
        
        print("Starting automatic calibration sequence...")
        request = String.Request()
        future = self.hand_eye_start_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=300.0)  # 5 minute timeout
        
        if future.result() is not None:
            print(f"✅ {future.result().data}")
            return "successfully" in future.result().data.lower()
        else:
            print("❌ Hand-eye calibration failed or timed out")
            return False
    
    def run_workflow(self):
        """Run the complete calibration workflow"""
        print("🎯 ChArUco Camera Calibration Workflow")
        print("=" * 50)
        
        if not self.wait_for_services():
            print("❌ Required services not available. Make sure calibration nodes are running.")
            return False
        
        # Step 1: Generate calibration board
        if not self.generate_calibration_board():
            return False
        
        input("\nPress Enter when you have printed and prepared the calibration board...")
        
        # Step 2: Collect calibration samples
        if not self.collect_charuco_samples():
            print("❌ Insufficient calibration samples collected")
            return False
        
        # Step 3: Perform camera calibration
        if not self.perform_charuco_calibration():
            return False
        
        # Step 4: Hand-eye calibration (optional)
        hand_eye = input("\nPerform hand-eye calibration? (y/n): ").strip().lower()
        if hand_eye == 'y':
            if not self.perform_hand_eye_calibration():
                print("⚠️  Hand-eye calibration failed, but camera calibration was successful")
        
        print("\n🎉 Calibration workflow completed!")
        print("\n📁 Check calibration results in:")
        print("   - /tmp/charuco_calibration/results/ (camera calibration)")
        print("   - /tmp/hand_eye_calibration/ (hand-eye calibration)")
        
        return True

def main():
    rclpy.init()
    
    print("🎯 ChArUco Camera Calibration Workflow")
    print("=" * 50)
    print("\nThis workflow will guide you through:")
    print("1. 📋 Generating a ChArUco calibration board")
    print("2. 📸 Collecting calibration images")
    print("3. 🔬 Computing camera calibration")
    print("4. 🤖 Hand-eye calibration (optional)")
    
    workflow = CalibrationWorkflow()
    
    try:
        success = workflow.run_workflow()
        if success:
            print("\n✅ All calibration steps completed successfully!")
        else:
            print("\n❌ Calibration workflow failed")
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nCalibration workflow interrupted by user")
    except Exception as e:
        print(f"\n❌ Calibration workflow error: {e}")
        sys.exit(1)
    finally:
        workflow.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()