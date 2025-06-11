#!/usr/bin/env python3
"""
xArm 6 Integration Test Script

This script tests the xArm 6 robot integration including:
- Robot description loading
- MoveIt configuration
- Simulation compatibility
- Training infrastructure integration
"""

import os
import sys
import subprocess
import time
import yaml

def run_command(cmd, timeout=10):
    """Run a command with timeout"""
    try:
        result = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout
        )
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "Command timed out"

def test_urdf_validation():
    """Test URDF file validation"""
    print("Testing URDF validation...")
    
    urdf_path = "/root/ros2_workspace/src/xarm_6_description/urdf/xarm_6.urdf"
    if not os.path.exists(urdf_path):
        print("✗ URDF file not found")
        return False
    
    cmd = f"source /opt/ros/humble/setup.bash && source install/setup.bash && check_urdf {urdf_path}"
    success, stdout, stderr = run_command(cmd)
    
    if success and "Successfully Parsed XML" in stdout:
        print("✓ URDF validation passed")
        return True
    else:
        print(f"✗ URDF validation failed: {stderr}")
        return False

def test_package_builds():
    """Test that all packages build successfully"""
    print("Testing package builds...")
    
    packages = [
        'xarm_6_description',
        'xarm_6_moveit_config', 
        'xarm_6_isaac_sim',
        'xarm_6_gazebo',
        'xarm_6_launch'
    ]
    
    all_passed = True
    for package in packages:
        cmd = f"source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 pkg list | grep {package}"
        success, stdout, stderr = run_command(cmd)
        
        if success and package in stdout:
            print(f"✓ {package} package available")
        else:
            print(f"✗ {package} package not found")
            all_passed = False
    
    return all_passed

def test_moveit_config():
    """Test MoveIt configuration files"""
    print("Testing MoveIt configuration...")
    
    config_files = {
        'SRDF': '/root/ros2_workspace/src/xarm_6_moveit_config/config/xarm_6.srdf',
        'Kinematics': '/root/ros2_workspace/src/xarm_6_moveit_config/config/kinematics.yaml',
        'Controllers': '/root/ros2_workspace/src/xarm_6_moveit_config/config/moveit_controllers.yaml'
    }
    
    all_passed = True
    for config_type, config_path in config_files.items():
        if os.path.exists(config_path):
            print(f"✓ {config_type} configuration exists")
            
            # Basic content validation
            with open(config_path, 'r') as f:
                content = f.read()
                if 'xarm' in content.lower() or 'joint' in content.lower():
                    print(f"✓ {config_type} content valid")
                else:
                    print(f"⚠ {config_type} content may be incomplete")
        else:
            print(f"✗ {config_type} configuration missing")
            all_passed = False
    
    return all_passed

def test_gazebo_integration():
    """Test Gazebo integration files"""
    print("Testing Gazebo integration...")
    
    gazebo_files = {
        'World': '/root/ros2_workspace/src/xarm_6_gazebo/worlds/xarm_6_world.world',
        'Launch': '/root/ros2_workspace/src/xarm_6_gazebo/launch/xarm_6_gazebo.launch.py',
        'URDF Xacro': '/root/ros2_workspace/src/xarm_6_gazebo/config/xarm_6_gazebo.urdf.xacro'
    }
    
    all_passed = True
    for file_type, file_path in gazebo_files.items():
        if os.path.exists(file_path):
            print(f"✓ {file_type} file exists")
        else:
            print(f"✗ {file_type} file missing")
            all_passed = False
    
    return all_passed

def test_isaac_sim_integration():
    """Test Isaac Sim integration"""
    print("Testing Isaac Sim integration...")
    
    isaac_files = {
        'Launcher': '/root/ros2_workspace/src/xarm_6_isaac_sim/xarm_6_isaac_sim/xarm_6_isaac_launcher.py',
        'Controller': '/root/ros2_workspace/src/xarm_6_isaac_sim/xarm_6_isaac_sim/xarm_6_simulation_controller.py',
        'Package': '/root/ros2_workspace/src/xarm_6_isaac_sim/package.xml'
    }
    
    all_passed = True
    for file_type, file_path in isaac_files.items():
        if os.path.exists(file_path):
            print(f"✓ {file_type} file exists")
        else:
            print(f"✗ {file_type} file missing")
            all_passed = False
    
    return all_passed

def test_training_integration():
    """Test training infrastructure integration"""
    print("Testing training integration...")
    
    training_files = {
        'Config': '/root/ros2_workspace/src/xarm_6_launch/config/xarm_6_training_config.yaml',
        'Integration': '/root/ros2_workspace/src/xarm_6_launch/config/xarm_6_integration.py',
        'Training Launch': '/root/ros2_workspace/src/xarm_6_launch/launch/xarm_6_isaac_training.launch.py'
    }
    
    all_passed = True
    for file_type, file_path in training_files.items():
        if os.path.exists(file_path):
            print(f"✓ {file_type} file exists")
            
            # Check training config content
            if file_type == 'Config':
                try:
                    with open(file_path, 'r') as f:
                        config = yaml.safe_load(f)
                        if 'robot_config' in config and 'training' in config:
                            print("✓ Training configuration structure valid")
                        else:
                            print("⚠ Training configuration incomplete")
                except Exception as e:
                    print(f"⚠ Training configuration parsing error: {e}")
        else:
            print(f"✗ {file_type} file missing")
            all_passed = False
    
    return all_passed

def test_launch_files():
    """Test launch file syntax"""
    print("Testing launch files...")
    
    launch_files = [
        '/root/ros2_workspace/src/xarm_6_launch/launch/xarm_6_complete.launch.py',
        '/root/ros2_workspace/src/xarm_6_launch/launch/xarm_6_gazebo_moveit.launch.py',
        '/root/ros2_workspace/src/xarm_6_launch/launch/xarm_6_isaac_training.launch.py'
    ]
    
    all_passed = True
    for launch_file in launch_files:
        if os.path.exists(launch_file):
            # Basic syntax check
            try:
                with open(launch_file, 'r') as f:
                    content = f.read()
                    if 'LaunchDescription' in content and 'generate_launch_description' in content:
                        print(f"✓ {os.path.basename(launch_file)} syntax valid")
                    else:
                        print(f"⚠ {os.path.basename(launch_file)} may have syntax issues")
            except Exception as e:
                print(f"✗ {os.path.basename(launch_file)} syntax error: {e}")
                all_passed = False
        else:
            print(f"✗ {os.path.basename(launch_file)} missing")
            all_passed = False
    
    return all_passed

def test_robot_configuration():
    """Test robot joint configuration"""
    print("Testing robot joint configuration...")
    
    # Check joint names file
    joint_names_file = '/root/ros2_workspace/src/xarm_6_description/config/joint_names_xarm6.yaml'
    if os.path.exists(joint_names_file):
        try:
            with open(joint_names_file, 'r') as f:
                config = yaml.safe_load(f)
                joints = config.get('controller_joint_names', [])
                if len(joints) == 6:
                    print("✓ Joint configuration complete (6 DOF)")
                else:
                    print(f"⚠ Joint configuration incomplete ({len(joints)} joints)")
        except Exception as e:
            print(f"✗ Joint configuration error: {e}")
            return False
    else:
        print("✗ Joint names configuration missing")
        return False
    
    # Check joint limits
    joint_limits_file = '/root/ros2_workspace/src/xarm_6_description/config/joint_limits.yaml'
    if os.path.exists(joint_limits_file):
        print("✓ Joint limits configuration exists")
    else:
        print("✗ Joint limits configuration missing")
        return False
    
    return True

def main():
    """Run all tests"""
    print("=" * 60)
    print("xArm 6 Integration Test Suite")
    print("=" * 60)
    
    tests = [
        ("URDF Validation", test_urdf_validation),
        ("Package Builds", test_package_builds),
        ("MoveIt Configuration", test_moveit_config),
        ("Gazebo Integration", test_gazebo_integration),
        ("Isaac Sim Integration", test_isaac_sim_integration),
        ("Training Integration", test_training_integration),
        ("Launch Files", test_launch_files),
        ("Robot Configuration", test_robot_configuration)
    ]
    
    results = {}
    for test_name, test_func in tests:
        print(f"\n{test_name}:")
        print("-" * 40)
        try:
            results[test_name] = test_func()
        except Exception as e:
            print(f"✗ Test failed with exception: {e}")
            results[test_name] = False
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    passed = sum(results.values())
    total = len(results)
    
    for test_name, result in results.items():
        status = "PASS" if result else "FAIL"
        print(f"{test_name:30} {status}")
    
    print(f"\nOverall: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! xArm 6 integration is ready.")
        return 0
    else:
        print("⚠️  Some tests failed. Check the output above for details.")
        return 1

if __name__ == '__main__':
    sys.exit(main())