# NVIDIA Isaac Sim Integration for RoArm M3

This document provides complete setup and usage instructions for integrating NVIDIA Isaac Sim with the RoArm M3 robotic manipulation system.

## 🎯 Overview

The Isaac Sim integration provides:
- **Photorealistic simulation** of the RoArm M3 robot and workspace
- **Synthetic data generation** for training AnyGrasp and other ML models
- **Physics-based grasp simulation** for testing and validation
- **Sim-to-real transfer** capabilities
- **Multi-camera sensor simulation** matching real hardware
- **Domain randomization** for robust model training

## 📋 Prerequisites

### Hardware Requirements
- **GPU**: NVIDIA RTX 2070 or better (RTX 3080+ recommended)
- **VRAM**: 8GB minimum (16GB+ recommended for complex scenes)
- **RAM**: 32GB recommended
- **Storage**: 50GB free space for Isaac Sim + datasets

### Software Requirements
- **Operating System**: Ubuntu 20.04/22.04 or Windows 10/11
- **NVIDIA Driver**: 470.57.02 or later
- **ROS2 Humble**: Native installation recommended

## 🚀 Installation

### Step 1: Install NVIDIA Isaac Sim

#### Option A: Omniverse Launcher (Recommended)
```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable and run
chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage

# In Omniverse Launcher:
# 1. Create account/sign in
# 2. Go to "Exchange" tab
# 3. Search for "Isaac Sim"
# 4. Install Isaac Sim 2023.1.1 or later
```

#### Option B: Direct Download
```bash
# Download Isaac Sim directly (requires NVIDIA Developer account)
# Visit: https://developer.nvidia.com/isaac-sim
# Download the Linux package and extract to /opt/isaac-sim/
```

### Step 2: Install Isaac ROS
```bash
# Install Isaac ROS packages
sudo apt update
sudo apt install ros-humble-isaac-ros-common \
                 ros-humble-isaac-ros-visual-slam \
                 ros-humble-isaac-ros-apriltag \
                 ros-humble-isaac-ros-depth-segmentation

# Install additional dependencies
pip3 install omni-isaac-sim numpy opencv-python
```

### Step 3: Build RoArm Isaac Integration
```bash
# Navigate to workspace
cd /root/ros2_workspace

# Build Isaac Sim package
source /opt/ros/humble/setup.bash
colcon build --packages-select roarm_isaac_sim

# Source the workspace
source install/setup.bash
```

### Step 4: Verify Installation
```bash
# Test Isaac Sim installation
python3 -c "from omni.isaac.kit import SimulationApp; print('Isaac Sim available')"

# Test ROS2 integration
ros2 pkg list | grep isaac
```

## 🎮 Quick Start

### Basic Isaac Sim Launch
```bash
# Launch Isaac Sim with RoArm M3
ros2 launch roarm_isaac_sim isaac_sim_complete.launch.py

# Launch in headless mode (no GUI)
ros2 launch roarm_isaac_sim isaac_sim_complete.launch.py headless:=true

# Enable synthetic data generation
ros2 launch roarm_isaac_sim isaac_sim_complete.launch.py enable_synthetic_data:=true
```

### Generate Synthetic Training Data
```bash
# Start data generation
ros2 topic pub /synthetic_data/generate_batch std_msgs/Int32 "data: 100"

# Monitor progress
ros2 topic echo /synthetic_data/progress
ros2 topic echo /synthetic_data/status
```

### Test AnyGrasp in Simulation
```bash
# Launch complete system
ros2 launch roarm_isaac_sim isaac_sim_complete.launch.py enable_anygrasp:=true

# Trigger grasp detection
ros2 service call /grasp_coordinator/start_grasp_workflow \
  roarm_anygrasp_integration/srv/GetGraspCandidates \
  "{num_candidates: 5, min_confidence: 0.6}"
```

## 🏗️ Architecture

### Core Components

#### 1. Isaac Sim Launcher (`isaac_sim_launcher.py`)
- Initializes Isaac Sim environment
- Loads RoArm M3 robot model
- Sets up workspace and objects
- Manages simulation lifecycle

#### 2. Isaac-ROS Bridge (`isaac_ros_bridge.py`)
- Publishes sensor data (cameras, IMU, joint states)
- Receives robot commands from ROS2
- Maintains real-time synchronization
- Handles coordinate frame transforms

#### 3. Synthetic Data Generator (`synthetic_data_generator.py`)
- Generates randomized scenes
- Applies domain randomization
- Captures multi-modal sensor data
- Creates training annotations

#### 4. Sim-to-Real Calibration (`sim_to_real_calibration.py`)
- Matches simulation physics to real world
- Calibrates sensor noise models
- Validates grasp transfer performance

### Data Flow
```
Isaac Sim ←→ Isaac-ROS Bridge ←→ ROS2 Ecosystem
    ↓                                    ↑
Synthetic Data Gen                 AnyGrasp Integration
    ↓                                    ↑
Training Datasets              Real Robot Commands
```

## 🛠️ Configuration

### Robot Model Setup
```yaml
# config/isaac_sim_parameters.yaml
isaac_sim_launcher:
  ros__parameters:
    robot_model: "roarm_m3"
    robot_asset_path: "/path/to/roarm_m3.usd"  # Optional custom path
    enable_physics: true
    real_time_factor: 1.0
```

### Camera Configuration
```yaml
isaac_ros_bridge:
  ros__parameters:
    cameras:
      d405:
        frame_id: "d405_color_optical_frame"
        width: 640
        height: 480
        fx: 615.0
        fy: 615.0
      oak_d:
        frame_id: "oak_d_color_optical_frame"
        width: 1280
        height: 720
        fx: 860.0
        fy: 860.0
```

### Synthetic Data Settings
```yaml
synthetic_data_generator:
  ros__parameters:
    output_directory: "/data/isaac_synthetic"
    num_scenes_per_batch: 100
    enable_domain_randomization: true
    objects_per_scene_min: 3
    objects_per_scene_max: 8
```

## 📊 Synthetic Data Generation

### Scene Randomization
- **Object placement**: Random positions, orientations, scales
- **Material properties**: Varied textures, colors, surface properties
- **Lighting conditions**: Different intensities, colors, angles
- **Camera poses**: Multiple viewpoints for robust training

### Output Formats
```
synthetic_data/
├── batch_0001/
│   ├── scene_0001/
│   │   ├── main_rgb.png
│   │   ├── main_depth.npy
│   │   ├── side_rgb.png
│   │   ├── wrist_rgb.png
│   │   ├── annotations.json
│   │   └── metadata.json
│   └── scene_0002/
│       └── ...
└── batch_0002/
    └── ...
```

### Annotation Format
```json
{
  "objects": [
    {
      "id": 0,
      "type": "cube",
      "bbox_3d": {"min": [0.1, 0.1, 0.02], "max": [0.14, 0.14, 0.06]},
      "bbox_2d": {"x": 320, "y": 240, "width": 50, "height": 50},
      "pose": {
        "position": [0.12, 0.12, 0.04],
        "rotation": [0, 0, 0]
      }
    }
  ],
  "grasps": [
    {
      "object_id": 0,
      "position": [0.12, 0.12, 0.06],
      "orientation": [0, 0, 0, 1],
      "width": 0.04,
      "confidence": 0.85
    }
  ]
}
```

## 🔧 Advanced Usage

### Custom Scene Creation
```python
# Create custom manipulation scene
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_prim

# Add custom objects
add_reference_to_stage(
    usd_path="omniverse://localhost/Library/custom_object.usd",
    prim_path="/World/CustomObject"
)

# Create procedural environment
create_prim("/World/Table", "Cube", 
           position=[0.5, 0, 0], 
           scale=[0.8, 0.6, 0.02])
```

### Domain Randomization Scripts
```python
# Advanced randomization example
import omni.replicator.core as rep

# Randomize lighting
lights = rep.create.light(
    light_type="Sphere",
    intensity=rep.distribution.uniform(500, 2000),
    temperature=rep.distribution.uniform(3000, 6500),
    position=rep.distribution.uniform((-1, -1, 0.5), (1, 1, 2))
)

# Randomize materials
materials = rep.create.material_omnipbr(
    diffuse=rep.distribution.uniform((0, 0, 0), (1, 1, 1)),
    roughness=rep.distribution.uniform(0.1, 0.9),
    metallic=rep.distribution.uniform(0, 1)
)
```

### Sim-to-Real Transfer
```python
# Physics parameter matching
from omni.physx import get_physx_interface

# Match friction coefficients
physx = get_physx_interface()
physx.set_default_material_friction(0.7)  # Measured from real robot

# Match joint dynamics
robot.set_joint_friction_coefficients([0.1, 0.1, 0.1, 0.05, 0.05, 0.05])
```

## 🧪 Testing and Validation

### Basic System Test
```bash
# Test Isaac Sim connection
ros2 topic echo /isaac_sim/status

# Test camera data
ros2 topic echo /camera/color/image_raw --once

# Test joint states
ros2 topic echo /joint_states
```

### Grasp Simulation Test
```bash
# Test grasp workflow
ros2 service call /grasp_coordinator/start_grasp_workflow \
  roarm_anygrasp_integration/srv/GetGraspCandidates "{}"

# Monitor grasp execution
ros2 topic echo /grasp_coordinator/status
```

### Performance Benchmarks
```bash
# Test rendering performance
ros2 param set /isaac_ros_bridge update_frequency 120.0

# Test physics performance
ros2 param set /isaac_sim_launcher physics_dt 0.008333  # 120 FPS
```

## 🐛 Troubleshooting

### Common Issues

**1. Isaac Sim Not Starting**
```bash
# Check GPU compatibility
nvidia-smi
lspci | grep -i nvidia

# Verify Omniverse installation
ls -la ~/.nvidia-omniverse/

# Check log files
tail -f ~/.nvidia-omniverse/logs/launcher.log
```

**2. ROS2 Bridge Connection Issues**
```bash
# Check Isaac Sim world status
ros2 topic echo /isaac_sim/scene_ready

# Verify bridge parameters
ros2 param list /isaac_ros_bridge

# Test manual connection
python3 -c "from omni.isaac.core import World; print(World.instance())"
```

**3. Performance Issues**
```bash
# Reduce rendering quality
ros2 param set /isaac_sim_launcher rendering_dt 0.033333  # 30 FPS

# Disable unnecessary features
ros2 param set /isaac_ros_bridge publish_camera_data false

# Check system resources
htop
nvidia-smi
```

**4. Synthetic Data Generation Problems**
```bash
# Check output directory permissions
ls -la /tmp/isaac_synthetic_data/

# Monitor generation progress
ros2 topic echo /synthetic_data/progress

# Check disk space
df -h /tmp/
```

### Debug Mode
```bash
# Enable debug logging
ros2 launch roarm_isaac_sim isaac_sim_complete.launch.py \
  --ros-args --log-level debug

# View Isaac Sim console output
tail -f ~/.nvidia-omniverse/logs/Kit/Isaac-Sim/*/kit.log
```

## 📈 Performance Optimization

### GPU Optimization
- Use RTX 3080 or better for optimal performance
- Enable RTX ray tracing for photorealistic rendering
- Use DLSS for improved frame rates

### Memory Management
- Limit number of simultaneous objects (< 50 per scene)
- Use LOD (Level of Detail) models for distant objects
- Clear unused assets between scene generations

### Network Optimization
- Use local Omniverse Nucleus server for faster asset loading
- Cache frequently used models locally
- Optimize mesh complexity for simulation objects

## 🔗 Integration with Existing Systems

### AnyGrasp Integration
The Isaac Sim integration seamlessly connects with existing AnyGrasp workflows:
- Real point cloud data from simulated cameras
- Physics-based grasp success validation
- Automated dataset generation for model improvement

### Multi-Camera Support
Simulated cameras match real hardware configuration:
- RealSense D405 wrist-mounted camera
- OAK-D workspace overview camera
- OAK-1 high-speed tracking camera

### IMU Integration
Simulated IMU data for testing sensor fusion:
- BNO055 wrist IMU simulation
- Camera stabilization testing
- Motion prediction validation

## 📚 Further Resources

- **Isaac Sim Documentation**: https://docs.omniverse.nvidia.com/isaacsim/
- **Isaac ROS**: https://nvidia-isaac-ros.github.io/
- **Omniverse Replicator**: https://docs.omniverse.nvidia.com/replicator/
- **RoArm M3 Isaac Examples**: `src/RoArm-M3/isaac_sim/examples/`

## 🤝 Contributing

When extending the Isaac Sim integration:

1. **Test in Simulation First**: Validate all changes in Isaac Sim before real robot testing
2. **Maintain Sim-to-Real Consistency**: Ensure simulation parameters match real world
3. **Document Performance Impact**: Note any performance implications of new features
4. **Update Asset Library**: Add new objects and materials to shared library
5. **Validate Data Quality**: Ensure synthetic data maintains quality standards

The Isaac Sim integration provides a powerful platform for developing, testing, and training robotic manipulation algorithms in a safe, controlled, and highly configurable environment.