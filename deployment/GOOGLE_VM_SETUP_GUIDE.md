# Google VM Setup Guide for R2 RoArm M3 System

## Overview
This guide provides complete instructions for rebuilding the R2 RoArm M3 system on Google Cloud VM instances.

## VM Requirements

### Minimum Specifications
- **CPU**: 4 vCPUs (Intel/AMD)
- **RAM**: 16 GB (for training and simulation)
- **Storage**: 100 GB SSD
- **OS**: Ubuntu 22.04 LTS
- **GPU**: Optional - NVIDIA T4 or better for ML training

### Recommended Specifications
- **CPU**: 8 vCPUs
- **RAM**: 32 GB
- **Storage**: 200 GB SSD
- **GPU**: NVIDIA V100 or A100 for optimal training performance

## Pre-Setup Checklist

### 1. GitHub Repository Access
- [ ] Clone R2 repository: `git clone https://github.com/guycap746/R2.git`
- [ ] Ensure SSH keys or token access configured

### 2. System Files to Transfer
- [ ] `requirements.txt` (Python packages)
- [ ] `system_packages.txt` (System packages)
- [ ] `apt_packages_detailed.txt` (Detailed package info)
- [ ] All source code from `/src` directory

### 3. Hardware Considerations for VM
- [ ] No physical robot hardware (simulation mode)
- [ ] No cameras (use simulated data or test images)
- [ ] Network-based development and testing only

## Installation Script

### Quick Start Command
```bash
curl -sSL https://raw.githubusercontent.com/guycap746/R2/main/setup_google_vm.sh | bash
```

## Manual Setup Steps

### 1. Base System Setup
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y git curl wget vim nano htop build-essential

# Install Python and pip
sudo apt install -y python3 python3-pip python3-venv
```

### 2. ROS2 Humble Installation
```bash
# Add ROS2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop-full

# Install development tools
sudo apt install -y ros-dev-tools python3-colcon-common-extensions
```

### 3. MoveIt and Additional ROS2 Packages
```bash
# Install MoveIt2
sudo apt install -y ros-humble-moveit

# Install additional ROS2 packages
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro
```

### 4. Python Dependencies
```bash
# Install from requirements.txt
pip3 install -r requirements.txt

# Key packages for the system
pip3 install \
  numpy \
  opencv-python \
  pyrealsense2 \
  transforms3d \
  scipy \
  matplotlib \
  pyyaml
```

### 5. Isaac Sim Setup (Optional)
```bash
# Download and install Isaac Sim
# Note: Requires NVIDIA GPU
wget https://developer.nvidia.com/isaac-sim-download
# Follow official installation guide
```

### 6. Workspace Setup
```bash
# Clone the repository
cd ~
git clone https://github.com/guycap746/R2.git ros2_workspace
cd ros2_workspace

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Build the workspace
colcon build --symlink-install

# Source workspace
echo "source ~/ros2_workspace/install/setup.bash" >> ~/.bashrc
source ~/ros2_workspace/install/setup.bash
```

## VM-Specific Configurations

### 1. Hardware Simulation Mode
```bash
# Set environment variables for simulation
export ROS_DOMAIN_ID=42
export ROBOT_SIMULATION=true
export USE_FAKE_HARDWARE=true
```

### 2. Network Configuration
```bash
# Configure ROS2 networking for VM
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### 3. Display and GUI Setup
```bash
# For GUI applications (if using X11 forwarding)
sudo apt install -y xauth x11-apps

# For VNC server (alternative)
sudo apt install -y tightvncserver
vncserver :1 -geometry 1920x1080 -depth 24
```

## Testing and Verification

### 1. Basic ROS2 Test
```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_py listener
```

### 2. Workspace Build Test
```bash
cd ~/ros2_workspace
colcon build --packages-select roarm_description
ros2 launch roarm_description display.launch.py
```

### 3. Simulation Test
```bash
# Test Gazebo simulation
ros2 launch xarm_6_gazebo xarm_6_gazebo.launch.py

# Test MoveIt
ros2 launch xarm_6_moveit_config demo.launch.py
```

## Performance Optimization

### 1. VM Resource Allocation
- Allocate maximum available RAM
- Enable all available CPU cores
- Use SSD storage for better I/O performance

### 2. ROS2 Performance Tuning
```bash
# Increase shared memory for large messages
echo "kernel.shmmax = 268435456" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

### 3. Build Optimization
```bash
# Use parallel builds
export MAKEFLAGS="-j$(nproc)"
colcon build --parallel-workers $(nproc)
```

## Troubleshooting

### Common Issues

1. **GPU Not Available**
   - Use CPU-only versions of ML libraries
   - Set `CUDA_VISIBLE_DEVICES=""` to force CPU mode

2. **Memory Issues**
   - Reduce parallel build workers: `colcon build --parallel-workers 2`
   - Increase VM memory allocation

3. **Network Issues**
   - Check firewall settings: `sudo ufw status`
   - Verify ROS2 domain ID: `echo $ROS_DOMAIN_ID`

4. **Display Issues**
   - For headless: Set `DISPLAY=:99` and use Xvfb
   - For X11: `ssh -X user@vm-ip`

## Backup and Migration

### 1. Create VM Snapshot
```bash
# Before major changes, create VM snapshot
gcloud compute disks snapshot DISK_NAME --snapshot-names=r2-system-backup-$(date +%Y%m%d)
```

### 2. Export Workspace
```bash
# Create portable backup
tar -czf r2_workspace_backup.tar.gz ~/ros2_workspace \
  --exclude="*/build" \
  --exclude="*/install" \
  --exclude="*/log"
```

## Security Considerations

### 1. VM Security
- Use SSH keys instead of passwords
- Configure firewall rules
- Regular system updates

### 2. ROS2 Security
```bash
# Enable ROS2 security (optional)
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

## Cost Optimization

### 1. VM Scheduling
- Use preemptible instances for development
- Schedule automatic shutdown during non-work hours
- Use sustained use discounts

### 2. Storage Optimization
- Use standard persistent disks for development
- Regular cleanup of build artifacts
- Compress large datasets

## Support and Resources

- **Documentation**: Check repository README files
- **Issues**: Create GitHub issues for bugs
- **Community**: ROS2 community forums
- **Google Cloud**: GCP documentation and support

---

**Note**: This system is designed for development and simulation. For production robot control, physical hardware integration will be required.