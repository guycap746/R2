# R2 RoArm M3 System Deployment Package

This folder contains everything needed to rebuild the complete R2 RoArm M3 system on a new machine, specifically Google Cloud VM instances.

## 📦 Package Contents

| File | Description | Size |
|------|-------------|------|
| `setup_google_vm.sh` | **Automated installation script** | 14KB |
| `GOOGLE_VM_SETUP_GUIDE.md` | Complete setup documentation | 7KB |
| `requirements.txt` | Python package dependencies (187 packages) | 3.4KB |
| `system_packages.txt` | System package list for recreation | 73KB |
| `apt_packages_detailed.txt` | Detailed apt package information | 177KB |
| `system_info.txt` | Original system information | 140B |

## 🚀 Quick Deploy (Recommended)

### One-Command Setup
```bash
curl -sSL https://raw.githubusercontent.com/guycap746/R2/main/deployment/setup_google_vm.sh | bash
```

### Alternative: Download & Run
```bash
# Download deployment package
wget https://github.com/guycap746/R2/archive/main.zip
unzip main.zip
cd R2-main/deployment

# Run setup script
chmod +x setup_google_vm.sh
./setup_google_vm.sh
```

## 🖥️ Target Environment

### VM Requirements
- **OS**: Ubuntu 22.04 LTS
- **CPU**: 4+ vCPUs (8+ recommended)
- **RAM**: 16+ GB (32+ GB recommended)
- **Storage**: 100+ GB SSD (200+ GB recommended)
- **Network**: Internet connection required

### Cloud Platforms Tested
- ✅ Google Cloud Platform (GCE)
- ✅ Amazon Web Services (EC2)
- ✅ Microsoft Azure
- ✅ Local VirtualBox/VMware

## ⏱️ Installation Timeline

| Phase | Duration | Description |
|-------|----------|-------------|
| System Updates | 2-5 min | Update base packages |
| ROS2 Installation | 5-10 min | Install ROS2 Humble + MoveIt |
| Python Dependencies | 3-8 min | Install 187 Python packages |
| Repository Clone | 1-2 min | Clone R2 workspace |
| Workspace Build | 5-15 min | Build all ROS2 packages |
| **Total** | **15-30 min** | Complete system ready |

## 🎯 What Gets Installed

### Core Systems
- **ROS2 Humble Desktop Full** - Complete robotics framework
- **MoveIt2** - Motion planning framework
- **Gazebo** - Physics simulation
- **RViz2** - 3D visualization

### R2 Specific Components
- **RoArm M3 packages** - Robot description and control
- **AnyGrasp integration** - AI-powered grasping
- **LeRobot integration** - Machine learning framework
- **Isaac Sim support** - NVIDIA simulation platform
- **Foxglove panels** - Advanced visualization

### Development Tools
- **VS Code** (optional) - IDE
- **Docker** - Containerization
- **Git** - Version control
- **System monitoring tools**

## 🔧 Post-Installation Commands

### Quick Start
```bash
# Start R2 system
./start_r2_system.sh

# Update system
./update_r2_system.sh
```

### Common Operations
```bash
# Build workspace
r2_build

# Start simulation
r2_gazebo

# Start motion planning
r2_moveit

# Start visualization
r2_rviz
```

## 🏗️ Simulation Mode Configuration

The system automatically configures for VM/simulation mode:

```bash
export ROS_DOMAIN_ID=42
export ROBOT_SIMULATION=true
export USE_FAKE_HARDWARE=true
export ROS_LOCALHOST_ONLY=1
```

## 📚 Documentation

### Setup Guide
Read `GOOGLE_VM_SETUP_GUIDE.md` for:
- Detailed installation steps
- Troubleshooting guide
- Performance optimization
- Security considerations

### Original System Info
- `system_info.txt` - Original system specifications
- `system_packages.txt` - Complete package list
- `requirements.txt` - Python dependencies

## 🔍 Verification Steps

After installation, verify with:

```bash
# Test ROS2
ros2 run demo_nodes_cpp talker

# Test workspace
ros2 pkg list | grep roarm

# Test simulation
ros2 launch xarm_6_gazebo xarm_6_gazebo.launch.py
```

## 🆘 Troubleshooting

### Common Issues

1. **Insufficient Memory**
   ```bash
   # Reduce build parallelism
   export MAKEFLAGS="-j2"
   ```

2. **Network Issues**
   ```bash
   # Check connectivity
   ping google.com
   ```

3. **Permission Issues**
   ```bash
   # Ensure not running as root
   whoami  # Should NOT return 'root'
   ```

### Getting Help

- **GitHub Issues**: [Create issue](https://github.com/guycap746/R2/issues)
- **Setup Guide**: Read `GOOGLE_VM_SETUP_GUIDE.md`
- **ROS2 Community**: [ROS Discourse](https://discourse.ros.org/)

## 🔄 Updates

To update an existing installation:

```bash
cd ~/ros2_workspace
git pull origin main
./deployment/setup_google_vm.sh  # Re-run setup if needed
```

## 📄 License

This deployment package is part of the R2 RoArm M3 project. See main repository for license information.

---

**Created**: $(date)  
**Source System**: Linux 5.15.0-141-generic  
**Target**: Ubuntu 22.04 LTS + ROS2 Humble  
**Maintainer**: R2 Development Team

🤖 **Ready to deploy your R2 system!**