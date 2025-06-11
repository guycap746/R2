# 🚀 R2 System Deployment Instructions

## Quick Start (Copy & Paste Ready)

### For Google Cloud VM:
```bash
# 1. Create Ubuntu 22.04 VM (4+ vCPUs, 16+ GB RAM)
# 2. SSH into your VM
# 3. Run this single command:

curl -sSL https://raw.githubusercontent.com/guycap746/R2/main/deployment/setup_google_vm.sh | bash
```

### For Any Linux System:
```bash
# Option A: Direct download
wget https://github.com/guycap746/R2/archive/main.zip
unzip main.zip
cd R2-main/deployment
chmod +x setup_google_vm.sh
./setup_google_vm.sh

# Option B: Git clone
git clone https://github.com/guycap746/R2.git
cd R2/deployment
./setup_google_vm.sh
```

## ⏱️ What to Expect

1. **Installation time**: 15-30 minutes
2. **Internet required**: Downloads ~2GB of packages
3. **No interaction needed**: Fully automated
4. **Final result**: Complete R2 system ready to use

## ✅ Post-Installation

After installation completes:

```bash
# Restart terminal or reload environment
source ~/.bashrc

# Test the system
./start_r2_system.sh

# Available commands:
r2_build    # Build workspace
r2_gazebo   # Start simulation
r2_moveit   # Start motion planning
r2_rviz     # Start visualization
```

## 📋 Prerequisites Checklist

- [ ] Ubuntu 22.04 LTS (or compatible)
- [ ] Regular user account (not root)
- [ ] Internet connection
- [ ] 4+ GB RAM minimum (16+ GB recommended)
- [ ] 20+ GB free disk space

## 🔧 VM Specifications

### Minimum (Development):
- **vCPUs**: 4
- **RAM**: 16 GB
- **Storage**: 100 GB SSD
- **Network**: Standard

### Recommended (Training):
- **vCPUs**: 8
- **RAM**: 32 GB  
- **Storage**: 200 GB SSD
- **GPU**: NVIDIA T4+ (optional)

## 🎯 Success Indicators

You'll know it worked when you see:
```
🎉 R2 RoArm M3 System successfully installed on Google VM!

✅ Installation Summary:
   • ROS2 Humble: ✅ Installed
   • MoveIt2: ✅ Installed  
   • Python packages: ✅ Installed
   • R2 workspace: ✅ Built and ready
   • Environment: ✅ Configured for VM simulation
```

## 🆘 If Something Goes Wrong

1. **Check the error message** - Script provides detailed feedback
2. **Ensure prerequisites** - Ubuntu 22.04, not root user, internet
3. **Try manual steps** - Follow `GOOGLE_VM_SETUP_GUIDE.md`
4. **Create GitHub issue** - Include error output

## 📦 What This Package Contains

- `setup_google_vm.sh` - Main installation script
- `requirements.txt` - Python dependencies
- `system_packages.txt` - System packages
- `GOOGLE_VM_SETUP_GUIDE.md` - Detailed documentation
- `README.md` - Package overview

---

**That's it!** One command gets you a complete R2 robotics system. 🤖