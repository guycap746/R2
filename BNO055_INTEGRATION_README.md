# BNO055 Wrist IMU Integration for RoArm M3

This document describes the complete integration of the Bosch BNO055 9-DOF smart sensor with the RoArm M3 robotic grasping system.

## 🎯 Overview

The BNO055 IMU is mounted on the wrist alongside the Intel RealSense D405 camera to provide:
- **Enhanced motion tracking** - Real-time orientation and movement detection
- **Camera stabilization** - Compensate for hand tremor and movement
- **Gravity compensation** - Accurate object orientation relative to gravity
- **Motion analysis** - Detect and analyze manipulation movements

## 📦 Package Structure

### `wrist_imu_integration` Package
```
wrist_imu_integration/
├── config/
│   └── bno055_wrist_config.yaml      # BNO055 configuration
├── launch/
│   ├── wrist_imu.launch.py           # Standalone IMU launch
│   └── d405_with_imu.launch.py       # Combined camera + IMU
├── scripts/
│   ├── imu_camera_fusion.py          # Data fusion node
│   └── wrist_imu_calibration.py      # Calibration helper
└── CMakeLists.txt
```

## 🔧 Hardware Setup

### BNO055 Connection
- **Interface**: USB-to-UART bridge
- **Default Port**: `/dev/ttyUSB0`
- **Baud Rate**: 115200
- **Power**: 3.3V from USB adapter

### Physical Mounting
- **Location**: Wrist-mounted with D405 camera
- **Orientation**: Aligned with camera optical frame
- **Offset**: ~2cm from camera center (configurable)

### USB Device Rules
```bash
# Add udev rule for consistent device naming
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6001", SYMLINK+="ttyUSB_IMU"' | sudo tee /etc/udev/rules.d/99-bno055.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 🚀 Quick Start

### 1. Install Dependencies
```bash
# BNO055 driver is already installed
sudo apt install ros-humble-bno055

# I2C tools (if using I2C interface)
sudo apt install i2c-tools python3-smbus
```

### 2. Basic IMU Launch
```bash
# Source environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch IMU only
ros2 launch wrist_imu_integration wrist_imu.launch.py

# Launch camera + IMU together
ros2 launch wrist_imu_integration d405_with_imu.launch.py
```

### 3. Verify IMU Data
```bash
# Check IMU topics
ros2 topic list | grep wrist_imu

# Monitor IMU data
ros2 topic echo /wrist_imu/imu

# Check calibration status
ros2 topic echo /wrist_imu/calib_status
```

## ⚙️ Configuration

### BNO055 Parameters (`config/bno055_wrist_config.yaml`)

**Connection Settings:**
```yaml
connection_type: "uart"        # USB-to-UART
uart_port: "/dev/ttyUSB0"     # USB port
uart_baudrate: "115200"       # Baud rate
```

**Data Acquisition:**
```yaml
data_query_frequency: 50      # 50Hz for motion tracking
frame_id: "wrist_imu_link"    # TF frame
operation_mode: 12            # NDOF mode (full 9DOF)
```

**Sensor Variance (tuned for wrist applications):**
```yaml
variance_acc: [0.01, 0.01, 0.01]          # Tight for stability
variance_angular_vel: [0.02, 0.02, 0.02]  # Precise orientation
variance_orientation: [0.01, 0.01, 0.01]  # High precision
```

### Launch Parameters

**wrist_imu.launch.py:**
- `usb_port` - IMU USB port (default: `/dev/ttyUSB0`)
- `frequency` - Data frequency in Hz (default: `50`)
- `enable_calibration` - Auto-calibration (default: `true`)
- `enable_fusion` - IMU-camera fusion (default: `true`)

**Example:**
```bash
ros2 launch wrist_imu_integration wrist_imu.launch.py \
    usb_port:=/dev/ttyUSB1 \
    frequency:=100 \
    enable_calibration:=true
```

## 🎛️ TF Frame Integration

### Frame Hierarchy
```
base_link
└── camera_mount_link
    ├── d405_link
    │   └── d405_color_optical_frame
    └── wrist_imu_link
```

### Transform Configuration
- **IMU to Camera**: Static transform represents physical mounting
- **Base to Mount**: Represents robot arm position
- **Dynamic Updates**: Real-time orientation from IMU

### TF Publishers
```bash
# Static transforms (configured in launch files)
- base_link → camera_mount_link       # Robot arm position
- camera_mount_link → d405_link       # Camera mounting
- wrist_imu_link → d405_color_optical_frame  # IMU-camera offset

# Dynamic transforms (from IMU data)
- wrist_imu_link orientation updates based on BNO055 readings
```

## 📊 Published Topics

### IMU Data Topics
```bash
/wrist_imu/imu                    # sensor_msgs/Imu
/wrist_imu/mag                    # sensor_msgs/MagneticField  
/wrist_imu/temp                   # sensor_msgs/Temperature
/wrist_imu/calib_status           # std_msgs/UInt8
```

### Fusion and Analysis Topics
```bash
/wrist_imu/imu_filtered           # sensor_msgs/Imu (filtered)
/camera/stabilization_offset     # geometry_msgs/Vector3Stamped
/camera/motion_detected           # std_msgs/Bool
/camera/stabilized_pose           # geometry_msgs/PoseStamped
```

### Calibration Topics
```bash
/wrist_imu/calibration_guidance   # std_msgs/String
/wrist_imu/calibration_complete   # std_msgs/String
```

## 🔧 Calibration Workflow

### Automatic Calibration
The system provides guided automatic calibration:

1. **Gyroscope** (10 seconds): Keep IMU perfectly still
2. **Accelerometer** (30 seconds): Place in 6 orientations
3. **Magnetometer** (20 seconds): Figure-8 movements  
4. **System** (15 seconds): Smooth combined movements

### Manual Calibration
```bash
# Start calibration manually
ros2 topic pub /wrist_imu/start_calibration std_msgs/Empty

# Monitor calibration progress
ros2 topic echo /wrist_imu/calibration_guidance

# Check completion
ros2 topic echo /wrist_imu/calibration_complete
```

### Calibration Status
BNO055 provides 0-3 calibration levels for each sensor:
- **0**: Uncalibrated
- **1**: Minimally calibrated
- **2**: Moderately calibrated  
- **3**: Fully calibrated

### Calibration Storage
```bash
# Calibration data saved to:
~/.ros2/wrist_imu_calibration.json

# Contains:
- Timestamp
- Calibration status for each sensor
- Reference IMU readings
```

## 🎯 IMU-Camera Data Fusion

### Features
- **Orientation Filtering**: SLERP quaternion smoothing
- **Motion Detection**: Angular velocity thresholding
- **Stabilization**: Real-time camera pose correction
- **Gravity Compensation**: Separate gravity from motion

### Fusion Node (`imu_camera_fusion.py`)

**Parameters:**
```yaml
camera_frame: 'd405_color_optical_frame'
imu_frame: 'wrist_imu_link'
fusion_frequency: 50.0           # Processing rate
stabilization_alpha: 0.8        # Low-pass filter strength
motion_threshold: 0.1           # Motion detection (rad/s)
```

**Published Data:**
- Filtered IMU readings with reduced noise
- Camera stabilization offsets
- Motion detection status
- Stabilized camera pose

## 🧪 Testing and Validation

### Quick Test
```bash
# Run system test with IMU
./run_tests.sh quick

# Test hardware simulation  
./run_tests.sh hardware
```

### IMU-Specific Tests
```bash
# Test IMU driver
ros2 run bno055 bno055 --ros-args --params-file config/bno055_wrist_config.yaml

# Test fusion node
ros2 run wrist_imu_integration imu_camera_fusion.py

# Test calibration
ros2 run wrist_imu_integration wrist_imu_calibration.py
```

### Dummy Hardware Simulation
The test system includes dummy IMU simulation:
- Realistic wrist motion patterns
- Gradual calibration progression
- Proper IMU message formatting
- Calibration status simulation

## 🔍 Troubleshooting

### Common Issues

**1. IMU Not Detected**
```bash
# Check USB connection
lsusb | grep -i ftdi

# Check device permissions
ls -l /dev/ttyUSB*
sudo usermod -a -G dialout $USER  # Add user to dialout group
```

**2. Calibration Problems**
```bash
# Reset calibration
ros2 service call /wrist_imu/reset_calibration std_srvs/Empty

# Check magnetic interference
ros2 topic echo /wrist_imu/mag  # Look for stable readings
```

**3. TF Transform Issues**
```bash
# Check TF tree
ros2 run tf2_tools view_frames.py

# Verify transforms
ros2 run tf2_ros tf2_echo wrist_imu_link d405_color_optical_frame
```

**4. Data Fusion Problems**
```bash
# Check IMU data rate
ros2 topic hz /wrist_imu/imu

# Verify camera data
ros2 topic hz /d405/color/camera_info

# Check synchronization
ros2 topic echo --field header.stamp /wrist_imu/imu
```

### Debug Mode
```bash
# Enable debug logging
ros2 launch wrist_imu_integration wrist_imu.launch.py \
    --ros-args --log-level debug
```

## 🎮 Usage Examples

### Basic IMU Monitoring
```bash
# Monitor orientation (quaternion)
ros2 topic echo /wrist_imu/imu --field orientation

# Monitor angular velocity
ros2 topic echo /wrist_imu/imu --field angular_velocity

# Check motion detection
ros2 topic echo /camera/motion_detected
```

### Camera Stabilization
```bash
# Get stabilization offset
ros2 topic echo /camera/stabilization_offset

# Monitor stabilized pose
ros2 topic echo /camera/stabilized_pose
```

### Calibration Monitoring
```bash
# Watch calibration progress
ros2 topic echo /wrist_imu/calibration_guidance

# Check calibration levels
ros2 topic echo /wrist_imu/calib_status
```

## 🔄 Integration with Grasping Pipeline

### Motion-Aware Grasping
- **Pre-grasp**: Detect hand stability before grasp execution
- **During Grasp**: Monitor grip stability and adjust
- **Post-grasp**: Verify object secure in hand

### Camera Stabilization
- **Real-time Correction**: Compensate for hand tremor
- **Motion Blur Reduction**: Skip frames during fast motion
- **Object Tracking**: Maintain target lock during movement

### Gravity Awareness
- **Object Orientation**: Determine "up" direction for grasping
- **Stability Analysis**: Predict object behavior after grasp
- **Manipulation Planning**: Account for gravity in motion plans

## 📈 Performance Specifications

### BNO055 Specifications
- **Update Rate**: Up to 100Hz
- **Orientation Accuracy**: ±1° (fully calibrated)
- **Angular Rate**: ±2000°/s range
- **Acceleration**: ±16g range
- **Magnetic Field**: ±1300µT range

### System Performance
- **Latency**: <20ms IMU to stabilization output
- **Data Rate**: 50Hz recommended for motion tracking
- **Calibration Time**: 30-60 seconds for full calibration
- **Stability**: <0.1° drift over 10 minutes

## 🛠️ Advanced Configuration

### Custom Mounting Orientation
```yaml
# Adjust for different mounting angles
placement_axis_remap: "P2"  # See BNO055 datasheet for options
```

### High-Frequency Applications
```yaml
# For fast motion tracking
data_query_frequency: 100
fusion_frequency: 100.0
motion_threshold: 0.05  # More sensitive
```

### Low-Power Mode
```yaml
# Reduce update rate for battery operation
data_query_frequency: 10
operation_mode: 8  # IMU mode (no magnetometer)
```

## 📚 References

- **BNO055 Datasheet**: [Bosch Sensortec BNO055](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- **ROS2 BNO055 Driver**: [ros-humble-bno055](https://github.com/flynneva/bno055)
- **IMU Calibration Guide**: [BNO055 Calibration](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration)
- **TF2 Documentation**: [ROS2 TF2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/)

## 🤝 Contributing

When extending the IMU integration:

1. **Test with Simulation**: Use dummy hardware nodes first
2. **Update Calibration**: Modify calibration workflow if needed
3. **Document Changes**: Update this README and configuration files
4. **Verify TF Tree**: Ensure transforms remain consistent
5. **Run Tests**: Use test system to validate changes

The BNO055 integration provides a solid foundation for enhanced robotic manipulation with motion awareness and camera stabilization.