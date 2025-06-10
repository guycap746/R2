# ChArUco Camera Calibration Workflow

This document describes the complete camera calibration workflow for the RoArm M3 dual camera grasping system using ChArUco boards.

## Overview

The calibration system provides:
1. **Camera Intrinsic Calibration**: Calibrate individual camera parameters
2. **Stereo Calibration**: Calibrate relative positions between dual cameras  
3. **Hand-Eye Calibration**: Calibrate camera position relative to robot end-effector
4. **Automated Workflow**: Interactive scripts to guide the process

## ChArUco Board Advantages

ChArUco boards combine the benefits of:
- **ArUco markers**: Robust detection and unique IDs
- **Chessboard patterns**: Sub-pixel corner accuracy
- **Partial occlusion tolerance**: Works even if parts are hidden
- **Pose estimation**: Direct 6DOF pose measurement

## Quick Start

### 1. Basic Camera Calibration

```bash
# Launch camera calibration
ros2 launch roarm_anygrasp_integration charuco_calibration.launch.py

# Run interactive workflow
ros2 run roarm_anygrasp_integration calibration_workflow.py
```

### 2. Dual Camera Calibration

```bash
# Launch both cameras
ros2 launch roarm_anygrasp_integration charuco_calibration.launch.py \
    enable_primary_camera:=true \
    enable_side_camera:=true

# Run calibration workflow
ros2 run roarm_anygrasp_integration calibration_workflow.py
```

### 3. Hand-Eye Calibration

```bash
# Launch full system with robot
ros2 launch roarm_anygrasp_integration charuco_calibration.launch.py \
    enable_primary_camera:=true \
    enable_hand_eye_calibration:=true \
    enable_robot:=true \
    use_fake_hardware:=false

# Run calibration workflow
ros2 run roarm_anygrasp_integration calibration_workflow.py
```

## Detailed Workflow

### Step 1: Generate Calibration Board

The system automatically generates a ChArUco board with these default parameters:
- **Board size**: 7x5 squares
- **Square size**: 40mm (4cm)  
- **Marker size**: 32mm (80% of square)
- **Dictionary**: DICT_4X4_50

```bash
# Generate board manually
ros2 service call /charuco_calibration/generate_board std_msgs/srv/String "{}"
```

**Printing Instructions:**
1. Print `/tmp/charuco_calibration/charuco_board.png` on A4 paper
2. Use "Actual Size" (100% scale) - no scaling!
3. Mount on rigid, flat surface (foam board, cardboard)
4. Ensure good lighting, avoid reflections

### Step 2: Collect Calibration Samples

The calibration requires 10-30 images with the board at different:
- **Positions**: Different areas of camera view
- **Orientations**: Various tilts and rotations  
- **Distances**: From close to far within focus range
- **Angles**: Different viewing angles

```bash
# Collect samples manually
ros2 service call /charuco_calibration/collect_sample std_msgs/srv/String "{}"

# View detection status
ros2 topic echo /charuco_calibration/status
ros2 topic echo /charuco_calibration/detection_image
```

**Tips for good samples:**
- Keep board fully visible
- Avoid motion blur - hold steady
- Good lighting without glare
- Cover different areas of image
- Include corner and edge positions

### Step 3: Compute Camera Calibration

Once enough samples are collected:

```bash
# Run calibration
ros2 service call /charuco_calibration/calibrate_cameras roarm_anygrasp_integration/srv/CalibrateCamera "{}"
```

**Results saved to**: `/tmp/charuco_calibration/results/`
- `calibration_results_YYYYMMDD_HHMMSS.json`: Complete calibration data
- Camera matrices, distortion coefficients, reprojection errors

### Step 4: Hand-Eye Calibration (Optional)

For robot-camera coordination, calibrate the camera position relative to the robot end-effector.

```bash
# Start hand-eye calibration
ros2 service call /hand_eye_calibration/start_calibration std_msgs/srv/String "{}"

# Manual pose collection
ros2 service call /hand_eye_calibration/collect_pose std_msgs/srv/String "{}"

# Compute hand-eye transform  
ros2 service call /hand_eye_calibration/compute_calibration std_msgs/srv/String "{}"
```

**Process:**
1. Place board in robot workspace
2. Robot moves to 8-10 different poses automatically
3. At each pose, system captures robot position and board detection
4. Computes camera-to-end-effector transform
5. Publishes TF transform for use in grasping

## Configuration Parameters

### ChArUco Board Parameters

```yaml
board_squares_x: 7          # Squares in X direction
board_squares_y: 5          # Squares in Y direction  
square_length: 0.04         # Square size in meters (4cm)
marker_length: 0.032        # Marker size in meters (3.2cm)
dictionary_id: 0            # DICT_4X4_50
```

### Calibration Parameters

```yaml
min_samples: 10             # Minimum calibration images
max_samples: 30             # Maximum calibration images
collection_interval: 2.0    # Auto-collection interval (seconds)
auto_collect: false         # Enable automatic collection
save_images: true           # Save calibration images
```

### Hand-Eye Parameters

```yaml
calibration_poses_count: 8  # Number of robot poses
workspace_center_x: 0.25    # Workspace center X (meters)
workspace_center_y: 0.0     # Workspace center Y (meters)  
workspace_center_z: 0.15    # Workspace center Z (meters)
workspace_radius: 0.15      # Workspace radius (meters)
```

## Service API

### ChArUco Calibration Services

- `/charuco_calibration/generate_board` - Generate calibration board
- `/charuco_calibration/collect_sample` - Collect calibration sample
- `/charuco_calibration/calibrate_cameras` - Compute calibration

### Hand-Eye Calibration Services

- `/hand_eye_calibration/start_calibration` - Start automatic calibration
- `/hand_eye_calibration/collect_pose` - Collect single pose
- `/hand_eye_calibration/compute_calibration` - Compute hand-eye transform

## Topics

### Status Topics

- `/charuco_calibration/status` - Calibration status messages
- `/charuco_calibration/detection_image` - Annotated detection images
- `/hand_eye_calibration/status` - Hand-eye calibration status

### Visualization Topics

- `/charuco_calibration/board_poses` - Detected board poses
- `/hand_eye_calibration/target_poses` - Target robot poses

## Troubleshooting

### Board Not Detected

- **Check lighting**: Avoid shadows and reflections
- **Check focus**: Ensure board is in focus range
- **Check visibility**: Entire board must be visible
- **Check print quality**: Ensure crisp, high-contrast print

### Poor Calibration Results

- **High reprojection error** (>1.0 pixel):
  - Collect more samples at different positions
  - Ensure board is flat and rigid
  - Check camera stability during capture
  
- **Inconsistent results**:
  - Use more calibration poses
  - Ensure stable lighting conditions
  - Check for camera synchronization issues

### Hand-Eye Calibration Issues

- **Robot movement fails**:
  - Check MoveIt configuration
  - Verify workspace boundaries
  - Ensure collision avoidance
  
- **Board not visible during robot movement**:
  - Adjust workspace parameters
  - Reposition board or camera
  - Modify calibration poses

## File Structure

```
/tmp/charuco_calibration/
├── charuco_board.png              # Generated calibration board
├── board_info.json                # Board parameters
├── images/                        # Collected calibration images
│   ├── sample_001_primary.jpg
│   ├── sample_001_primary_annotated.jpg
│   └── ...
└── results/                       # Calibration results
    └── calibration_results_YYYYMMDD_HHMMSS.json

/tmp/hand_eye_calibration/
├── poses/                         # Robot poses and detections
│   ├── pose_001.json
│   └── ...
├── images/                        # Images at each pose
│   ├── pose_001.jpg
│   └── ...
└── hand_eye_calibration_YYYYMMDD_HHMMSS.json
```

## Integration with Grasping System

After calibration, the system automatically:

1. **Uses calibrated camera parameters** for accurate depth and pose estimation
2. **Applies hand-eye transform** to convert detections to robot coordinates  
3. **Publishes TF transforms** for coordinate frame alignment
4. **Improves grasp accuracy** through proper camera-robot coordination

The calibrated transforms are automatically loaded and used by the grasping pipeline.

## Advanced Usage

### Custom Board Sizes

```bash
ros2 launch roarm_anygrasp_integration charuco_calibration.launch.py \
    board_squares_x:=9 \
    board_squares_y:=6 \
    square_length:=0.03 \
    marker_length:=0.024
```

### Batch Processing

```bash
# Collect many samples automatically
ros2 param set /charuco_calibration_node auto_collect true
ros2 param set /charuco_calibration_node collection_interval 3.0
```

### Custom Workspace

```bash
ros2 param set /hand_eye_calibration_node workspace_center_x 0.3
ros2 param set /hand_eye_calibration_node workspace_center_y -0.1
ros2 param set /hand_eye_calibration_node workspace_radius 0.2
```

## Maintenance

### Recalibration Schedule

- **Camera intrinsics**: Every 6 months or after camera hardware changes
- **Hand-eye calibration**: After robot reconfiguration or camera remounting
- **Verification**: Monthly accuracy checks with known objects

### Calibration Verification

```bash
# Test calibration accuracy
ros2 launch roarm_anygrasp_integration charuco_calibration.launch.py
# Place board at known position and verify detection accuracy
```