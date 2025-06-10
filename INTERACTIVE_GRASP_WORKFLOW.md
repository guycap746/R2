# Interactive AnyGrasp Workflow - User Selection Guide

This document describes the enhanced AnyGrasp integration that requires user confirmation before executing grasps, allowing you to review and select from the top 5 grasp candidates.

## Overview

The interactive workflow provides a **human-in-the-loop** approach to grasp execution:

1. **Detection**: AnyGrasp analyzes the scene and identifies potential grasps
2. **Filtering**: System filters and ranks candidates by confidence and quality
3. **Selection**: User reviews top 5 candidates and selects the best one
4. **Confirmation**: Optional confirmation dialog before execution
5. **Execution**: Coordinated robot movement and grasping sequence

## Architecture Components

### 🤖 Interactive AnyGrasp Node
- **Package**: `roarm_anygrasp_integration`
- **Executable**: `anygrasp_interactive_node`
- **Purpose**: Handles grasp detection with user selection pause

**Key Features:**
- Manual detection trigger (no auto-detection)
- Configurable candidate filtering
- Top-N candidate selection
- Service-based interaction

### 🎯 Grasp Coordinator
- **Package**: `roarm_anygrasp_integration` 
- **Executable**: `grasp_coordinator`
- **Purpose**: Manages complete workflow and robot execution

**Key Features:**
- Workflow state management
- User selection handling
- MoveIt integration
- Safety monitoring

### 🖥️ Foxglove Interactive Panel
- **File**: `/foxglove_panels/anygrasp_interactive_panel.ts`
- **Purpose**: Web-based user interface for grasp selection

**Key Features:**
- Visual candidate selection
- Real-time status monitoring
- Confirmation dialogs
- Emergency stop controls

## Service Interfaces

### Get Grasp Candidates
```bash
# Service: /grasp_coordinator/start_grasp_workflow
# Type: roarm_anygrasp_integration/GetGraspCandidates

# Request:
num_candidates: 5        # Number of top candidates to return
min_confidence: 0.6      # Minimum confidence threshold

# Response:
grasp_poses: [...]       # Array of candidate poses
confidence_scores: [...]  # Confidence values (0.0-1.0)
grasp_widths: [...]      # Gripper width requirements
quality_scores: [...]    # Overall quality scores
detection_status: "..."  # Status message
total_grasps_detected: N # Total grasps found
```

### Execute Selected Grasp
```bash
# Service: /grasp_coordinator/execute_selected
# Type: roarm_anygrasp_integration/SelectGrasp

# Request:
selected_grasp_index: 2  # Index of chosen candidate (0-4)

# Response:
success: true/false      # Execution result
message: "..."           # Status/error message
selected_pose: {...}     # Confirmed grasp pose
confidence_score: 0.85   # Selected grasp confidence
```

## Usage Instructions

### 1. Launch the Interactive System

```bash
# Start the complete interactive system
ros2 launch roarm_moveit roarm_anygrasp_demo.launch.py

# Or launch components separately:
ros2 run roarm_anygrasp_integration anygrasp_interactive_node
ros2 run roarm_anygrasp_integration grasp_coordinator
```

### 2. Access Foxglove Interface

**Option A: Foxglove Studio**
1. Download and install [Foxglove Studio](https://foxglove.dev/download)
2. Connect to: `ws://YOUR_ROBOT_IP:8765`
3. Import layout: `/foxglove_configs/roarm_m3_layout.json`

**Option B: Web Browser**
1. Go to [app.foxglove.dev](https://app.foxglove.dev)
2. Connect to: `ws://YOUR_ROBOT_IP:8765`

### 3. Interactive Workflow Steps

#### Step 1: Configure Settings
- **Max Candidates**: Choose 1-5 candidates to review
- **Min Confidence**: Set minimum confidence threshold (0.0-1.0)
- **Show Confirmation**: Enable/disable confirmation dialog
- **Auto-execute**: Automatically execute after selection

#### Step 2: Start Detection
1. Click **"🔍 Start Workflow"** button
2. Wait for grasp detection to complete
3. Monitor status: `DETECTING` → `SELECTING`

#### Step 3: Review Candidates
- View ranked list of top candidates
- Each candidate shows:
  - **Confidence score** (percentage)
  - **Grasp width** requirement
  - **Quality score** 
  - **3D position** coordinates

#### Step 4: Select Grasp
1. Click on preferred candidate in the list
2. Review selection highlighting
3. Click **"🚀 Execute Selected Grasp"**

#### Step 5: Confirm Execution (Optional)
- If confirmation enabled, review details:
  - Selected candidate information
  - Final position and orientation
- Click **"✅ Execute"** or **"❌ Cancel"**

#### Step 6: Monitor Execution
- Watch progress: `EXECUTING` status
- Sequence includes:
  1. Gripper opening
  2. Approach movement
  3. Grasp positioning
  4. Gripper closing
  5. Object lifting

### 4. Emergency Controls

**Emergency Stop**: Click **"🛑 STOP"** button at any time
- Immediately halts all robot motion
- Opens gripper for safety
- Resets workflow state

**Workflow Reset**: Click **"🔄 Reset"** to start over
- Clears current candidates
- Returns to idle state
- Preserves configuration settings

## Command Line Interface

### Manual Service Calls

**Start Detection Workflow:**
```bash
ros2 service call /grasp_coordinator/start_grasp_workflow \
  roarm_anygrasp_integration/srv/GetGraspCandidates \
  "{num_candidates: 5, min_confidence: 0.6}"
```

**Execute Selected Grasp:**
```bash
ros2 service call /grasp_coordinator/execute_selected \
  roarm_anygrasp_integration/srv/SelectGrasp \
  "{selected_grasp_index: 2}"
```

### Monitor Status
```bash
# Monitor workflow status
ros2 topic echo /grasp_coordinator/status

# Monitor AnyGrasp status  
ros2 topic echo /anygrasp/status

# View detected candidates
ros2 topic echo /anygrasp/top_candidates
```

## Configuration Parameters

### AnyGrasp Interactive Node
```yaml
anygrasp_interactive_node:
  ros__parameters:
    auto_detect: false                    # Manual trigger only
    detection_confidence_threshold: 0.6   # Minimum confidence
    max_candidates: 5                     # Top candidates to return
    min_grasp_width: 0.01                # Minimum gripper width (m)
    max_grasp_width: 0.12                # Maximum gripper width (m)
```

### Grasp Coordinator
```yaml
grasp_coordinator:
  ros__parameters:
    approach_distance: 0.1        # Distance to approach from above (m)
    lift_distance: 0.05          # Height to lift object (m)
    gripper_open_position: 0.0   # Open gripper position (rad)
    gripper_close_position: 1.2  # Closed gripper position (rad)
    movement_speed: 0.1          # Robot movement speed (m/s)
    max_candidates: 5            # Maximum candidates to consider
    min_confidence: 0.6          # Minimum confidence threshold
```

## Troubleshooting

### Common Issues

**1. No Candidates Detected**
- Check camera is streaming: `ros2 topic echo /camera/depth/color/points`
- Verify objects are in view and well-lit
- Lower confidence threshold in settings
- Check AnyGrasp license is valid

**2. Service Call Failures**
- Verify nodes are running: `ros2 node list`
- Check service availability: `ros2 service list | grep grasp`
- Monitor node logs: `ros2 node info /grasp_coordinator`

**3. Execution Failures**
- Check MoveIt planning: `ros2 topic echo /move_group/display_planned_path`
- Verify robot driver is connected: `ros2 topic echo /roarm_driver/status`
- Ensure target poses are reachable

**4. Foxglove Connection Issues**
- Verify bridge is running: `ros2 node list | grep foxglove`
- Check port availability: `netstat -ln | grep 8765`
- Try different IP address or port

### Debug Commands

```bash
# Check all grasp-related services
ros2 service list | grep -E "(grasp|anygrasp)"

# Monitor workflow state
ros2 topic echo /grasp_coordinator/status

# View current candidates
ros2 topic echo /anygrasp/top_candidates

# Check robot state
ros2 topic echo /joint_states

# Test individual components
ros2 run roarm_anygrasp_integration anygrasp_interactive_node --ros-args --log-level debug
```

## Workflow States

| State | Description | User Actions Available |
|-------|-------------|------------------------|
| `idle` | Ready to start | Configure settings, start workflow |
| `detecting` | AnyGrasp running | Wait, emergency stop |
| `selecting` | Candidates ready | Select grasp, emergency stop |
| `confirming` | Awaiting confirmation | Confirm, cancel |
| `executing` | Robot moving | Monitor, emergency stop |
| `completed` | Success | Start new workflow |
| `failed` | Error occurred | Check logs, reset |

## Safety Considerations

### Automatic Safety Features
- **Emergency stop** available at all times
- **Gripper opening** on stop or error
- **Collision checking** during movement
- **Reachability validation** before execution

### Manual Safety Practices
- **Clear workspace** before starting
- **Monitor execution** closely
- **Keep emergency stop** easily accessible
- **Verify grasp selection** before confirming

### Risk Mitigation
- **Start with high confidence thresholds**
- **Test in simulation first**
- **Use slower movement speeds initially**
- **Have manual override ready**

## Performance Optimization

### Detection Speed
- **Reduce point cloud density** in camera settings
- **Limit detection region** to workspace area
- **Use appropriate confidence thresholds**
- **Enable GPU acceleration** if available

### Selection Efficiency
- **Use fewer candidates** (3-5) for faster review
- **Enable auto-execute** for trusted scenarios
- **Disable confirmation** for production use
- **Create custom preset configurations**

### Execution Reliability
- **Tune movement speeds** for stability
- **Adjust approach distances** for safety
- **Calibrate gripper positions** precisely
- **Use collision-aware planning**

## Advanced Features

### Custom Candidate Filtering
Implement custom filtering logic in the interactive node:
```python
def custom_filter(self, candidates):
    # Add custom filtering logic
    # E.g., prefer grasps near workspace center
    filtered = []
    for candidate in candidates:
        if self.is_in_preferred_region(candidate.pose):
            filtered.append(candidate)
    return filtered
```

### Integration with Other Systems
- **Vision preprocessing** for object segmentation
- **Task planning** for pick-and-place sequences  
- **Learning from corrections** for improvement
- **Multi-arm coordination** for complex tasks

### Batch Processing
Process multiple objects in sequence:
1. Detect grasps for all objects
2. Present prioritized candidate list
3. Execute grasps in optimal order
4. Handle failures gracefully

## Future Enhancements

### Planned Features
- **ML-based candidate ranking** using user preferences
- **Haptic feedback** for grasp quality
- **AR visualization** for spatial understanding
- **Voice commands** for hands-free operation

### Integration Roadmap
- **ROS2 Actions** for better progress monitoring
- **Behavior Trees** for complex task sequences
- **Digital twins** for simulation validation
- **Cloud-based learning** for shared improvements

This interactive workflow provides a robust, user-guided approach to robotic grasping that balances automation with human oversight, ensuring reliable and safe operation in complex environments.