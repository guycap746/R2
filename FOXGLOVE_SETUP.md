# Foxglove Integration for ROS2 + RoArm M3 + AnyGrasp

This document provides comprehensive setup and usage instructions for Foxglove Studio integration with your ROS2 + RoArm M3 + AnyGrasp system.

## Overview

Foxglove Studio provides a powerful web-based interface for robotics visualization, debugging, and control. This integration includes:

- **Real-time 3D visualization** of the RoArm M3 robot
- **Interactive control panels** for robot operation
- **AnyGrasp visualization** with grasp selection and execution
- **Camera streams** and point cloud visualization
- **System monitoring** and diagnostics
- **Custom panels** for specialized robot control

## Prerequisites

- Docker container with Foxglove Bridge installed
- ROS2 system running with RoArm M3 packages
- Network access for web-based interface
- Modern web browser (Chrome, Firefox, Edge)

## Quick Start

### 1. Launch with Foxglove Integration

Start the complete system with Foxglove Bridge:

```bash
# Full system with integrated Foxglove
ros2 launch roarm_moveit roarm_anygrasp_demo.launch.py

# Or launch Foxglove separately
ros2 launch roarm_moveit foxglove_roarm.launch.py
```

### 2. Access Foxglove Studio

#### Option A: Foxglove Studio Desktop App
1. Download from [foxglove.dev](https://foxglove.dev/download)
2. Install and launch Foxglove Studio
3. Connect to: `ws://YOUR_ROBOT_IP:8765`

#### Option B: Web Browser (foxglove.dev)
1. Go to [app.foxglove.dev](https://app.foxglove.dev)
2. Click "Open connection"
3. Enter: `ws://YOUR_ROBOT_IP:8765`
4. Click "Open"

#### Option C: Local Web Interface (if available)
1. Navigate to: `http://YOUR_ROBOT_IP:8080`
2. Use built-in web interface

### 3. Load Robot Layout

1. In Foxglove Studio, click "⚙️ Settings"
2. Go to "Layouts" 
3. Import `/ros2_ws/foxglove_configs/roarm_m3_layout.json`
4. Or manually configure panels as described below

## Configuration

### Foxglove Bridge Settings

The bridge runs on port 8765 by default. Configure in launch file:

```python
foxglove_bridge_node = Node(
    package='foxglove_bridge',
    executable='foxglove_bridge',
    parameters=[{
        'port': 8765,
        'address': '0.0.0.0',  # Listen on all interfaces
        'tls': False,          # Set to True for HTTPS
    }]
)
```

### Custom Configuration File

Edit `/ros2_ws/foxglove_configs/foxglove_bridge_config.yaml`:

```yaml
port: 8765
address: "0.0.0.0"
topics:
  "/joint_states":
    qos:
      reliability: "reliable"
      depth: 10
  "/camera/depth/color/points":
    qos:
      reliability: "best_effort"
      depth: 2
```

## Panel Setup

### 1. 3D Visualization Panel

Configure the main 3D panel for robot visualization:

1. **Add 3D Panel**: Click "+" → "3D"
2. **Robot Model**: 
   - Add layer: "URDF"
   - Source type: "Parameter"
   - Parameter: `/robot_description`
3. **Transform Tree**:
   - Add layer: "Transforms"
   - Enable axis visualization
4. **Point Cloud**:
   - Add layer: "Point Cloud"
   - Topic: `/camera/depth/color/points`
5. **Grasp Poses**:
   - Add layer: "Pose Array"
   - Topic: `/anygrasp/grasp_poses`

### 2. Robot Control Panel

Custom panel for RoArm M3 control:

1. **Add Custom Panel**: Import the TypeScript panel
2. **File**: `/ros2_ws/foxglove_panels/roarm_control_panel.ts`
3. **Features**:
   - Joint state monitoring
   - Position control sliders
   - Preset positions
   - Emergency stop
   - Gripper control
   - LED control

### 3. AnyGrasp Visualization Panel

Specialized panel for grasp detection and execution:

1. **Add Custom Panel**: Import the AnyGrasp panel
2. **File**: `/ros2_ws/foxglove_panels/anygrasp_visualization_panel.ts`
3. **Features**:
   - Grasp detection trigger
   - Grasp filtering and sorting
   - Grasp selection and execution
   - Confidence visualization

### 4. Camera Stream Panel

For visual feedback:

1. **Add Image Panel**: Click "+" → "Image"
2. **Topic**: `/camera/color/image_raw`
3. **Settings**: Enable smooth rendering

### 5. Diagnostics Panel

System health monitoring:

1. **Add Diagnostics Panel**: Click "+" → "Diagnostics"
2. **Topic**: `/diagnostics`
3. **Pin important nodes**: roarm_driver, moveit, anygrasp_node

### 6. Parameter Panel

Runtime parameter adjustment:

1. **Add Parameters Panel**: Click "+" → "Parameters"
2. **Expand nodes**: /servo_node, /anygrasp_node, /roarm_driver
3. **Modify parameters** in real-time

## Usage Guide

### Basic Robot Operation

1. **Connection Check**: Verify green "Connected" status in control panel
2. **Home Position**: Click "🏠 Home Position" to initialize
3. **Manual Control**: Use X/Y/Z sliders for position control
4. **Preset Positions**: Use preset buttons for common poses

### Grasp Detection and Execution

1. **Scene Setup**: Position objects in camera view
2. **Detect Grasps**: Click "🔍 Detect Grasps" in AnyGrasp panel
3. **Filter Results**: Adjust confidence threshold and max grasps
4. **Select Grasp**: Click on desired grasp in list
5. **Execute**: Click "🚀 Execute Selected Grasp"

### Camera and Perception

1. **Live Feed**: Monitor camera stream panel
2. **Point Cloud**: View 3D point cloud in main panel
3. **Detection Results**: See grasp poses overlaid on 3D view

### System Monitoring

1. **Diagnostics**: Check system health in diagnostics panel
2. **Joint States**: Monitor current joint positions
3. **Topic Monitor**: Use built-in topic monitoring tools

## Advanced Features

### Custom Panel Development

Create custom panels for specific needs:

```typescript
// Custom panel template
import { PanelExtensionContext } from "@foxglove/studio";

function MyCustomPanel({ context }: { context: PanelExtensionContext }) {
  // Subscribe to topics
  context.subscribe([{ topic: "/my_topic" }]);
  
  // Publish messages
  context.publish?.("/my_output_topic", { data: "hello" });
  
  // Call services
  context.callService?.("/my_service", { request: "data" });
  
  return <div>My Custom Panel</div>;
}
```

### Layout Sharing

Share layouts between team members:

1. **Export Layout**: Settings → Layouts → Export
2. **Share File**: Send `.json` layout file
3. **Import Layout**: Settings → Layouts → Import

### Recording and Playback

Record robot sessions for analysis:

1. **Start Recording**: Click record button in Foxglove
2. **Perform Operations**: Execute robot tasks
3. **Stop Recording**: Stop and save recording
4. **Playback**: Load recording for offline analysis

## Troubleshooting

### Connection Issues

**Problem**: Cannot connect to Foxglove Bridge
**Solutions**:
- Check bridge is running: `ros2 node list | grep foxglove`
- Verify port: `netstat -ln | grep 8765`
- Check firewall settings
- Try different IP/port combination

**Problem**: Bridge starts but no topics visible
**Solutions**:
- Check topic whitelist in configuration
- Verify topics are publishing: `ros2 topic list`
- Check QoS compatibility

### Performance Issues

**Problem**: Slow or choppy visualization
**Solutions**:
- Reduce point cloud density in camera settings
- Lower update rates for high-frequency topics
- Disable unused visualization layers
- Use compression if available

**Problem**: High CPU/memory usage
**Solutions**:
- Limit number of visualization panels
- Reduce history length for topics
- Close unused browser tabs
- Monitor system resources

### Visualization Issues

**Problem**: Robot model not displaying
**Solutions**:
- Check `/robot_description` parameter exists
- Verify URDF file is valid
- Check transform tree is publishing
- Reload URDF layer

**Problem**: Point cloud not visible
**Solutions**:
- Check camera is publishing: `ros2 topic echo /camera/depth/color/points`
- Verify point cloud format compatibility
- Adjust visualization settings (point size, color mode)
- Check camera calibration

## Network Configuration

### Port Configuration

Default ports used:
- **Foxglove Bridge**: 8765 (WebSocket)
- **Web Interface**: 8080 (HTTP)
- **ROS2 DDS**: 7400-7500 (UDP)

### Firewall Rules

For Ubuntu/Linux:
```bash
# Allow Foxglove Bridge
sudo ufw allow 8765

# Allow web interface
sudo ufw allow 8080

# Allow ROS2 DDS (if needed)
sudo ufw allow 7400:7500/udp
```

### Docker Network

If using Docker, ensure ports are exposed:
```yaml
# docker-compose.yml
services:
  ros2-dev:
    ports:
      - "8765:8765"  # Foxglove Bridge
      - "8080:8080"  # Web interface
```

## Security Considerations

### Authentication

For production use, enable authentication:

```yaml
# foxglove_bridge_config.yaml
tls: true
certfile: "/path/to/cert.pem"
keyfile: "/path/to/key.pem"
```

### Access Control

Limit topic access:
```yaml
topic_whitelist:
  - "/joint_states"
  - "/camera/color/image_raw"
  # Only expose necessary topics
```

### Network Security

- Use VPN for remote access
- Implement proper firewall rules
- Consider using HTTPS/WSS for web access
- Regular security updates

## Performance Optimization

### Topic Selection

Only publish necessary topics:
```python
# In launch file, configure specific topics
topic_whitelist = [
    "/joint_states",
    "/tf", 
    "/tf_static",
    "/camera/color/image_raw",
    "/anygrasp/grasp_poses"
]
```

### QoS Configuration

Optimize QoS for each topic type:
```yaml
topics:
  "/joint_states":
    qos:
      reliability: "reliable"
      depth: 5
  "/camera/depth/color/points":
    qos:
      reliability: "best_effort"
      depth: 1
```

### Update Rates

Configure appropriate update rates:
```python
# Lower update rate for expensive operations
parameters=[{
    'publish_frequency': 10.0,  # 10 Hz instead of 30 Hz
}]
```

## Integration Examples

### Pick and Place Workflow

1. **Setup**: Launch full system with cameras
2. **Scan**: Trigger grasp detection
3. **Select**: Choose best grasp from Foxglove panel
4. **Execute**: Run pick and place sequence
5. **Monitor**: Watch execution in 3D visualization

### Teleoperation

1. **Connect**: Remote operator connects via Foxglove
2. **Control**: Use custom control panels for robot operation
3. **Feedback**: Visual and diagnostic feedback in real-time
4. **Safety**: Emergency stop always available

### Data Collection

1. **Record**: Start recording robot operations
2. **Execute**: Perform various robot tasks
3. **Analyze**: Review recordings offline
4. **Improve**: Use data for algorithm improvement

## Support and Resources

- **Foxglove Documentation**: [docs.foxglove.dev](https://docs.foxglove.dev)
- **ROS2 Bridge**: [github.com/foxglove/ros-foxglove-bridge](https://github.com/foxglove/ros-foxglove-bridge)
- **Custom Panels**: [docs.foxglove.dev/docs/panels/custom-panels](https://docs.foxglove.dev/docs/panels/custom-panels)
- **Community**: [foxglove.dev/slack](https://foxglove.dev/slack)

For issues specific to this integration:
- Check robot driver status: `ros2 topic echo /roarm_driver/status`
- Monitor bridge logs: `ros2 node info /foxglove_bridge`
- Test individual components: `ros2 launch roarm_moveit foxglove_roarm.launch.py`