# Roboflow Integration for AnyGrasp Dataset Building

This document describes the Roboflow integration that automatically captures and uploads RGB images with grasp point annotations during the AnyGrasp workflow. This creates valuable training data to improve grasp detection models.

## Overview

The Roboflow integration provides **automated dataset creation** for improving grasp detection:

1. **Image Capture**: Automatically captures RGB images during grasp detection
2. **Annotation**: Creates bounding box annotations around detected grasp points
3. **Classification**: Labels images with grasp quality and confidence scores
4. **Upload**: Sends annotated images to Roboflow for dataset building
5. **Model Training**: Enables creation of improved classification models

## Architecture Components

### 📊 Roboflow Integration Node
- **Package**: `roarm_anygrasp_integration`
- **Executable**: `roboflow_integration_node`
- **Purpose**: Handles image capture, annotation, and upload to Roboflow

**Key Features:**
- Automatic RGB image capture during grasp detection
- Real-time grasp point annotation generation
- Configurable upload filters and batching
- Queue management for reliable uploads
- Integration with AnyGrasp workflow states

### 🌐 Roboflow Configuration Panel
- **File**: `/foxglove_panels/roboflow_configuration_panel.ts`
- **Purpose**: Web-based interface for Roboflow setup and monitoring

**Key Features:**
- API key configuration
- Workspace and project management
- Upload statistics and monitoring
- Manual upload triggers
- Error reporting and diagnostics

## Service Interfaces

### Configure Roboflow
```bash
# Service: /roboflow/configure
# Type: roarm_anygrasp_integration/ConfigureRoboflow

# Request:
api_key: "your_roboflow_api_key"
workspace_name: "roarm-grasping"
project_name: "grasp-detection"
dataset_version: "v1"
auto_upload: true
include_failed_grasps: true
annotation_format: "yolo"
min_confidence_for_upload: 0.3

# Response:
success: true/false
message: "Configuration status"
workspace_url: "https://app.roboflow.com/workspace"
project_url: "https://app.roboflow.com/workspace/project"
```

### Upload Images
```bash
# Service: /roboflow/upload_image
# Type: roarm_anygrasp_integration/UploadToRoboflow

# Request:
rgb_image: sensor_msgs/Image
grasp_poses: geometry_msgs/PoseArray
confidence_scores: [0.8, 0.6, 0.9, ...]
class_labels: ["good_grasp", "fair_grasp", ...]
scene_description: "Pick and place scenario"
object_types: "mixed_objects"
upload_immediately: true

# Response:
success: true/false
message: "Upload status"
roboflow_image_id: "img_12345"
annotation_count: 3
```

## Setup and Configuration

### 1. Get Roboflow API Key

1. Visit [app.roboflow.com](https://app.roboflow.com)
2. Create a free account
3. Go to Settings → Account → API Key
4. Copy your private API key

### 2. Create Roboflow Project

1. **Create Workspace**: `roarm-grasping` (or your preferred name)
2. **Create Project**: `grasp-detection` 
3. **Set Project Type**: Object Detection
4. **Configure Classes**: 
   - `grasp_point` - Detected grasp locations
   - `good_grasp` - High-confidence grasps (>0.8)
   - `fair_grasp` - Medium-confidence grasps (0.5-0.8)
   - `poor_grasp` - Low-confidence grasps (<0.5)

### 3. Launch with Roboflow Integration

```bash
# Launch complete system with Roboflow enabled
ros2 launch roarm_moveit roarm_anygrasp_demo.launch.py roboflow_enabled:=true

# Or launch Roboflow node separately
ros2 run roarm_anygrasp_integration roboflow_integration_node \
  --ros-args -p roboflow_api_key:="your_api_key" \
              -p workspace_name:="roarm-grasping" \
              -p project_name:="grasp-detection"
```

### 4. Configure via Foxglove

1. **Access Foxglove**: Connect to `ws://YOUR_ROBOT_IP:8765`
2. **Add Roboflow Panel**: Import `/foxglove_panels/roboflow_configuration_panel.ts`
3. **Enter Configuration**:
   - API Key
   - Workspace name
   - Project name
   - Upload settings
4. **Apply Configuration**: Click "Apply Configuration"

### 5. Configure via Service

```bash
# Configure Roboflow via ROS2 service
ros2 service call /roboflow/configure \
  roarm_anygrasp_integration/srv/ConfigureRoboflow \
  "{
    api_key: 'your_roboflow_api_key',
    workspace_name: 'roarm-grasping', 
    project_name: 'grasp-detection',
    dataset_version: 'v1',
    auto_upload: true,
    include_failed_grasps: true,
    annotation_format: 'yolo',
    min_confidence_for_upload: 0.3
  }"
```

## Automated Workflow

### Data Collection Process

1. **Scene Setup**: Position objects in camera view
2. **Grasp Detection**: AnyGrasp analyzes scene and finds candidates
3. **Image Capture**: RGB frame is automatically captured
4. **Annotation Creation**: Grasp points are converted to bounding boxes
5. **Quality Classification**: Grasps are classified by confidence level
6. **Queue Upload**: Image and annotations are queued for upload
7. **Batch Processing**: Uploads are processed in background batches

### Annotation Format

Images are annotated with bounding boxes around grasp points:

```yaml
# YOLO format annotation
annotations:
  - class: "grasp_point"
    class_id: 0
    bbox: [center_x, center_y, width, height]  # Normalized coordinates
    confidence: 0.85
    metadata:
      world_coordinates: [x, y, z]  # 3D position in robot frame
      grasp_width: 0.05  # Required gripper opening
      quality_score: 0.78  # Overall grasp quality
```

### Upload Triggers

Images are uploaded automatically when:
- **Grasp detection completes** (if auto_upload enabled)
- **User selects grasp** in interactive workflow
- **Grasp execution completes** (success or failure)
- **Manual trigger** via Foxglove panel or service call

## Configuration Parameters

### Node Parameters
```yaml
roboflow_integration_node:
  ros__parameters:
    roboflow_api_key: ""                    # Your Roboflow API key
    workspace_name: "roarm-grasping"        # Roboflow workspace
    project_name: "grasp-detection"         # Roboflow project
    auto_upload: true                       # Enable automatic uploads
    storage_path: "/tmp/roboflow_images"    # Local storage for images
    max_queue_size: 100                     # Maximum upload queue size
    upload_batch_size: 5                    # Images per upload batch
    min_confidence_for_upload: 0.3          # Minimum confidence threshold
    include_failed_grasps: true             # Upload failed grasp attempts
    annotation_format: "yolo"               # Annotation format (yolo/coco/voc)
```

### Upload Filters
```yaml
upload_filters:
  min_confidence: 0.3          # Skip grasps below this confidence
  max_images_per_hour: 50      # Rate limiting
  deduplicate_similar: true    # Skip very similar scenes
  min_grasp_count: 1          # Require at least N grasps in scene
  max_grasp_count: 20         # Skip scenes with too many grasps
```

## Data Quality and Labeling

### Automatic Classification

Grasps are automatically classified based on:
- **Confidence Score**: AnyGrasp detection confidence
- **Execution Result**: Success/failure of actual grasp
- **Geometric Validity**: Reachability and collision checks
- **Grasp Quality Metrics**: Width, approach angle, stability

### Label Categories

| Class | Criteria | Usage |
|-------|----------|-------|
| `excellent_grasp` | Confidence > 0.9, successful execution | High-quality training examples |
| `good_grasp` | Confidence > 0.7, likely successful | Positive training examples |
| `fair_grasp` | Confidence > 0.5, uncertain outcome | Borderline examples for improvement |
| `poor_grasp` | Confidence < 0.5, likely failure | Negative training examples |
| `failed_grasp` | Execution failed regardless of confidence | Hard negative examples |

### Metadata Enrichment

Each uploaded image includes rich metadata:
```json
{
  "scene_info": {
    "lighting_conditions": "normal",
    "object_count": 3,
    "object_types": ["cube", "sphere", "cylinder"],
    "background_type": "textured_surface",
    "camera_distance": 0.6
  },
  "robot_state": {
    "joint_positions": [...],
    "end_effector_pose": {...},
    "gripper_width": 0.08
  },
  "detection_info": {
    "total_grasps_detected": 8,
    "filtered_grasps": 5,
    "processing_time": 1.2,
    "algorithm_version": "anygrasp_v1.0"
  }
}
```

## Dataset Management

### Version Control

Roboflow automatically versions your dataset:
- **v1**: Initial dataset with basic annotations
- **v2**: Enhanced with execution results
- **v3**: Improved with user feedback
- **v4**: Augmented with additional scenarios

### Data Augmentation

Roboflow provides automatic augmentation:
- **Brightness/Contrast**: Simulate different lighting
- **Noise**: Add camera noise simulation
- **Blur**: Simulate motion blur
- **Rotation**: Limited rotation augmentation
- **Crop**: Focus on grasp regions

### Export Formats

Dataset can be exported in multiple formats:
- **YOLO**: For custom training pipelines
- **COCO**: For research and comparison
- **Pascal VOC**: For legacy systems
- **TensorFlow**: For TF-based training
- **PyTorch**: For PyTorch workflows

## Model Training Integration

### Custom Model Training

Use collected data to train improved models:

1. **Export Dataset**: Download from Roboflow in preferred format
2. **Train Model**: Use your preferred ML framework
3. **Validate Performance**: Test on held-out validation set
4. **Deploy Model**: Replace AnyGrasp components with trained model

### Roboflow Train

Use Roboflow's managed training:

1. **Configure Training**: Set up training parameters in Roboflow
2. **Start Training**: Use Roboflow's cloud training
3. **Monitor Progress**: Track training metrics
4. **Deploy Model**: Use Roboflow inference API

### Integration Back to ROS2

Replace AnyGrasp detection with trained model:

```python
# Custom model integration
from roboflow import Roboflow

class CustomGraspDetector:
    def __init__(self):
        rf = Roboflow(api_key="your_key")
        project = rf.workspace("workspace").project("project")
        self.model = project.version(4).model
    
    def detect_grasps(self, image_path):
        prediction = self.model.predict(image_path, confidence=40)
        return self.convert_to_grasp_poses(prediction)
```

## Monitoring and Analytics

### Upload Statistics

Monitor via Foxglove panel or topics:
```bash
# Monitor upload status
ros2 topic echo /roboflow/upload_status

# Check queue size
ros2 param get /roboflow_integration_node queue_size
```

### Dataset Analytics

Track dataset growth in Roboflow:
- **Total Images**: Overall dataset size
- **Annotations per Image**: Average grasp density
- **Class Distribution**: Balance of grasp quality
- **Upload Frequency**: Data collection rate

### Performance Metrics

Monitor impact on system performance:
- **Upload Queue Size**: Memory usage indicator
- **Processing Latency**: Impact on real-time operation
- **Network Bandwidth**: Upload data usage
- **Storage Usage**: Local disk consumption

## Troubleshooting

### Common Issues

**1. Upload Failures**
```bash
# Check API key and configuration
ros2 service call /roboflow/configure roarm_anygrasp_integration/srv/ConfigureRoboflow "{...}"

# Verify network connectivity
curl -X GET "https://api.roboflow.com/YOUR_WORKSPACE/YOUR_PROJECT"

# Check queue status
ros2 topic echo /roboflow/upload_status
```

**2. Authentication Errors**
- Verify API key is correct and not expired
- Check workspace and project names
- Ensure project exists and is accessible

**3. High Queue Size**
- Reduce upload frequency with min_confidence filter
- Increase upload_batch_size for faster processing
- Check network connectivity for upload delays

**4. Missing Annotations**
- Verify camera calibration for proper 3D-2D projection
- Check grasp pose coordinate frames
- Ensure minimum confidence threshold is appropriate

### Debug Commands

```bash
# Enable debug logging
ros2 run roarm_anygrasp_integration roboflow_integration_node \
  --ros-args --log-level debug

# Monitor all Roboflow topics
ros2 topic list | grep roboflow | xargs -I {} ros2 topic echo {} --once

# Check service availability
ros2 service list | grep roboflow

# Test manual upload
ros2 service call /roboflow/upload_image roarm_anygrasp_integration/srv/UploadToRoboflow "{...}"
```

### Performance Optimization

**Reduce Upload Volume:**
- Increase `min_confidence_for_upload`
- Enable `deduplicate_similar` scenes
- Set `max_images_per_hour` limit

**Improve Upload Speed:**
- Increase `upload_batch_size`
- Use faster network connection
- Enable image compression

**Memory Management:**
- Set appropriate `max_queue_size`
- Monitor queue growth
- Clear failed uploads periodically

## Advanced Features

### Custom Annotation Logic

Extend annotation creation for specific needs:

```python
# Custom annotation for specialized grasps
def create_custom_annotations(self, cv_image, grasp_poses, object_info):
    annotations = []
    
    for pose in grasp_poses:
        # Add object-specific classification
        object_class = self.classify_object_at_pose(pose, object_info)
        
        # Create specialized bounding box
        bbox = self.create_oriented_bbox(pose, object_class)
        
        # Add rich metadata
        annotation = {
            'class': f'{object_class}_grasp',
            'bbox': bbox,
            'metadata': {
                'object_type': object_class,
                'grasp_approach': self.analyze_approach(pose),
                'difficulty': self.estimate_difficulty(pose, object_info)
            }
        }
        annotations.append(annotation)
    
    return annotations
```

### Multi-Camera Support

Capture from multiple camera angles:

```python
# Multi-camera capture for richer dataset
class MultiCameraCapture:
    def __init__(self):
        self.cameras = ['front', 'side', 'top']
        
    def capture_scene(self, grasp_poses):
        images = {}
        for camera in self.cameras:
            img = self.capture_from_camera(camera)
            annotations = self.project_grasps_to_camera(grasp_poses, camera)
            images[camera] = {'image': img, 'annotations': annotations}
        
        return images
```

### Active Learning Integration

Implement active learning for selective data collection:

```python
# Active learning for targeted data collection
class ActiveLearningFilter:
    def should_upload(self, image, grasps, model_uncertainty):
        # Upload uncertain predictions for model improvement
        if model_uncertainty > threshold:
            return True
        
        # Upload novel scenes
        if self.is_novel_scene(image):
            return True
            
        # Upload failure cases
        if self.execution_failed(grasps):
            return True
            
        return False
```

## Future Enhancements

### Planned Features
- **Real-time model updates** from Roboflow
- **Federated learning** across multiple robots
- **Synthetic data generation** for data augmentation
- **Semi-supervised learning** with unlabeled data
- **Multi-modal fusion** with depth and tactile data

### Integration Opportunities
- **Simulation data** from Isaac Sim or Gazebo
- **Human demonstrations** for imitation learning
- **Failure analysis** for robust grasp planning
- **Domain adaptation** for different environments
- **Transfer learning** between different robot platforms

This Roboflow integration creates a powerful feedback loop where real-world grasp attempts continuously improve the underlying detection models, leading to better grasp success rates over time.