// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from roarm_anygrasp_integration:srv/UploadToRoboflow.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__STRUCT_H_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'rgb_image'
#include "sensor_msgs/msg/detail/image__struct.h"
// Member 'grasp_poses'
#include "geometry_msgs/msg/detail/pose_array__struct.h"
// Member 'confidence_scores'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'class_labels'
// Member 'scene_description'
// Member 'object_types'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/UploadToRoboflow in the package roarm_anygrasp_integration.
typedef struct roarm_anygrasp_integration__srv__UploadToRoboflow_Request
{
  sensor_msgs__msg__Image rgb_image;
  geometry_msgs__msg__PoseArray grasp_poses;
  rosidl_runtime_c__float__Sequence confidence_scores;
  rosidl_runtime_c__String__Sequence class_labels;
  /// Metadata
  rosidl_runtime_c__String scene_description;
  rosidl_runtime_c__String object_types;
  bool include_annotations;
  bool upload_immediately;
} roarm_anygrasp_integration__srv__UploadToRoboflow_Request;

// Struct for a sequence of roarm_anygrasp_integration__srv__UploadToRoboflow_Request.
typedef struct roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence
{
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'roboflow_image_id'
// Member 'upload_url'
// Member 'dataset_version'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/UploadToRoboflow in the package roarm_anygrasp_integration.
typedef struct roarm_anygrasp_integration__srv__UploadToRoboflow_Response
{
  /// Response: Upload status and details
  bool success;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__String roboflow_image_id;
  rosidl_runtime_c__String upload_url;
  int32_t annotation_count;
  rosidl_runtime_c__String dataset_version;
} roarm_anygrasp_integration__srv__UploadToRoboflow_Response;

// Struct for a sequence of roarm_anygrasp_integration__srv__UploadToRoboflow_Response.
typedef struct roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence
{
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__STRUCT_H_
