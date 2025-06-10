// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__STRUCT_H_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'api_key'
// Member 'workspace_name'
// Member 'project_name'
// Member 'dataset_version'
// Member 'annotation_format'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ConfigureRoboflow in the package roarm_anygrasp_integration.
typedef struct roarm_anygrasp_integration__srv__ConfigureRoboflow_Request
{
  rosidl_runtime_c__String api_key;
  rosidl_runtime_c__String workspace_name;
  rosidl_runtime_c__String project_name;
  rosidl_runtime_c__String dataset_version;
  bool auto_upload;
  bool include_failed_grasps;
  rosidl_runtime_c__String annotation_format;
  float min_confidence_for_upload;
} roarm_anygrasp_integration__srv__ConfigureRoboflow_Request;

// Struct for a sequence of roarm_anygrasp_integration__srv__ConfigureRoboflow_Request.
typedef struct roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence
{
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// Member 'workspace_url'
// Member 'project_url'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ConfigureRoboflow in the package roarm_anygrasp_integration.
typedef struct roarm_anygrasp_integration__srv__ConfigureRoboflow_Response
{
  /// Response: Configuration status
  bool success;
  rosidl_runtime_c__String message;
  rosidl_runtime_c__String workspace_url;
  rosidl_runtime_c__String project_url;
  int32_t total_images_in_dataset;
} roarm_anygrasp_integration__srv__ConfigureRoboflow_Response;

// Struct for a sequence of roarm_anygrasp_integration__srv__ConfigureRoboflow_Response.
typedef struct roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence
{
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__STRUCT_H_
