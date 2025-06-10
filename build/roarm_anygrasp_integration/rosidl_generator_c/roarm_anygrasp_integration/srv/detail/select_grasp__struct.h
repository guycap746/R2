// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from roarm_anygrasp_integration:srv/SelectGrasp.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__STRUCT_H_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SelectGrasp in the package roarm_anygrasp_integration.
typedef struct roarm_anygrasp_integration__srv__SelectGrasp_Request
{
  int32_t selected_grasp_index;
  /// Optional: User can request to see more candidates
  bool show_more_candidates;
} roarm_anygrasp_integration__srv__SelectGrasp_Request;

// Struct for a sequence of roarm_anygrasp_integration__srv__SelectGrasp_Request.
typedef struct roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence
{
  roarm_anygrasp_integration__srv__SelectGrasp_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"
// Member 'selected_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in srv/SelectGrasp in the package roarm_anygrasp_integration.
typedef struct roarm_anygrasp_integration__srv__SelectGrasp_Response
{
  /// Response: Confirmation and execution status
  bool success;
  rosidl_runtime_c__String message;
  /// Selected grasp details for confirmation
  geometry_msgs__msg__PoseStamped selected_pose;
  float confidence_score;
  float grasp_width;
} roarm_anygrasp_integration__srv__SelectGrasp_Response;

// Struct for a sequence of roarm_anygrasp_integration__srv__SelectGrasp_Response.
typedef struct roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence
{
  roarm_anygrasp_integration__srv__SelectGrasp_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__STRUCT_H_
