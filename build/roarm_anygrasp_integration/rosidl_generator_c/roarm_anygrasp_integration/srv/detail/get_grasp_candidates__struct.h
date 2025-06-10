// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from roarm_anygrasp_integration:srv/GetGraspCandidates.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__STRUCT_H_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetGraspCandidates in the package roarm_anygrasp_integration.
typedef struct roarm_anygrasp_integration__srv__GetGraspCandidates_Request
{
  int32_t num_candidates;
  /// Minimum confidence threshold (0.0 - 1.0)
  float min_confidence;
} roarm_anygrasp_integration__srv__GetGraspCandidates_Request;

// Struct for a sequence of roarm_anygrasp_integration__srv__GetGraspCandidates_Request.
typedef struct roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence
{
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'grasp_poses'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"
// Member 'confidence_scores'
// Member 'grasp_widths'
// Member 'quality_scores'
// Member 'original_indices'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'detection_status'
#include "rosidl_runtime_c/string.h"
// Member 'detection_timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in srv/GetGraspCandidates in the package roarm_anygrasp_integration.
typedef struct roarm_anygrasp_integration__srv__GetGraspCandidates_Response
{
  /// Response: Top grasp candidates with metadata
  geometry_msgs__msg__PoseStamped__Sequence grasp_poses;
  rosidl_runtime_c__float__Sequence confidence_scores;
  rosidl_runtime_c__float__Sequence grasp_widths;
  rosidl_runtime_c__float__Sequence quality_scores;
  rosidl_runtime_c__int32__Sequence original_indices;
  /// Detection metadata
  rosidl_runtime_c__String detection_status;
  int32_t total_grasps_detected;
  builtin_interfaces__msg__Time detection_timestamp;
} roarm_anygrasp_integration__srv__GetGraspCandidates_Response;

// Struct for a sequence of roarm_anygrasp_integration__srv__GetGraspCandidates_Response.
typedef struct roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence
{
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__STRUCT_H_
