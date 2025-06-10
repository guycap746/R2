// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from roarm_anygrasp_integration:srv/CalibrateCamera.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_H_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CalibrateCamera in the package roarm_anygrasp_integration.
typedef struct roarm_anygrasp_integration__srv__CalibrateCamera_Request
{
  uint8_t structure_needs_at_least_one_member;
} roarm_anygrasp_integration__srv__CalibrateCamera_Request;

// Struct for a sequence of roarm_anygrasp_integration__srv__CalibrateCamera_Request.
typedef struct roarm_anygrasp_integration__srv__CalibrateCamera_Request__Sequence
{
  roarm_anygrasp_integration__srv__CalibrateCamera_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_anygrasp_integration__srv__CalibrateCamera_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CalibrateCamera in the package roarm_anygrasp_integration.
typedef struct roarm_anygrasp_integration__srv__CalibrateCamera_Response
{
  bool success;
  rosidl_runtime_c__String message;
  double primary_reprojection_error;
  double side_reprojection_error;
} roarm_anygrasp_integration__srv__CalibrateCamera_Response;

// Struct for a sequence of roarm_anygrasp_integration__srv__CalibrateCamera_Response.
typedef struct roarm_anygrasp_integration__srv__CalibrateCamera_Response__Sequence
{
  roarm_anygrasp_integration__srv__CalibrateCamera_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} roarm_anygrasp_integration__srv__CalibrateCamera_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_H_
