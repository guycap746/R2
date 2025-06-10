// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from roarm_anygrasp_integration:srv/GetGraspCandidates.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__rosidl_typesupport_introspection_c.h"
#include "roarm_anygrasp_integration/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__functions.h"
#include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request__init(message_memory);
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_fini_function(void * message_memory)
{
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_message_member_array[2] = {
  {
    "num_candidates",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__GetGraspCandidates_Request, num_candidates),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "min_confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__GetGraspCandidates_Request, min_confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_message_members = {
  "roarm_anygrasp_integration__srv",  // message namespace
  "GetGraspCandidates_Request",  // message name
  2,  // number of fields
  sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Request),
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_message_member_array,  // message members
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_message_type_support_handle = {
  0,
  &roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, GetGraspCandidates_Request)() {
  if (!roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_message_type_support_handle.typesupport_identifier) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &roarm_anygrasp_integration__srv__GetGraspCandidates_Request__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__rosidl_typesupport_introspection_c.h"
// already included above
// #include "roarm_anygrasp_integration/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__functions.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__struct.h"


// Include directives for member types
// Member `grasp_poses`
#include "geometry_msgs/msg/pose_stamped.h"
// Member `grasp_poses`
#include "geometry_msgs/msg/detail/pose_stamped__rosidl_typesupport_introspection_c.h"
// Member `confidence_scores`
// Member `grasp_widths`
// Member `quality_scores`
// Member `original_indices`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `detection_status`
#include "rosidl_runtime_c/string_functions.h"
// Member `detection_timestamp`
#include "builtin_interfaces/msg/time.h"
// Member `detection_timestamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response__init(message_memory);
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_fini_function(void * message_memory)
{
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(message_memory);
}

size_t roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__size_function__GetGraspCandidates_Response__grasp_poses(
  const void * untyped_member)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return member->size;
}

const void * roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__grasp_poses(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__PoseStamped__Sequence * member =
    (const geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void * roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__grasp_poses(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  return &member->data[index];
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__fetch_function__GetGraspCandidates_Response__grasp_poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__PoseStamped * item =
    ((const geometry_msgs__msg__PoseStamped *)
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__grasp_poses(untyped_member, index));
  geometry_msgs__msg__PoseStamped * value =
    (geometry_msgs__msg__PoseStamped *)(untyped_value);
  *value = *item;
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__assign_function__GetGraspCandidates_Response__grasp_poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__PoseStamped * item =
    ((geometry_msgs__msg__PoseStamped *)
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__grasp_poses(untyped_member, index));
  const geometry_msgs__msg__PoseStamped * value =
    (const geometry_msgs__msg__PoseStamped *)(untyped_value);
  *item = *value;
}

bool roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__resize_function__GetGraspCandidates_Response__grasp_poses(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__PoseStamped__Sequence * member =
    (geometry_msgs__msg__PoseStamped__Sequence *)(untyped_member);
  geometry_msgs__msg__PoseStamped__Sequence__fini(member);
  return geometry_msgs__msg__PoseStamped__Sequence__init(member, size);
}

size_t roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__size_function__GetGraspCandidates_Response__confidence_scores(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__confidence_scores(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__confidence_scores(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__fetch_function__GetGraspCandidates_Response__confidence_scores(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__confidence_scores(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__assign_function__GetGraspCandidates_Response__confidence_scores(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__confidence_scores(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__resize_function__GetGraspCandidates_Response__confidence_scores(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__size_function__GetGraspCandidates_Response__grasp_widths(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__grasp_widths(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__grasp_widths(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__fetch_function__GetGraspCandidates_Response__grasp_widths(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__grasp_widths(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__assign_function__GetGraspCandidates_Response__grasp_widths(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__grasp_widths(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__resize_function__GetGraspCandidates_Response__grasp_widths(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__size_function__GetGraspCandidates_Response__quality_scores(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__quality_scores(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__quality_scores(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__fetch_function__GetGraspCandidates_Response__quality_scores(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__quality_scores(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__assign_function__GetGraspCandidates_Response__quality_scores(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__quality_scores(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__resize_function__GetGraspCandidates_Response__quality_scores(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__size_function__GetGraspCandidates_Response__original_indices(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__original_indices(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__original_indices(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__fetch_function__GetGraspCandidates_Response__original_indices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__original_indices(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__assign_function__GetGraspCandidates_Response__original_indices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__original_indices(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__resize_function__GetGraspCandidates_Response__original_indices(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_member_array[8] = {
  {
    "grasp_poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response, grasp_poses),  // bytes offset in struct
    NULL,  // default value
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__size_function__GetGraspCandidates_Response__grasp_poses,  // size() function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__grasp_poses,  // get_const(index) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__grasp_poses,  // get(index) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__fetch_function__GetGraspCandidates_Response__grasp_poses,  // fetch(index, &value) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__assign_function__GetGraspCandidates_Response__grasp_poses,  // assign(index, value) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__resize_function__GetGraspCandidates_Response__grasp_poses  // resize(index) function pointer
  },
  {
    "confidence_scores",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response, confidence_scores),  // bytes offset in struct
    NULL,  // default value
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__size_function__GetGraspCandidates_Response__confidence_scores,  // size() function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__confidence_scores,  // get_const(index) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__confidence_scores,  // get(index) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__fetch_function__GetGraspCandidates_Response__confidence_scores,  // fetch(index, &value) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__assign_function__GetGraspCandidates_Response__confidence_scores,  // assign(index, value) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__resize_function__GetGraspCandidates_Response__confidence_scores  // resize(index) function pointer
  },
  {
    "grasp_widths",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response, grasp_widths),  // bytes offset in struct
    NULL,  // default value
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__size_function__GetGraspCandidates_Response__grasp_widths,  // size() function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__grasp_widths,  // get_const(index) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__grasp_widths,  // get(index) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__fetch_function__GetGraspCandidates_Response__grasp_widths,  // fetch(index, &value) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__assign_function__GetGraspCandidates_Response__grasp_widths,  // assign(index, value) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__resize_function__GetGraspCandidates_Response__grasp_widths  // resize(index) function pointer
  },
  {
    "quality_scores",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response, quality_scores),  // bytes offset in struct
    NULL,  // default value
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__size_function__GetGraspCandidates_Response__quality_scores,  // size() function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__quality_scores,  // get_const(index) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__quality_scores,  // get(index) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__fetch_function__GetGraspCandidates_Response__quality_scores,  // fetch(index, &value) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__assign_function__GetGraspCandidates_Response__quality_scores,  // assign(index, value) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__resize_function__GetGraspCandidates_Response__quality_scores  // resize(index) function pointer
  },
  {
    "original_indices",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response, original_indices),  // bytes offset in struct
    NULL,  // default value
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__size_function__GetGraspCandidates_Response__original_indices,  // size() function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_const_function__GetGraspCandidates_Response__original_indices,  // get_const(index) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__get_function__GetGraspCandidates_Response__original_indices,  // get(index) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__fetch_function__GetGraspCandidates_Response__original_indices,  // fetch(index, &value) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__assign_function__GetGraspCandidates_Response__original_indices,  // assign(index, value) function pointer
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__resize_function__GetGraspCandidates_Response__original_indices  // resize(index) function pointer
  },
  {
    "detection_status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response, detection_status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total_grasps_detected",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response, total_grasps_detected),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "detection_timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response, detection_timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_members = {
  "roarm_anygrasp_integration__srv",  // message namespace
  "GetGraspCandidates_Response",  // message name
  8,  // number of fields
  sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response),
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_member_array,  // message members
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_type_support_handle = {
  0,
  &roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, GetGraspCandidates_Response)() {
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseStamped)();
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_type_support_handle.typesupport_identifier) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &roarm_anygrasp_integration__srv__GetGraspCandidates_Response__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "roarm_anygrasp_integration/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers roarm_anygrasp_integration__srv__detail__get_grasp_candidates__rosidl_typesupport_introspection_c__GetGraspCandidates_service_members = {
  "roarm_anygrasp_integration__srv",  // service namespace
  "GetGraspCandidates",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // roarm_anygrasp_integration__srv__detail__get_grasp_candidates__rosidl_typesupport_introspection_c__GetGraspCandidates_Request_message_type_support_handle,
  NULL  // response message
  // roarm_anygrasp_integration__srv__detail__get_grasp_candidates__rosidl_typesupport_introspection_c__GetGraspCandidates_Response_message_type_support_handle
};

static rosidl_service_type_support_t roarm_anygrasp_integration__srv__detail__get_grasp_candidates__rosidl_typesupport_introspection_c__GetGraspCandidates_service_type_support_handle = {
  0,
  &roarm_anygrasp_integration__srv__detail__get_grasp_candidates__rosidl_typesupport_introspection_c__GetGraspCandidates_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, GetGraspCandidates_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, GetGraspCandidates_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_anygrasp_integration
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, GetGraspCandidates)() {
  if (!roarm_anygrasp_integration__srv__detail__get_grasp_candidates__rosidl_typesupport_introspection_c__GetGraspCandidates_service_type_support_handle.typesupport_identifier) {
    roarm_anygrasp_integration__srv__detail__get_grasp_candidates__rosidl_typesupport_introspection_c__GetGraspCandidates_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)roarm_anygrasp_integration__srv__detail__get_grasp_candidates__rosidl_typesupport_introspection_c__GetGraspCandidates_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, GetGraspCandidates_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, GetGraspCandidates_Response)()->data;
  }

  return &roarm_anygrasp_integration__srv__detail__get_grasp_candidates__rosidl_typesupport_introspection_c__GetGraspCandidates_service_type_support_handle;
}
