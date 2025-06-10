// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from roarm_anygrasp_integration:srv/UploadToRoboflow.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__rosidl_typesupport_introspection_c.h"
#include "roarm_anygrasp_integration/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__functions.h"
#include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__struct.h"


// Include directives for member types
// Member `rgb_image`
#include "sensor_msgs/msg/image.h"
// Member `rgb_image`
#include "sensor_msgs/msg/detail/image__rosidl_typesupport_introspection_c.h"
// Member `grasp_poses`
#include "geometry_msgs/msg/pose_array.h"
// Member `grasp_poses`
#include "geometry_msgs/msg/detail/pose_array__rosidl_typesupport_introspection_c.h"
// Member `confidence_scores`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `class_labels`
// Member `scene_description`
// Member `object_types`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request__init(message_memory);
}

void roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_fini_function(void * message_memory)
{
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(message_memory);
}

size_t roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__size_function__UploadToRoboflow_Request__confidence_scores(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_const_function__UploadToRoboflow_Request__confidence_scores(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_function__UploadToRoboflow_Request__confidence_scores(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__fetch_function__UploadToRoboflow_Request__confidence_scores(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_const_function__UploadToRoboflow_Request__confidence_scores(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__assign_function__UploadToRoboflow_Request__confidence_scores(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_function__UploadToRoboflow_Request__confidence_scores(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__resize_function__UploadToRoboflow_Request__confidence_scores(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

size_t roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__size_function__UploadToRoboflow_Request__class_labels(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_const_function__UploadToRoboflow_Request__class_labels(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_function__UploadToRoboflow_Request__class_labels(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__fetch_function__UploadToRoboflow_Request__class_labels(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_const_function__UploadToRoboflow_Request__class_labels(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__assign_function__UploadToRoboflow_Request__class_labels(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_function__UploadToRoboflow_Request__class_labels(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__resize_function__UploadToRoboflow_Request__class_labels(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_member_array[8] = {
  {
    "rgb_image",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request, rgb_image),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "grasp_poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request, grasp_poses),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence_scores",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request, confidence_scores),  // bytes offset in struct
    NULL,  // default value
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__size_function__UploadToRoboflow_Request__confidence_scores,  // size() function pointer
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_const_function__UploadToRoboflow_Request__confidence_scores,  // get_const(index) function pointer
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_function__UploadToRoboflow_Request__confidence_scores,  // get(index) function pointer
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__fetch_function__UploadToRoboflow_Request__confidence_scores,  // fetch(index, &value) function pointer
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__assign_function__UploadToRoboflow_Request__confidence_scores,  // assign(index, value) function pointer
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__resize_function__UploadToRoboflow_Request__confidence_scores  // resize(index) function pointer
  },
  {
    "class_labels",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request, class_labels),  // bytes offset in struct
    NULL,  // default value
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__size_function__UploadToRoboflow_Request__class_labels,  // size() function pointer
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_const_function__UploadToRoboflow_Request__class_labels,  // get_const(index) function pointer
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__get_function__UploadToRoboflow_Request__class_labels,  // get(index) function pointer
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__fetch_function__UploadToRoboflow_Request__class_labels,  // fetch(index, &value) function pointer
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__assign_function__UploadToRoboflow_Request__class_labels,  // assign(index, value) function pointer
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__resize_function__UploadToRoboflow_Request__class_labels  // resize(index) function pointer
  },
  {
    "scene_description",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request, scene_description),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "object_types",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request, object_types),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "include_annotations",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request, include_annotations),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "upload_immediately",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request, upload_immediately),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_members = {
  "roarm_anygrasp_integration__srv",  // message namespace
  "UploadToRoboflow_Request",  // message name
  8,  // number of fields
  sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request),
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_member_array,  // message members
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_type_support_handle = {
  0,
  &roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, UploadToRoboflow_Request)() {
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Image)();
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, PoseArray)();
  if (!roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_type_support_handle.typesupport_identifier) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &roarm_anygrasp_integration__srv__UploadToRoboflow_Request__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "roarm_anygrasp_integration/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__functions.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__struct.h"


// Include directives for member types
// Member `message`
// Member `roboflow_image_id`
// Member `upload_url`
// Member `dataset_version`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response__init(message_memory);
}

void roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_fini_function(void * message_memory)
{
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_message_member_array[6] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "roboflow_image_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response, roboflow_image_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "upload_url",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response, upload_url),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "annotation_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response, annotation_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dataset_version",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response, dataset_version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_message_members = {
  "roarm_anygrasp_integration__srv",  // message namespace
  "UploadToRoboflow_Response",  // message name
  6,  // number of fields
  sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response),
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_message_member_array,  // message members
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_message_type_support_handle = {
  0,
  &roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, UploadToRoboflow_Response)() {
  if (!roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_message_type_support_handle.typesupport_identifier) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &roarm_anygrasp_integration__srv__UploadToRoboflow_Response__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "roarm_anygrasp_integration/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers roarm_anygrasp_integration__srv__detail__upload_to_roboflow__rosidl_typesupport_introspection_c__UploadToRoboflow_service_members = {
  "roarm_anygrasp_integration__srv",  // service namespace
  "UploadToRoboflow",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // roarm_anygrasp_integration__srv__detail__upload_to_roboflow__rosidl_typesupport_introspection_c__UploadToRoboflow_Request_message_type_support_handle,
  NULL  // response message
  // roarm_anygrasp_integration__srv__detail__upload_to_roboflow__rosidl_typesupport_introspection_c__UploadToRoboflow_Response_message_type_support_handle
};

static rosidl_service_type_support_t roarm_anygrasp_integration__srv__detail__upload_to_roboflow__rosidl_typesupport_introspection_c__UploadToRoboflow_service_type_support_handle = {
  0,
  &roarm_anygrasp_integration__srv__detail__upload_to_roboflow__rosidl_typesupport_introspection_c__UploadToRoboflow_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, UploadToRoboflow_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, UploadToRoboflow_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_anygrasp_integration
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, UploadToRoboflow)() {
  if (!roarm_anygrasp_integration__srv__detail__upload_to_roboflow__rosidl_typesupport_introspection_c__UploadToRoboflow_service_type_support_handle.typesupport_identifier) {
    roarm_anygrasp_integration__srv__detail__upload_to_roboflow__rosidl_typesupport_introspection_c__UploadToRoboflow_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)roarm_anygrasp_integration__srv__detail__upload_to_roboflow__rosidl_typesupport_introspection_c__UploadToRoboflow_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, UploadToRoboflow_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, UploadToRoboflow_Response)()->data;
  }

  return &roarm_anygrasp_integration__srv__detail__upload_to_roboflow__rosidl_typesupport_introspection_c__UploadToRoboflow_service_type_support_handle;
}
