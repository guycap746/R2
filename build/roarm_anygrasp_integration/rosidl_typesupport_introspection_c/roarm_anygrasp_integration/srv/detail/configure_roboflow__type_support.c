// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__rosidl_typesupport_introspection_c.h"
#include "roarm_anygrasp_integration/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__functions.h"
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__struct.h"


// Include directives for member types
// Member `api_key`
// Member `workspace_name`
// Member `project_name`
// Member `dataset_version`
// Member `annotation_format`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__init(message_memory);
}

void roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_fini_function(void * message_memory)
{
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_message_member_array[8] = {
  {
    "api_key",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request, api_key),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "workspace_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request, workspace_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "project_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request, project_name),  // bytes offset in struct
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
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request, dataset_version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "auto_upload",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request, auto_upload),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "include_failed_grasps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request, include_failed_grasps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "annotation_format",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request, annotation_format),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "min_confidence_for_upload",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request, min_confidence_for_upload),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_message_members = {
  "roarm_anygrasp_integration__srv",  // message namespace
  "ConfigureRoboflow_Request",  // message name
  8,  // number of fields
  sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request),
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_message_member_array,  // message members
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_message_type_support_handle = {
  0,
  &roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, ConfigureRoboflow_Request)() {
  if (!roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_message_type_support_handle.typesupport_identifier) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "roarm_anygrasp_integration/srv/detail/configure_roboflow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "roarm_anygrasp_integration/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/configure_roboflow__functions.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/configure_roboflow__struct.h"


// Include directives for member types
// Member `message`
// Member `workspace_url`
// Member `project_url`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__init(message_memory);
}

void roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_fini_function(void * message_memory)
{
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_message_member_array[5] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response, success),  // bytes offset in struct
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
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "workspace_url",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response, workspace_url),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "project_url",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response, project_url),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "total_images_in_dataset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response, total_images_in_dataset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_message_members = {
  "roarm_anygrasp_integration__srv",  // message namespace
  "ConfigureRoboflow_Response",  // message name
  5,  // number of fields
  sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response),
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_message_member_array,  // message members
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_message_type_support_handle = {
  0,
  &roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, ConfigureRoboflow_Response)() {
  if (!roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_message_type_support_handle.typesupport_identifier) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "roarm_anygrasp_integration/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/configure_roboflow__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers roarm_anygrasp_integration__srv__detail__configure_roboflow__rosidl_typesupport_introspection_c__ConfigureRoboflow_service_members = {
  "roarm_anygrasp_integration__srv",  // service namespace
  "ConfigureRoboflow",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // roarm_anygrasp_integration__srv__detail__configure_roboflow__rosidl_typesupport_introspection_c__ConfigureRoboflow_Request_message_type_support_handle,
  NULL  // response message
  // roarm_anygrasp_integration__srv__detail__configure_roboflow__rosidl_typesupport_introspection_c__ConfigureRoboflow_Response_message_type_support_handle
};

static rosidl_service_type_support_t roarm_anygrasp_integration__srv__detail__configure_roboflow__rosidl_typesupport_introspection_c__ConfigureRoboflow_service_type_support_handle = {
  0,
  &roarm_anygrasp_integration__srv__detail__configure_roboflow__rosidl_typesupport_introspection_c__ConfigureRoboflow_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, ConfigureRoboflow_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, ConfigureRoboflow_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_roarm_anygrasp_integration
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, ConfigureRoboflow)() {
  if (!roarm_anygrasp_integration__srv__detail__configure_roboflow__rosidl_typesupport_introspection_c__ConfigureRoboflow_service_type_support_handle.typesupport_identifier) {
    roarm_anygrasp_integration__srv__detail__configure_roboflow__rosidl_typesupport_introspection_c__ConfigureRoboflow_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)roarm_anygrasp_integration__srv__detail__configure_roboflow__rosidl_typesupport_introspection_c__ConfigureRoboflow_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, ConfigureRoboflow_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, roarm_anygrasp_integration, srv, ConfigureRoboflow_Response)()->data;
  }

  return &roarm_anygrasp_integration__srv__detail__configure_roboflow__rosidl_typesupport_introspection_c__ConfigureRoboflow_service_type_support_handle;
}
