// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
// generated code does not contain a copyright notice
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "roarm_anygrasp_integration/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__struct.h"
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // annotation_format, api_key, dataset_version, project_name, workspace_name
#include "rosidl_runtime_c/string_functions.h"  // annotation_format, api_key, dataset_version, project_name, workspace_name

// forward declare type support functions


using _ConfigureRoboflow_Request__ros_msg_type = roarm_anygrasp_integration__srv__ConfigureRoboflow_Request;

static bool _ConfigureRoboflow_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ConfigureRoboflow_Request__ros_msg_type * ros_message = static_cast<const _ConfigureRoboflow_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: api_key
  {
    const rosidl_runtime_c__String * str = &ros_message->api_key;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: workspace_name
  {
    const rosidl_runtime_c__String * str = &ros_message->workspace_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: project_name
  {
    const rosidl_runtime_c__String * str = &ros_message->project_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: dataset_version
  {
    const rosidl_runtime_c__String * str = &ros_message->dataset_version;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: auto_upload
  {
    cdr << (ros_message->auto_upload ? true : false);
  }

  // Field name: include_failed_grasps
  {
    cdr << (ros_message->include_failed_grasps ? true : false);
  }

  // Field name: annotation_format
  {
    const rosidl_runtime_c__String * str = &ros_message->annotation_format;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: min_confidence_for_upload
  {
    cdr << ros_message->min_confidence_for_upload;
  }

  return true;
}

static bool _ConfigureRoboflow_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ConfigureRoboflow_Request__ros_msg_type * ros_message = static_cast<_ConfigureRoboflow_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: api_key
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->api_key.data) {
      rosidl_runtime_c__String__init(&ros_message->api_key);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->api_key,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'api_key'\n");
      return false;
    }
  }

  // Field name: workspace_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->workspace_name.data) {
      rosidl_runtime_c__String__init(&ros_message->workspace_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->workspace_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'workspace_name'\n");
      return false;
    }
  }

  // Field name: project_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->project_name.data) {
      rosidl_runtime_c__String__init(&ros_message->project_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->project_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'project_name'\n");
      return false;
    }
  }

  // Field name: dataset_version
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->dataset_version.data) {
      rosidl_runtime_c__String__init(&ros_message->dataset_version);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->dataset_version,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'dataset_version'\n");
      return false;
    }
  }

  // Field name: auto_upload
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->auto_upload = tmp ? true : false;
  }

  // Field name: include_failed_grasps
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->include_failed_grasps = tmp ? true : false;
  }

  // Field name: annotation_format
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->annotation_format.data) {
      rosidl_runtime_c__String__init(&ros_message->annotation_format);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->annotation_format,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'annotation_format'\n");
      return false;
    }
  }

  // Field name: min_confidence_for_upload
  {
    cdr >> ros_message->min_confidence_for_upload;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t get_serialized_size_roarm_anygrasp_integration__srv__ConfigureRoboflow_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ConfigureRoboflow_Request__ros_msg_type * ros_message = static_cast<const _ConfigureRoboflow_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name api_key
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->api_key.size + 1);
  // field.name workspace_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->workspace_name.size + 1);
  // field.name project_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->project_name.size + 1);
  // field.name dataset_version
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->dataset_version.size + 1);
  // field.name auto_upload
  {
    size_t item_size = sizeof(ros_message->auto_upload);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name include_failed_grasps
  {
    size_t item_size = sizeof(ros_message->include_failed_grasps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name annotation_format
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->annotation_format.size + 1);
  // field.name min_confidence_for_upload
  {
    size_t item_size = sizeof(ros_message->min_confidence_for_upload);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ConfigureRoboflow_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_roarm_anygrasp_integration__srv__ConfigureRoboflow_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t max_serialized_size_roarm_anygrasp_integration__srv__ConfigureRoboflow_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: api_key
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: workspace_name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: project_name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: dataset_version
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: auto_upload
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: include_failed_grasps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: annotation_format
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: min_confidence_for_upload
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = roarm_anygrasp_integration__srv__ConfigureRoboflow_Request;
    is_plain =
      (
      offsetof(DataType, min_confidence_for_upload) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ConfigureRoboflow_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_roarm_anygrasp_integration__srv__ConfigureRoboflow_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ConfigureRoboflow_Request = {
  "roarm_anygrasp_integration::srv",
  "ConfigureRoboflow_Request",
  _ConfigureRoboflow_Request__cdr_serialize,
  _ConfigureRoboflow_Request__cdr_deserialize,
  _ConfigureRoboflow_Request__get_serialized_size,
  _ConfigureRoboflow_Request__max_serialized_size
};

static rosidl_message_type_support_t _ConfigureRoboflow_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ConfigureRoboflow_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, ConfigureRoboflow_Request)() {
  return &_ConfigureRoboflow_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "roarm_anygrasp_integration/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/configure_roboflow__struct.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/configure_roboflow__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/string.h"  // message, project_url, workspace_url
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // message, project_url, workspace_url

// forward declare type support functions


using _ConfigureRoboflow_Response__ros_msg_type = roarm_anygrasp_integration__srv__ConfigureRoboflow_Response;

static bool _ConfigureRoboflow_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ConfigureRoboflow_Response__ros_msg_type * ros_message = static_cast<const _ConfigureRoboflow_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: message
  {
    const rosidl_runtime_c__String * str = &ros_message->message;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: workspace_url
  {
    const rosidl_runtime_c__String * str = &ros_message->workspace_url;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: project_url
  {
    const rosidl_runtime_c__String * str = &ros_message->project_url;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: total_images_in_dataset
  {
    cdr << ros_message->total_images_in_dataset;
  }

  return true;
}

static bool _ConfigureRoboflow_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ConfigureRoboflow_Response__ros_msg_type * ros_message = static_cast<_ConfigureRoboflow_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: message
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message.data) {
      rosidl_runtime_c__String__init(&ros_message->message);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message'\n");
      return false;
    }
  }

  // Field name: workspace_url
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->workspace_url.data) {
      rosidl_runtime_c__String__init(&ros_message->workspace_url);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->workspace_url,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'workspace_url'\n");
      return false;
    }
  }

  // Field name: project_url
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->project_url.data) {
      rosidl_runtime_c__String__init(&ros_message->project_url);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->project_url,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'project_url'\n");
      return false;
    }
  }

  // Field name: total_images_in_dataset
  {
    cdr >> ros_message->total_images_in_dataset;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t get_serialized_size_roarm_anygrasp_integration__srv__ConfigureRoboflow_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ConfigureRoboflow_Response__ros_msg_type * ros_message = static_cast<const _ConfigureRoboflow_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message.size + 1);
  // field.name workspace_url
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->workspace_url.size + 1);
  // field.name project_url
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->project_url.size + 1);
  // field.name total_images_in_dataset
  {
    size_t item_size = sizeof(ros_message->total_images_in_dataset);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ConfigureRoboflow_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_roarm_anygrasp_integration__srv__ConfigureRoboflow_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t max_serialized_size_roarm_anygrasp_integration__srv__ConfigureRoboflow_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: message
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: workspace_url
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: project_url
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: total_images_in_dataset
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = roarm_anygrasp_integration__srv__ConfigureRoboflow_Response;
    is_plain =
      (
      offsetof(DataType, total_images_in_dataset) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ConfigureRoboflow_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_roarm_anygrasp_integration__srv__ConfigureRoboflow_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ConfigureRoboflow_Response = {
  "roarm_anygrasp_integration::srv",
  "ConfigureRoboflow_Response",
  _ConfigureRoboflow_Response__cdr_serialize,
  _ConfigureRoboflow_Response__cdr_deserialize,
  _ConfigureRoboflow_Response__get_serialized_size,
  _ConfigureRoboflow_Response__max_serialized_size
};

static rosidl_message_type_support_t _ConfigureRoboflow_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ConfigureRoboflow_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, ConfigureRoboflow_Response)() {
  return &_ConfigureRoboflow_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "roarm_anygrasp_integration/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "roarm_anygrasp_integration/srv/configure_roboflow.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t ConfigureRoboflow__callbacks = {
  "roarm_anygrasp_integration::srv",
  "ConfigureRoboflow",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, ConfigureRoboflow_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, ConfigureRoboflow_Response)(),
};

static rosidl_service_type_support_t ConfigureRoboflow__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &ConfigureRoboflow__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, ConfigureRoboflow)() {
  return &ConfigureRoboflow__handle;
}

#if defined(__cplusplus)
}
#endif
