// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
// generated code does not contain a copyright notice
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__rosidl_typesupport_fastrtps_cpp.hpp"
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace roarm_anygrasp_integration
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration
cdr_serialize(
  const roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: api_key
  cdr << ros_message.api_key;
  // Member: workspace_name
  cdr << ros_message.workspace_name;
  // Member: project_name
  cdr << ros_message.project_name;
  // Member: dataset_version
  cdr << ros_message.dataset_version;
  // Member: auto_upload
  cdr << (ros_message.auto_upload ? true : false);
  // Member: include_failed_grasps
  cdr << (ros_message.include_failed_grasps ? true : false);
  // Member: annotation_format
  cdr << ros_message.annotation_format;
  // Member: min_confidence_for_upload
  cdr << ros_message.min_confidence_for_upload;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & ros_message)
{
  // Member: api_key
  cdr >> ros_message.api_key;

  // Member: workspace_name
  cdr >> ros_message.workspace_name;

  // Member: project_name
  cdr >> ros_message.project_name;

  // Member: dataset_version
  cdr >> ros_message.dataset_version;

  // Member: auto_upload
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.auto_upload = tmp ? true : false;
  }

  // Member: include_failed_grasps
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.include_failed_grasps = tmp ? true : false;
  }

  // Member: annotation_format
  cdr >> ros_message.annotation_format;

  // Member: min_confidence_for_upload
  cdr >> ros_message.min_confidence_for_upload;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration
get_serialized_size(
  const roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: api_key
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.api_key.size() + 1);
  // Member: workspace_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.workspace_name.size() + 1);
  // Member: project_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.project_name.size() + 1);
  // Member: dataset_version
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.dataset_version.size() + 1);
  // Member: auto_upload
  {
    size_t item_size = sizeof(ros_message.auto_upload);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: include_failed_grasps
  {
    size_t item_size = sizeof(ros_message.include_failed_grasps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: annotation_format
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.annotation_format.size() + 1);
  // Member: min_confidence_for_upload
  {
    size_t item_size = sizeof(ros_message.min_confidence_for_upload);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration
max_serialized_size_ConfigureRoboflow_Request(
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


  // Member: api_key
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

  // Member: workspace_name
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

  // Member: project_name
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

  // Member: dataset_version
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

  // Member: auto_upload
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: include_failed_grasps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: annotation_format
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

  // Member: min_confidence_for_upload
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
    using DataType = roarm_anygrasp_integration::srv::ConfigureRoboflow_Request;
    is_plain =
      (
      offsetof(DataType, min_confidence_for_upload) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ConfigureRoboflow_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const roarm_anygrasp_integration::srv::ConfigureRoboflow_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ConfigureRoboflow_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ConfigureRoboflow_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const roarm_anygrasp_integration::srv::ConfigureRoboflow_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ConfigureRoboflow_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ConfigureRoboflow_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ConfigureRoboflow_Request__callbacks = {
  "roarm_anygrasp_integration::srv",
  "ConfigureRoboflow_Request",
  _ConfigureRoboflow_Request__cdr_serialize,
  _ConfigureRoboflow_Request__cdr_deserialize,
  _ConfigureRoboflow_Request__get_serialized_size,
  _ConfigureRoboflow_Request__max_serialized_size
};

static rosidl_message_type_support_t _ConfigureRoboflow_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ConfigureRoboflow_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace roarm_anygrasp_integration

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
get_message_type_support_handle<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request>()
{
  return &roarm_anygrasp_integration::srv::typesupport_fastrtps_cpp::_ConfigureRoboflow_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, roarm_anygrasp_integration, srv, ConfigureRoboflow_Request)() {
  return &roarm_anygrasp_integration::srv::typesupport_fastrtps_cpp::_ConfigureRoboflow_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace roarm_anygrasp_integration
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration
cdr_serialize(
  const roarm_anygrasp_integration::srv::ConfigureRoboflow_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: success
  cdr << (ros_message.success ? true : false);
  // Member: message
  cdr << ros_message.message;
  // Member: workspace_url
  cdr << ros_message.workspace_url;
  // Member: project_url
  cdr << ros_message.project_url;
  // Member: total_images_in_dataset
  cdr << ros_message.total_images_in_dataset;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  roarm_anygrasp_integration::srv::ConfigureRoboflow_Response & ros_message)
{
  // Member: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.success = tmp ? true : false;
  }

  // Member: message
  cdr >> ros_message.message;

  // Member: workspace_url
  cdr >> ros_message.workspace_url;

  // Member: project_url
  cdr >> ros_message.project_url;

  // Member: total_images_in_dataset
  cdr >> ros_message.total_images_in_dataset;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration
get_serialized_size(
  const roarm_anygrasp_integration::srv::ConfigureRoboflow_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: success
  {
    size_t item_size = sizeof(ros_message.success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: message
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.message.size() + 1);
  // Member: workspace_url
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.workspace_url.size() + 1);
  // Member: project_url
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.project_url.size() + 1);
  // Member: total_images_in_dataset
  {
    size_t item_size = sizeof(ros_message.total_images_in_dataset);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration
max_serialized_size_ConfigureRoboflow_Response(
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


  // Member: success
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: message
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

  // Member: workspace_url
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

  // Member: project_url
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

  // Member: total_images_in_dataset
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
    using DataType = roarm_anygrasp_integration::srv::ConfigureRoboflow_Response;
    is_plain =
      (
      offsetof(DataType, total_images_in_dataset) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ConfigureRoboflow_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const roarm_anygrasp_integration::srv::ConfigureRoboflow_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ConfigureRoboflow_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ConfigureRoboflow_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const roarm_anygrasp_integration::srv::ConfigureRoboflow_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ConfigureRoboflow_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ConfigureRoboflow_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ConfigureRoboflow_Response__callbacks = {
  "roarm_anygrasp_integration::srv",
  "ConfigureRoboflow_Response",
  _ConfigureRoboflow_Response__cdr_serialize,
  _ConfigureRoboflow_Response__cdr_deserialize,
  _ConfigureRoboflow_Response__get_serialized_size,
  _ConfigureRoboflow_Response__max_serialized_size
};

static rosidl_message_type_support_t _ConfigureRoboflow_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ConfigureRoboflow_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace roarm_anygrasp_integration

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
get_message_type_support_handle<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response>()
{
  return &roarm_anygrasp_integration::srv::typesupport_fastrtps_cpp::_ConfigureRoboflow_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, roarm_anygrasp_integration, srv, ConfigureRoboflow_Response)() {
  return &roarm_anygrasp_integration::srv::typesupport_fastrtps_cpp::_ConfigureRoboflow_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace roarm_anygrasp_integration
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _ConfigureRoboflow__callbacks = {
  "roarm_anygrasp_integration::srv",
  "ConfigureRoboflow",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, roarm_anygrasp_integration, srv, ConfigureRoboflow_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, roarm_anygrasp_integration, srv, ConfigureRoboflow_Response)(),
};

static rosidl_service_type_support_t _ConfigureRoboflow__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ConfigureRoboflow__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace roarm_anygrasp_integration

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_roarm_anygrasp_integration
const rosidl_service_type_support_t *
get_service_type_support_handle<roarm_anygrasp_integration::srv::ConfigureRoboflow>()
{
  return &roarm_anygrasp_integration::srv::typesupport_fastrtps_cpp::_ConfigureRoboflow__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, roarm_anygrasp_integration, srv, ConfigureRoboflow)() {
  return &roarm_anygrasp_integration::srv::typesupport_fastrtps_cpp::_ConfigureRoboflow__handle;
}

#ifdef __cplusplus
}
#endif
