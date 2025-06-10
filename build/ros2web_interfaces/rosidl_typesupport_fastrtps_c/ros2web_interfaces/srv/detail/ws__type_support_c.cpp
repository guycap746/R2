// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ros2web_interfaces:srv/WS.idl
// generated code does not contain a copyright notice
#include "ros2web_interfaces/srv/detail/ws__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ros2web_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2web_interfaces/srv/detail/ws__struct.h"
#include "ros2web_interfaces/srv/detail/ws__functions.h"
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

#include "ros2web_interfaces/msg/detail/ws_msg_data__functions.h"  // data
#include "rosidl_runtime_c/string.h"  // extra, route, srv_name, ws_id
#include "rosidl_runtime_c/string_functions.h"  // extra, route, srv_name, ws_id

// forward declare type support functions
size_t get_serialized_size_ros2web_interfaces__msg__WSMsgData(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2web_interfaces__msg__WSMsgData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, WSMsgData)();


using _WS_Request__ros_msg_type = ros2web_interfaces__srv__WS_Request;

static bool _WS_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _WS_Request__ros_msg_type * ros_message = static_cast<const _WS_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: srv_name
  {
    const rosidl_runtime_c__String * str = &ros_message->srv_name;
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

  // Field name: route
  {
    const rosidl_runtime_c__String * str = &ros_message->route;
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

  // Field name: type
  {
    cdr << ros_message->type;
  }

  // Field name: ws_id
  {
    const rosidl_runtime_c__String * str = &ros_message->ws_id;
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

  // Field name: data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, WSMsgData
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->data, cdr))
    {
      return false;
    }
  }

  // Field name: extra
  {
    const rosidl_runtime_c__String * str = &ros_message->extra;
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

  return true;
}

static bool _WS_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _WS_Request__ros_msg_type * ros_message = static_cast<_WS_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: srv_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->srv_name.data) {
      rosidl_runtime_c__String__init(&ros_message->srv_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->srv_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'srv_name'\n");
      return false;
    }
  }

  // Field name: route
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->route.data) {
      rosidl_runtime_c__String__init(&ros_message->route);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->route,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'route'\n");
      return false;
    }
  }

  // Field name: type
  {
    cdr >> ros_message->type;
  }

  // Field name: ws_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->ws_id.data) {
      rosidl_runtime_c__String__init(&ros_message->ws_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->ws_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'ws_id'\n");
      return false;
    }
  }

  // Field name: data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, WSMsgData
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->data))
    {
      return false;
    }
  }

  // Field name: extra
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->extra.data) {
      rosidl_runtime_c__String__init(&ros_message->extra);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->extra,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'extra'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2web_interfaces
size_t get_serialized_size_ros2web_interfaces__srv__WS_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _WS_Request__ros_msg_type * ros_message = static_cast<const _WS_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name srv_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->srv_name.size + 1);
  // field.name route
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->route.size + 1);
  // field.name type
  {
    size_t item_size = sizeof(ros_message->type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ws_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->ws_id.size + 1);
  // field.name data

  current_alignment += get_serialized_size_ros2web_interfaces__msg__WSMsgData(
    &(ros_message->data), current_alignment);
  // field.name extra
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->extra.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _WS_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2web_interfaces__srv__WS_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2web_interfaces
size_t max_serialized_size_ros2web_interfaces__srv__WS_Request(
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

  // member: srv_name
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
  // member: route
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
  // member: type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ws_id
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
  // member: data
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2web_interfaces__msg__WSMsgData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: extra
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2web_interfaces__srv__WS_Request;
    is_plain =
      (
      offsetof(DataType, extra) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _WS_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2web_interfaces__srv__WS_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_WS_Request = {
  "ros2web_interfaces::srv",
  "WS_Request",
  _WS_Request__cdr_serialize,
  _WS_Request__cdr_deserialize,
  _WS_Request__get_serialized_size,
  _WS_Request__max_serialized_size
};

static rosidl_message_type_support_t _WS_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_WS_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, srv, WS_Request)() {
  return &_WS_Request__type_support;
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
// #include "ros2web_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "ros2web_interfaces/srv/detail/ws__struct.h"
// already included above
// #include "ros2web_interfaces/srv/detail/ws__functions.h"
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
// #include "ros2web_interfaces/msg/detail/ws_msg_data__functions.h"  // data
// already included above
// #include "rosidl_runtime_c/string.h"  // extra, ws_id
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // extra, ws_id

// forward declare type support functions
size_t get_serialized_size_ros2web_interfaces__msg__WSMsgData(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_ros2web_interfaces__msg__WSMsgData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, WSMsgData)();


using _WS_Response__ros_msg_type = ros2web_interfaces__srv__WS_Response;

static bool _WS_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _WS_Response__ros_msg_type * ros_message = static_cast<const _WS_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: type
  {
    cdr << ros_message->type;
  }

  // Field name: ws_id
  {
    const rosidl_runtime_c__String * str = &ros_message->ws_id;
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

  // Field name: data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, WSMsgData
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->data, cdr))
    {
      return false;
    }
  }

  // Field name: extra
  {
    const rosidl_runtime_c__String * str = &ros_message->extra;
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

  return true;
}

static bool _WS_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _WS_Response__ros_msg_type * ros_message = static_cast<_WS_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: type
  {
    cdr >> ros_message->type;
  }

  // Field name: ws_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->ws_id.data) {
      rosidl_runtime_c__String__init(&ros_message->ws_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->ws_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'ws_id'\n");
      return false;
    }
  }

  // Field name: data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, ros2web_interfaces, msg, WSMsgData
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->data))
    {
      return false;
    }
  }

  // Field name: extra
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->extra.data) {
      rosidl_runtime_c__String__init(&ros_message->extra);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->extra,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'extra'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2web_interfaces
size_t get_serialized_size_ros2web_interfaces__srv__WS_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _WS_Response__ros_msg_type * ros_message = static_cast<const _WS_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name type
  {
    size_t item_size = sizeof(ros_message->type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ws_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->ws_id.size + 1);
  // field.name data

  current_alignment += get_serialized_size_ros2web_interfaces__msg__WSMsgData(
    &(ros_message->data), current_alignment);
  // field.name extra
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->extra.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _WS_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ros2web_interfaces__srv__WS_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ros2web_interfaces
size_t max_serialized_size_ros2web_interfaces__srv__WS_Response(
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

  // member: type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ws_id
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
  // member: data
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_ros2web_interfaces__msg__WSMsgData(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: extra
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros2web_interfaces__srv__WS_Response;
    is_plain =
      (
      offsetof(DataType, extra) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _WS_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ros2web_interfaces__srv__WS_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_WS_Response = {
  "ros2web_interfaces::srv",
  "WS_Response",
  _WS_Response__cdr_serialize,
  _WS_Response__cdr_deserialize,
  _WS_Response__get_serialized_size,
  _WS_Response__max_serialized_size
};

static rosidl_message_type_support_t _WS_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_WS_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, srv, WS_Response)() {
  return &_WS_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "ros2web_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ros2web_interfaces/srv/ws.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t WS__callbacks = {
  "ros2web_interfaces::srv",
  "WS",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, srv, WS_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, srv, WS_Response)(),
};

static rosidl_service_type_support_t WS__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &WS__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ros2web_interfaces, srv, WS)() {
  return &WS__handle;
}

#if defined(__cplusplus)
}
#endif
