// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from roarm_anygrasp_integration:srv/SelectGrasp.idl
// generated code does not contain a copyright notice
#include "roarm_anygrasp_integration/srv/detail/select_grasp__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "roarm_anygrasp_integration/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "roarm_anygrasp_integration/srv/detail/select_grasp__struct.h"
#include "roarm_anygrasp_integration/srv/detail/select_grasp__functions.h"
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


// forward declare type support functions


using _SelectGrasp_Request__ros_msg_type = roarm_anygrasp_integration__srv__SelectGrasp_Request;

static bool _SelectGrasp_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SelectGrasp_Request__ros_msg_type * ros_message = static_cast<const _SelectGrasp_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: selected_grasp_index
  {
    cdr << ros_message->selected_grasp_index;
  }

  // Field name: show_more_candidates
  {
    cdr << (ros_message->show_more_candidates ? true : false);
  }

  return true;
}

static bool _SelectGrasp_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SelectGrasp_Request__ros_msg_type * ros_message = static_cast<_SelectGrasp_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: selected_grasp_index
  {
    cdr >> ros_message->selected_grasp_index;
  }

  // Field name: show_more_candidates
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->show_more_candidates = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t get_serialized_size_roarm_anygrasp_integration__srv__SelectGrasp_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SelectGrasp_Request__ros_msg_type * ros_message = static_cast<const _SelectGrasp_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name selected_grasp_index
  {
    size_t item_size = sizeof(ros_message->selected_grasp_index);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name show_more_candidates
  {
    size_t item_size = sizeof(ros_message->show_more_candidates);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SelectGrasp_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_roarm_anygrasp_integration__srv__SelectGrasp_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t max_serialized_size_roarm_anygrasp_integration__srv__SelectGrasp_Request(
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

  // member: selected_grasp_index
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: show_more_candidates
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = roarm_anygrasp_integration__srv__SelectGrasp_Request;
    is_plain =
      (
      offsetof(DataType, show_more_candidates) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SelectGrasp_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_roarm_anygrasp_integration__srv__SelectGrasp_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SelectGrasp_Request = {
  "roarm_anygrasp_integration::srv",
  "SelectGrasp_Request",
  _SelectGrasp_Request__cdr_serialize,
  _SelectGrasp_Request__cdr_deserialize,
  _SelectGrasp_Request__get_serialized_size,
  _SelectGrasp_Request__max_serialized_size
};

static rosidl_message_type_support_t _SelectGrasp_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SelectGrasp_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, SelectGrasp_Request)() {
  return &_SelectGrasp_Request__type_support;
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
// #include "roarm_anygrasp_integration/srv/detail/select_grasp__struct.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/select_grasp__functions.h"
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

#include "geometry_msgs/msg/detail/pose_stamped__functions.h"  // selected_pose
#include "rosidl_runtime_c/string.h"  // message
#include "rosidl_runtime_c/string_functions.h"  // message

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
size_t get_serialized_size_geometry_msgs__msg__PoseStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
size_t max_serialized_size_geometry_msgs__msg__PoseStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped)();


using _SelectGrasp_Response__ros_msg_type = roarm_anygrasp_integration__srv__SelectGrasp_Response;

static bool _SelectGrasp_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SelectGrasp_Response__ros_msg_type * ros_message = static_cast<const _SelectGrasp_Response__ros_msg_type *>(untyped_ros_message);
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

  // Field name: selected_pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->selected_pose, cdr))
    {
      return false;
    }
  }

  // Field name: confidence_score
  {
    cdr << ros_message->confidence_score;
  }

  // Field name: grasp_width
  {
    cdr << ros_message->grasp_width;
  }

  return true;
}

static bool _SelectGrasp_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SelectGrasp_Response__ros_msg_type * ros_message = static_cast<_SelectGrasp_Response__ros_msg_type *>(untyped_ros_message);
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

  // Field name: selected_pose
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->selected_pose))
    {
      return false;
    }
  }

  // Field name: confidence_score
  {
    cdr >> ros_message->confidence_score;
  }

  // Field name: grasp_width
  {
    cdr >> ros_message->grasp_width;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t get_serialized_size_roarm_anygrasp_integration__srv__SelectGrasp_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SelectGrasp_Response__ros_msg_type * ros_message = static_cast<const _SelectGrasp_Response__ros_msg_type *>(untyped_ros_message);
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
  // field.name selected_pose

  current_alignment += get_serialized_size_geometry_msgs__msg__PoseStamped(
    &(ros_message->selected_pose), current_alignment);
  // field.name confidence_score
  {
    size_t item_size = sizeof(ros_message->confidence_score);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name grasp_width
  {
    size_t item_size = sizeof(ros_message->grasp_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SelectGrasp_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_roarm_anygrasp_integration__srv__SelectGrasp_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t max_serialized_size_roarm_anygrasp_integration__srv__SelectGrasp_Response(
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
  // member: selected_pose
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: confidence_score
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: grasp_width
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
    using DataType = roarm_anygrasp_integration__srv__SelectGrasp_Response;
    is_plain =
      (
      offsetof(DataType, grasp_width) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _SelectGrasp_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_roarm_anygrasp_integration__srv__SelectGrasp_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SelectGrasp_Response = {
  "roarm_anygrasp_integration::srv",
  "SelectGrasp_Response",
  _SelectGrasp_Response__cdr_serialize,
  _SelectGrasp_Response__cdr_deserialize,
  _SelectGrasp_Response__get_serialized_size,
  _SelectGrasp_Response__max_serialized_size
};

static rosidl_message_type_support_t _SelectGrasp_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SelectGrasp_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, SelectGrasp_Response)() {
  return &_SelectGrasp_Response__type_support;
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
#include "roarm_anygrasp_integration/srv/select_grasp.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SelectGrasp__callbacks = {
  "roarm_anygrasp_integration::srv",
  "SelectGrasp",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, SelectGrasp_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, SelectGrasp_Response)(),
};

static rosidl_service_type_support_t SelectGrasp__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SelectGrasp__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, SelectGrasp)() {
  return &SelectGrasp__handle;
}

#if defined(__cplusplus)
}
#endif
