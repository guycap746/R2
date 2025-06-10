// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from roarm_anygrasp_integration:srv/GetGraspCandidates.idl
// generated code does not contain a copyright notice
#include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "roarm_anygrasp_integration/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__struct.h"
#include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__functions.h"
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


using _GetGraspCandidates_Request__ros_msg_type = roarm_anygrasp_integration__srv__GetGraspCandidates_Request;

static bool _GetGraspCandidates_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetGraspCandidates_Request__ros_msg_type * ros_message = static_cast<const _GetGraspCandidates_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: num_candidates
  {
    cdr << ros_message->num_candidates;
  }

  // Field name: min_confidence
  {
    cdr << ros_message->min_confidence;
  }

  return true;
}

static bool _GetGraspCandidates_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetGraspCandidates_Request__ros_msg_type * ros_message = static_cast<_GetGraspCandidates_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: num_candidates
  {
    cdr >> ros_message->num_candidates;
  }

  // Field name: min_confidence
  {
    cdr >> ros_message->min_confidence;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t get_serialized_size_roarm_anygrasp_integration__srv__GetGraspCandidates_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetGraspCandidates_Request__ros_msg_type * ros_message = static_cast<const _GetGraspCandidates_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name num_candidates
  {
    size_t item_size = sizeof(ros_message->num_candidates);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name min_confidence
  {
    size_t item_size = sizeof(ros_message->min_confidence);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GetGraspCandidates_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_roarm_anygrasp_integration__srv__GetGraspCandidates_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t max_serialized_size_roarm_anygrasp_integration__srv__GetGraspCandidates_Request(
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

  // member: num_candidates
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: min_confidence
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
    using DataType = roarm_anygrasp_integration__srv__GetGraspCandidates_Request;
    is_plain =
      (
      offsetof(DataType, min_confidence) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetGraspCandidates_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_roarm_anygrasp_integration__srv__GetGraspCandidates_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetGraspCandidates_Request = {
  "roarm_anygrasp_integration::srv",
  "GetGraspCandidates_Request",
  _GetGraspCandidates_Request__cdr_serialize,
  _GetGraspCandidates_Request__cdr_deserialize,
  _GetGraspCandidates_Request__get_serialized_size,
  _GetGraspCandidates_Request__max_serialized_size
};

static rosidl_message_type_support_t _GetGraspCandidates_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetGraspCandidates_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, GetGraspCandidates_Request)() {
  return &_GetGraspCandidates_Request__type_support;
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
// #include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__struct.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__functions.h"
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

#include "builtin_interfaces/msg/detail/time__functions.h"  // detection_timestamp
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"  // grasp_poses
#include "rosidl_runtime_c/primitives_sequence.h"  // confidence_scores, grasp_widths, original_indices, quality_scores
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // confidence_scores, grasp_widths, original_indices, quality_scores
#include "rosidl_runtime_c/string.h"  // detection_status
#include "rosidl_runtime_c/string_functions.h"  // detection_status

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
size_t get_serialized_size_builtin_interfaces__msg__Time(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
size_t max_serialized_size_builtin_interfaces__msg__Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time)();
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


using _GetGraspCandidates_Response__ros_msg_type = roarm_anygrasp_integration__srv__GetGraspCandidates_Response;

static bool _GetGraspCandidates_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetGraspCandidates_Response__ros_msg_type * ros_message = static_cast<const _GetGraspCandidates_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: grasp_poses
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    size_t size = ros_message->grasp_poses.size;
    auto array_ptr = ros_message->grasp_poses.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: confidence_scores
  {
    size_t size = ros_message->confidence_scores.size;
    auto array_ptr = ros_message->confidence_scores.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: grasp_widths
  {
    size_t size = ros_message->grasp_widths.size;
    auto array_ptr = ros_message->grasp_widths.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: quality_scores
  {
    size_t size = ros_message->quality_scores.size;
    auto array_ptr = ros_message->quality_scores.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: original_indices
  {
    size_t size = ros_message->original_indices.size;
    auto array_ptr = ros_message->original_indices.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: detection_status
  {
    const rosidl_runtime_c__String * str = &ros_message->detection_status;
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

  // Field name: total_grasps_detected
  {
    cdr << ros_message->total_grasps_detected;
  }

  // Field name: detection_timestamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->detection_timestamp, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _GetGraspCandidates_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetGraspCandidates_Response__ros_msg_type * ros_message = static_cast<_GetGraspCandidates_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: grasp_poses
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->grasp_poses.data) {
      geometry_msgs__msg__PoseStamped__Sequence__fini(&ros_message->grasp_poses);
    }
    if (!geometry_msgs__msg__PoseStamped__Sequence__init(&ros_message->grasp_poses, size)) {
      fprintf(stderr, "failed to create array for field 'grasp_poses'");
      return false;
    }
    auto array_ptr = ros_message->grasp_poses.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: confidence_scores
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->confidence_scores.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->confidence_scores);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->confidence_scores, size)) {
      fprintf(stderr, "failed to create array for field 'confidence_scores'");
      return false;
    }
    auto array_ptr = ros_message->confidence_scores.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: grasp_widths
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->grasp_widths.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->grasp_widths);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->grasp_widths, size)) {
      fprintf(stderr, "failed to create array for field 'grasp_widths'");
      return false;
    }
    auto array_ptr = ros_message->grasp_widths.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: quality_scores
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->quality_scores.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->quality_scores);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->quality_scores, size)) {
      fprintf(stderr, "failed to create array for field 'quality_scores'");
      return false;
    }
    auto array_ptr = ros_message->quality_scores.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: original_indices
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->original_indices.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->original_indices);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->original_indices, size)) {
      fprintf(stderr, "failed to create array for field 'original_indices'");
      return false;
    }
    auto array_ptr = ros_message->original_indices.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: detection_status
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->detection_status.data) {
      rosidl_runtime_c__String__init(&ros_message->detection_status);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->detection_status,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'detection_status'\n");
      return false;
    }
  }

  // Field name: total_grasps_detected
  {
    cdr >> ros_message->total_grasps_detected;
  }

  // Field name: detection_timestamp
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->detection_timestamp))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t get_serialized_size_roarm_anygrasp_integration__srv__GetGraspCandidates_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetGraspCandidates_Response__ros_msg_type * ros_message = static_cast<const _GetGraspCandidates_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name grasp_poses
  {
    size_t array_size = ros_message->grasp_poses.size;
    auto array_ptr = ros_message->grasp_poses.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_geometry_msgs__msg__PoseStamped(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name confidence_scores
  {
    size_t array_size = ros_message->confidence_scores.size;
    auto array_ptr = ros_message->confidence_scores.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name grasp_widths
  {
    size_t array_size = ros_message->grasp_widths.size;
    auto array_ptr = ros_message->grasp_widths.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name quality_scores
  {
    size_t array_size = ros_message->quality_scores.size;
    auto array_ptr = ros_message->quality_scores.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name original_indices
  {
    size_t array_size = ros_message->original_indices.size;
    auto array_ptr = ros_message->original_indices.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name detection_status
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->detection_status.size + 1);
  // field.name total_grasps_detected
  {
    size_t item_size = sizeof(ros_message->total_grasps_detected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name detection_timestamp

  current_alignment += get_serialized_size_builtin_interfaces__msg__Time(
    &(ros_message->detection_timestamp), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _GetGraspCandidates_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_roarm_anygrasp_integration__srv__GetGraspCandidates_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t max_serialized_size_roarm_anygrasp_integration__srv__GetGraspCandidates_Response(
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

  // member: grasp_poses
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


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
  // member: confidence_scores
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: grasp_widths
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: quality_scores
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: original_indices
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: detection_status
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
  // member: total_grasps_detected
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: detection_timestamp
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_builtin_interfaces__msg__Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = roarm_anygrasp_integration__srv__GetGraspCandidates_Response;
    is_plain =
      (
      offsetof(DataType, detection_timestamp) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetGraspCandidates_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_roarm_anygrasp_integration__srv__GetGraspCandidates_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetGraspCandidates_Response = {
  "roarm_anygrasp_integration::srv",
  "GetGraspCandidates_Response",
  _GetGraspCandidates_Response__cdr_serialize,
  _GetGraspCandidates_Response__cdr_deserialize,
  _GetGraspCandidates_Response__get_serialized_size,
  _GetGraspCandidates_Response__max_serialized_size
};

static rosidl_message_type_support_t _GetGraspCandidates_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetGraspCandidates_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, GetGraspCandidates_Response)() {
  return &_GetGraspCandidates_Response__type_support;
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
#include "roarm_anygrasp_integration/srv/get_grasp_candidates.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GetGraspCandidates__callbacks = {
  "roarm_anygrasp_integration::srv",
  "GetGraspCandidates",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, GetGraspCandidates_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, GetGraspCandidates_Response)(),
};

static rosidl_service_type_support_t GetGraspCandidates__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GetGraspCandidates__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, GetGraspCandidates)() {
  return &GetGraspCandidates__handle;
}

#if defined(__cplusplus)
}
#endif
