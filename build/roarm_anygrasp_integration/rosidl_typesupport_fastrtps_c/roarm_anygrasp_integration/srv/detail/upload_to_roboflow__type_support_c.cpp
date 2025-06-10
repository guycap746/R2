// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from roarm_anygrasp_integration:srv/UploadToRoboflow.idl
// generated code does not contain a copyright notice
#include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "roarm_anygrasp_integration/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__struct.h"
#include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__functions.h"
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

#include "geometry_msgs/msg/detail/pose_array__functions.h"  // grasp_poses
#include "rosidl_runtime_c/primitives_sequence.h"  // confidence_scores
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // confidence_scores
#include "rosidl_runtime_c/string.h"  // class_labels, object_types, scene_description
#include "rosidl_runtime_c/string_functions.h"  // class_labels, object_types, scene_description
#include "sensor_msgs/msg/detail/image__functions.h"  // rgb_image

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
size_t get_serialized_size_geometry_msgs__msg__PoseArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
size_t max_serialized_size_geometry_msgs__msg__PoseArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseArray)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
size_t get_serialized_size_sensor_msgs__msg__Image(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
size_t max_serialized_size_sensor_msgs__msg__Image(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_roarm_anygrasp_integration
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image)();


using _UploadToRoboflow_Request__ros_msg_type = roarm_anygrasp_integration__srv__UploadToRoboflow_Request;

static bool _UploadToRoboflow_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UploadToRoboflow_Request__ros_msg_type * ros_message = static_cast<const _UploadToRoboflow_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: rgb_image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->rgb_image, cdr))
    {
      return false;
    }
  }

  // Field name: grasp_poses
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseArray
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->grasp_poses, cdr))
    {
      return false;
    }
  }

  // Field name: confidence_scores
  {
    size_t size = ros_message->confidence_scores.size;
    auto array_ptr = ros_message->confidence_scores.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: class_labels
  {
    size_t size = ros_message->class_labels.size;
    auto array_ptr = ros_message->class_labels.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
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
  }

  // Field name: scene_description
  {
    const rosidl_runtime_c__String * str = &ros_message->scene_description;
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

  // Field name: object_types
  {
    const rosidl_runtime_c__String * str = &ros_message->object_types;
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

  // Field name: include_annotations
  {
    cdr << (ros_message->include_annotations ? true : false);
  }

  // Field name: upload_immediately
  {
    cdr << (ros_message->upload_immediately ? true : false);
  }

  return true;
}

static bool _UploadToRoboflow_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UploadToRoboflow_Request__ros_msg_type * ros_message = static_cast<_UploadToRoboflow_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: rgb_image
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, sensor_msgs, msg, Image
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->rgb_image))
    {
      return false;
    }
  }

  // Field name: grasp_poses
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseArray
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->grasp_poses))
    {
      return false;
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

  // Field name: class_labels
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->class_labels.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->class_labels);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->class_labels, size)) {
      fprintf(stderr, "failed to create array for field 'class_labels'");
      return false;
    }
    auto array_ptr = ros_message->class_labels.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'class_labels'\n");
        return false;
      }
    }
  }

  // Field name: scene_description
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->scene_description.data) {
      rosidl_runtime_c__String__init(&ros_message->scene_description);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->scene_description,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'scene_description'\n");
      return false;
    }
  }

  // Field name: object_types
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->object_types.data) {
      rosidl_runtime_c__String__init(&ros_message->object_types);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->object_types,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'object_types'\n");
      return false;
    }
  }

  // Field name: include_annotations
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->include_annotations = tmp ? true : false;
  }

  // Field name: upload_immediately
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->upload_immediately = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t get_serialized_size_roarm_anygrasp_integration__srv__UploadToRoboflow_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UploadToRoboflow_Request__ros_msg_type * ros_message = static_cast<const _UploadToRoboflow_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name rgb_image

  current_alignment += get_serialized_size_sensor_msgs__msg__Image(
    &(ros_message->rgb_image), current_alignment);
  // field.name grasp_poses

  current_alignment += get_serialized_size_geometry_msgs__msg__PoseArray(
    &(ros_message->grasp_poses), current_alignment);
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
  // field.name class_labels
  {
    size_t array_size = ros_message->class_labels.size;
    auto array_ptr = ros_message->class_labels.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name scene_description
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->scene_description.size + 1);
  // field.name object_types
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->object_types.size + 1);
  // field.name include_annotations
  {
    size_t item_size = sizeof(ros_message->include_annotations);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name upload_immediately
  {
    size_t item_size = sizeof(ros_message->upload_immediately);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UploadToRoboflow_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_roarm_anygrasp_integration__srv__UploadToRoboflow_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t max_serialized_size_roarm_anygrasp_integration__srv__UploadToRoboflow_Request(
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

  // member: rgb_image
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__Image(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: grasp_poses
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseArray(
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
  // member: class_labels
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: scene_description
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
  // member: object_types
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
  // member: include_annotations
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: upload_immediately
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
    using DataType = roarm_anygrasp_integration__srv__UploadToRoboflow_Request;
    is_plain =
      (
      offsetof(DataType, upload_immediately) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _UploadToRoboflow_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_roarm_anygrasp_integration__srv__UploadToRoboflow_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UploadToRoboflow_Request = {
  "roarm_anygrasp_integration::srv",
  "UploadToRoboflow_Request",
  _UploadToRoboflow_Request__cdr_serialize,
  _UploadToRoboflow_Request__cdr_deserialize,
  _UploadToRoboflow_Request__get_serialized_size,
  _UploadToRoboflow_Request__max_serialized_size
};

static rosidl_message_type_support_t _UploadToRoboflow_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UploadToRoboflow_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, UploadToRoboflow_Request)() {
  return &_UploadToRoboflow_Request__type_support;
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
// #include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__struct.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__functions.h"
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
// #include "rosidl_runtime_c/string.h"  // dataset_version, message, roboflow_image_id, upload_url
// already included above
// #include "rosidl_runtime_c/string_functions.h"  // dataset_version, message, roboflow_image_id, upload_url

// forward declare type support functions


using _UploadToRoboflow_Response__ros_msg_type = roarm_anygrasp_integration__srv__UploadToRoboflow_Response;

static bool _UploadToRoboflow_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UploadToRoboflow_Response__ros_msg_type * ros_message = static_cast<const _UploadToRoboflow_Response__ros_msg_type *>(untyped_ros_message);
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

  // Field name: roboflow_image_id
  {
    const rosidl_runtime_c__String * str = &ros_message->roboflow_image_id;
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

  // Field name: upload_url
  {
    const rosidl_runtime_c__String * str = &ros_message->upload_url;
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

  // Field name: annotation_count
  {
    cdr << ros_message->annotation_count;
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

  return true;
}

static bool _UploadToRoboflow_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UploadToRoboflow_Response__ros_msg_type * ros_message = static_cast<_UploadToRoboflow_Response__ros_msg_type *>(untyped_ros_message);
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

  // Field name: roboflow_image_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->roboflow_image_id.data) {
      rosidl_runtime_c__String__init(&ros_message->roboflow_image_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->roboflow_image_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'roboflow_image_id'\n");
      return false;
    }
  }

  // Field name: upload_url
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->upload_url.data) {
      rosidl_runtime_c__String__init(&ros_message->upload_url);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->upload_url,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'upload_url'\n");
      return false;
    }
  }

  // Field name: annotation_count
  {
    cdr >> ros_message->annotation_count;
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

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t get_serialized_size_roarm_anygrasp_integration__srv__UploadToRoboflow_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UploadToRoboflow_Response__ros_msg_type * ros_message = static_cast<const _UploadToRoboflow_Response__ros_msg_type *>(untyped_ros_message);
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
  // field.name roboflow_image_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->roboflow_image_id.size + 1);
  // field.name upload_url
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->upload_url.size + 1);
  // field.name annotation_count
  {
    size_t item_size = sizeof(ros_message->annotation_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dataset_version
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->dataset_version.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _UploadToRoboflow_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_roarm_anygrasp_integration__srv__UploadToRoboflow_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_roarm_anygrasp_integration
size_t max_serialized_size_roarm_anygrasp_integration__srv__UploadToRoboflow_Response(
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
  // member: roboflow_image_id
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
  // member: upload_url
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
  // member: annotation_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
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

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = roarm_anygrasp_integration__srv__UploadToRoboflow_Response;
    is_plain =
      (
      offsetof(DataType, dataset_version) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _UploadToRoboflow_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_roarm_anygrasp_integration__srv__UploadToRoboflow_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UploadToRoboflow_Response = {
  "roarm_anygrasp_integration::srv",
  "UploadToRoboflow_Response",
  _UploadToRoboflow_Response__cdr_serialize,
  _UploadToRoboflow_Response__cdr_deserialize,
  _UploadToRoboflow_Response__get_serialized_size,
  _UploadToRoboflow_Response__max_serialized_size
};

static rosidl_message_type_support_t _UploadToRoboflow_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UploadToRoboflow_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, UploadToRoboflow_Response)() {
  return &_UploadToRoboflow_Response__type_support;
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
#include "roarm_anygrasp_integration/srv/upload_to_roboflow.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t UploadToRoboflow__callbacks = {
  "roarm_anygrasp_integration::srv",
  "UploadToRoboflow",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, UploadToRoboflow_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, UploadToRoboflow_Response)(),
};

static rosidl_service_type_support_t UploadToRoboflow__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &UploadToRoboflow__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, roarm_anygrasp_integration, srv, UploadToRoboflow)() {
  return &UploadToRoboflow__handle;
}

#if defined(__cplusplus)
}
#endif
