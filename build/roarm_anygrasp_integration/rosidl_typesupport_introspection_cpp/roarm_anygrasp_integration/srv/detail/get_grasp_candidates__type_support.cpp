// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from roarm_anygrasp_integration:srv/GetGraspCandidates.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace roarm_anygrasp_integration
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetGraspCandidates_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) roarm_anygrasp_integration::srv::GetGraspCandidates_Request(_init);
}

void GetGraspCandidates_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<roarm_anygrasp_integration::srv::GetGraspCandidates_Request *>(message_memory);
  typed_message->~GetGraspCandidates_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetGraspCandidates_Request_message_member_array[2] = {
  {
    "num_candidates",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration::srv::GetGraspCandidates_Request, num_candidates),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "min_confidence",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration::srv::GetGraspCandidates_Request, min_confidence),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetGraspCandidates_Request_message_members = {
  "roarm_anygrasp_integration::srv",  // message namespace
  "GetGraspCandidates_Request",  // message name
  2,  // number of fields
  sizeof(roarm_anygrasp_integration::srv::GetGraspCandidates_Request),
  GetGraspCandidates_Request_message_member_array,  // message members
  GetGraspCandidates_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetGraspCandidates_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetGraspCandidates_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetGraspCandidates_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace roarm_anygrasp_integration


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<roarm_anygrasp_integration::srv::GetGraspCandidates_Request>()
{
  return &::roarm_anygrasp_integration::srv::rosidl_typesupport_introspection_cpp::GetGraspCandidates_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, roarm_anygrasp_integration, srv, GetGraspCandidates_Request)() {
  return &::roarm_anygrasp_integration::srv::rosidl_typesupport_introspection_cpp::GetGraspCandidates_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace roarm_anygrasp_integration
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void GetGraspCandidates_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) roarm_anygrasp_integration::srv::GetGraspCandidates_Response(_init);
}

void GetGraspCandidates_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<roarm_anygrasp_integration::srv::GetGraspCandidates_Response *>(message_memory);
  typed_message->~GetGraspCandidates_Response();
}

size_t size_function__GetGraspCandidates_Response__grasp_poses(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetGraspCandidates_Response__grasp_poses(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return &member[index];
}

void * get_function__GetGraspCandidates_Response__grasp_poses(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetGraspCandidates_Response__grasp_poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(
    get_const_function__GetGraspCandidates_Response__grasp_poses(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(untyped_value);
  value = item;
}

void assign_function__GetGraspCandidates_Response__grasp_poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::PoseStamped *>(
    get_function__GetGraspCandidates_Response__grasp_poses(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::PoseStamped *>(untyped_value);
  item = value;
}

void resize_function__GetGraspCandidates_Response__grasp_poses(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::PoseStamped> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetGraspCandidates_Response__confidence_scores(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetGraspCandidates_Response__confidence_scores(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__GetGraspCandidates_Response__confidence_scores(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetGraspCandidates_Response__confidence_scores(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__GetGraspCandidates_Response__confidence_scores(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__GetGraspCandidates_Response__confidence_scores(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__GetGraspCandidates_Response__confidence_scores(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__GetGraspCandidates_Response__confidence_scores(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetGraspCandidates_Response__grasp_widths(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetGraspCandidates_Response__grasp_widths(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__GetGraspCandidates_Response__grasp_widths(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetGraspCandidates_Response__grasp_widths(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__GetGraspCandidates_Response__grasp_widths(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__GetGraspCandidates_Response__grasp_widths(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__GetGraspCandidates_Response__grasp_widths(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__GetGraspCandidates_Response__grasp_widths(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetGraspCandidates_Response__quality_scores(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetGraspCandidates_Response__quality_scores(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__GetGraspCandidates_Response__quality_scores(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetGraspCandidates_Response__quality_scores(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__GetGraspCandidates_Response__quality_scores(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__GetGraspCandidates_Response__quality_scores(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__GetGraspCandidates_Response__quality_scores(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__GetGraspCandidates_Response__quality_scores(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__GetGraspCandidates_Response__original_indices(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__GetGraspCandidates_Response__original_indices(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__GetGraspCandidates_Response__original_indices(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__GetGraspCandidates_Response__original_indices(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__GetGraspCandidates_Response__original_indices(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__GetGraspCandidates_Response__original_indices(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__GetGraspCandidates_Response__original_indices(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__GetGraspCandidates_Response__original_indices(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GetGraspCandidates_Response_message_member_array[8] = {
  {
    "grasp_poses",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration::srv::GetGraspCandidates_Response, grasp_poses),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetGraspCandidates_Response__grasp_poses,  // size() function pointer
    get_const_function__GetGraspCandidates_Response__grasp_poses,  // get_const(index) function pointer
    get_function__GetGraspCandidates_Response__grasp_poses,  // get(index) function pointer
    fetch_function__GetGraspCandidates_Response__grasp_poses,  // fetch(index, &value) function pointer
    assign_function__GetGraspCandidates_Response__grasp_poses,  // assign(index, value) function pointer
    resize_function__GetGraspCandidates_Response__grasp_poses  // resize(index) function pointer
  },
  {
    "confidence_scores",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration::srv::GetGraspCandidates_Response, confidence_scores),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetGraspCandidates_Response__confidence_scores,  // size() function pointer
    get_const_function__GetGraspCandidates_Response__confidence_scores,  // get_const(index) function pointer
    get_function__GetGraspCandidates_Response__confidence_scores,  // get(index) function pointer
    fetch_function__GetGraspCandidates_Response__confidence_scores,  // fetch(index, &value) function pointer
    assign_function__GetGraspCandidates_Response__confidence_scores,  // assign(index, value) function pointer
    resize_function__GetGraspCandidates_Response__confidence_scores  // resize(index) function pointer
  },
  {
    "grasp_widths",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration::srv::GetGraspCandidates_Response, grasp_widths),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetGraspCandidates_Response__grasp_widths,  // size() function pointer
    get_const_function__GetGraspCandidates_Response__grasp_widths,  // get_const(index) function pointer
    get_function__GetGraspCandidates_Response__grasp_widths,  // get(index) function pointer
    fetch_function__GetGraspCandidates_Response__grasp_widths,  // fetch(index, &value) function pointer
    assign_function__GetGraspCandidates_Response__grasp_widths,  // assign(index, value) function pointer
    resize_function__GetGraspCandidates_Response__grasp_widths  // resize(index) function pointer
  },
  {
    "quality_scores",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration::srv::GetGraspCandidates_Response, quality_scores),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetGraspCandidates_Response__quality_scores,  // size() function pointer
    get_const_function__GetGraspCandidates_Response__quality_scores,  // get_const(index) function pointer
    get_function__GetGraspCandidates_Response__quality_scores,  // get(index) function pointer
    fetch_function__GetGraspCandidates_Response__quality_scores,  // fetch(index, &value) function pointer
    assign_function__GetGraspCandidates_Response__quality_scores,  // assign(index, value) function pointer
    resize_function__GetGraspCandidates_Response__quality_scores  // resize(index) function pointer
  },
  {
    "original_indices",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration::srv::GetGraspCandidates_Response, original_indices),  // bytes offset in struct
    nullptr,  // default value
    size_function__GetGraspCandidates_Response__original_indices,  // size() function pointer
    get_const_function__GetGraspCandidates_Response__original_indices,  // get_const(index) function pointer
    get_function__GetGraspCandidates_Response__original_indices,  // get(index) function pointer
    fetch_function__GetGraspCandidates_Response__original_indices,  // fetch(index, &value) function pointer
    assign_function__GetGraspCandidates_Response__original_indices,  // assign(index, value) function pointer
    resize_function__GetGraspCandidates_Response__original_indices  // resize(index) function pointer
  },
  {
    "detection_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration::srv::GetGraspCandidates_Response, detection_status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "total_grasps_detected",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration::srv::GetGraspCandidates_Response, total_grasps_detected),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "detection_timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(roarm_anygrasp_integration::srv::GetGraspCandidates_Response, detection_timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GetGraspCandidates_Response_message_members = {
  "roarm_anygrasp_integration::srv",  // message namespace
  "GetGraspCandidates_Response",  // message name
  8,  // number of fields
  sizeof(roarm_anygrasp_integration::srv::GetGraspCandidates_Response),
  GetGraspCandidates_Response_message_member_array,  // message members
  GetGraspCandidates_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetGraspCandidates_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GetGraspCandidates_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetGraspCandidates_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace roarm_anygrasp_integration


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<roarm_anygrasp_integration::srv::GetGraspCandidates_Response>()
{
  return &::roarm_anygrasp_integration::srv::rosidl_typesupport_introspection_cpp::GetGraspCandidates_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, roarm_anygrasp_integration, srv, GetGraspCandidates_Response)() {
  return &::roarm_anygrasp_integration::srv::rosidl_typesupport_introspection_cpp::GetGraspCandidates_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace roarm_anygrasp_integration
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers GetGraspCandidates_service_members = {
  "roarm_anygrasp_integration::srv",  // service namespace
  "GetGraspCandidates",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<roarm_anygrasp_integration::srv::GetGraspCandidates>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t GetGraspCandidates_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GetGraspCandidates_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace roarm_anygrasp_integration


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<roarm_anygrasp_integration::srv::GetGraspCandidates>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::roarm_anygrasp_integration::srv::rosidl_typesupport_introspection_cpp::GetGraspCandidates_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::roarm_anygrasp_integration::srv::GetGraspCandidates_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::roarm_anygrasp_integration::srv::GetGraspCandidates_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, roarm_anygrasp_integration, srv, GetGraspCandidates)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<roarm_anygrasp_integration::srv::GetGraspCandidates>();
}

#ifdef __cplusplus
}
#endif
