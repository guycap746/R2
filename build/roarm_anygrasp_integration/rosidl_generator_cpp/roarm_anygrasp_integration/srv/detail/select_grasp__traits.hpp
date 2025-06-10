// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from roarm_anygrasp_integration:srv/SelectGrasp.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__TRAITS_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "roarm_anygrasp_integration/srv/detail/select_grasp__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace roarm_anygrasp_integration
{

namespace srv
{

inline void to_flow_style_yaml(
  const SelectGrasp_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: selected_grasp_index
  {
    out << "selected_grasp_index: ";
    rosidl_generator_traits::value_to_yaml(msg.selected_grasp_index, out);
    out << ", ";
  }

  // member: show_more_candidates
  {
    out << "show_more_candidates: ";
    rosidl_generator_traits::value_to_yaml(msg.show_more_candidates, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SelectGrasp_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: selected_grasp_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selected_grasp_index: ";
    rosidl_generator_traits::value_to_yaml(msg.selected_grasp_index, out);
    out << "\n";
  }

  // member: show_more_candidates
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "show_more_candidates: ";
    rosidl_generator_traits::value_to_yaml(msg.show_more_candidates, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SelectGrasp_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace roarm_anygrasp_integration

namespace rosidl_generator_traits
{

[[deprecated("use roarm_anygrasp_integration::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const roarm_anygrasp_integration::srv::SelectGrasp_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  roarm_anygrasp_integration::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roarm_anygrasp_integration::srv::to_yaml() instead")]]
inline std::string to_yaml(const roarm_anygrasp_integration::srv::SelectGrasp_Request & msg)
{
  return roarm_anygrasp_integration::srv::to_yaml(msg);
}

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::SelectGrasp_Request>()
{
  return "roarm_anygrasp_integration::srv::SelectGrasp_Request";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::SelectGrasp_Request>()
{
  return "roarm_anygrasp_integration/srv/SelectGrasp_Request";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::SelectGrasp_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::SelectGrasp_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<roarm_anygrasp_integration::srv::SelectGrasp_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'selected_pose'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"

namespace roarm_anygrasp_integration
{

namespace srv
{

inline void to_flow_style_yaml(
  const SelectGrasp_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << ", ";
  }

  // member: selected_pose
  {
    out << "selected_pose: ";
    to_flow_style_yaml(msg.selected_pose, out);
    out << ", ";
  }

  // member: confidence_score
  {
    out << "confidence_score: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence_score, out);
    out << ", ";
  }

  // member: grasp_width
  {
    out << "grasp_width: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_width, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SelectGrasp_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }

  // member: selected_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "selected_pose:\n";
    to_block_style_yaml(msg.selected_pose, out, indentation + 2);
  }

  // member: confidence_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence_score: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence_score, out);
    out << "\n";
  }

  // member: grasp_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grasp_width: ";
    rosidl_generator_traits::value_to_yaml(msg.grasp_width, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SelectGrasp_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace roarm_anygrasp_integration

namespace rosidl_generator_traits
{

[[deprecated("use roarm_anygrasp_integration::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const roarm_anygrasp_integration::srv::SelectGrasp_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  roarm_anygrasp_integration::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roarm_anygrasp_integration::srv::to_yaml() instead")]]
inline std::string to_yaml(const roarm_anygrasp_integration::srv::SelectGrasp_Response & msg)
{
  return roarm_anygrasp_integration::srv::to_yaml(msg);
}

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::SelectGrasp_Response>()
{
  return "roarm_anygrasp_integration::srv::SelectGrasp_Response";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::SelectGrasp_Response>()
{
  return "roarm_anygrasp_integration/srv/SelectGrasp_Response";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::SelectGrasp_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::SelectGrasp_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<roarm_anygrasp_integration::srv::SelectGrasp_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::SelectGrasp>()
{
  return "roarm_anygrasp_integration::srv::SelectGrasp";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::SelectGrasp>()
{
  return "roarm_anygrasp_integration/srv/SelectGrasp";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::SelectGrasp>
  : std::integral_constant<
    bool,
    has_fixed_size<roarm_anygrasp_integration::srv::SelectGrasp_Request>::value &&
    has_fixed_size<roarm_anygrasp_integration::srv::SelectGrasp_Response>::value
  >
{
};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::SelectGrasp>
  : std::integral_constant<
    bool,
    has_bounded_size<roarm_anygrasp_integration::srv::SelectGrasp_Request>::value &&
    has_bounded_size<roarm_anygrasp_integration::srv::SelectGrasp_Response>::value
  >
{
};

template<>
struct is_service<roarm_anygrasp_integration::srv::SelectGrasp>
  : std::true_type
{
};

template<>
struct is_service_request<roarm_anygrasp_integration::srv::SelectGrasp_Request>
  : std::true_type
{
};

template<>
struct is_service_response<roarm_anygrasp_integration::srv::SelectGrasp_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__TRAITS_HPP_
