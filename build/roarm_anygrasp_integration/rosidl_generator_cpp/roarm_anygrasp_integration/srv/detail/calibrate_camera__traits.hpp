// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from roarm_anygrasp_integration:srv/CalibrateCamera.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__TRAITS_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "roarm_anygrasp_integration/srv/detail/calibrate_camera__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace roarm_anygrasp_integration
{

namespace srv
{

inline void to_flow_style_yaml(
  const CalibrateCamera_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrateCamera_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrateCamera_Request & msg, bool use_flow_style = false)
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
  const roarm_anygrasp_integration::srv::CalibrateCamera_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  roarm_anygrasp_integration::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roarm_anygrasp_integration::srv::to_yaml() instead")]]
inline std::string to_yaml(const roarm_anygrasp_integration::srv::CalibrateCamera_Request & msg)
{
  return roarm_anygrasp_integration::srv::to_yaml(msg);
}

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::CalibrateCamera_Request>()
{
  return "roarm_anygrasp_integration::srv::CalibrateCamera_Request";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::CalibrateCamera_Request>()
{
  return "roarm_anygrasp_integration/srv/CalibrateCamera_Request";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::CalibrateCamera_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::CalibrateCamera_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<roarm_anygrasp_integration::srv::CalibrateCamera_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace roarm_anygrasp_integration
{

namespace srv
{

inline void to_flow_style_yaml(
  const CalibrateCamera_Response & msg,
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

  // member: primary_reprojection_error
  {
    out << "primary_reprojection_error: ";
    rosidl_generator_traits::value_to_yaml(msg.primary_reprojection_error, out);
    out << ", ";
  }

  // member: side_reprojection_error
  {
    out << "side_reprojection_error: ";
    rosidl_generator_traits::value_to_yaml(msg.side_reprojection_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CalibrateCamera_Response & msg,
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

  // member: primary_reprojection_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "primary_reprojection_error: ";
    rosidl_generator_traits::value_to_yaml(msg.primary_reprojection_error, out);
    out << "\n";
  }

  // member: side_reprojection_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "side_reprojection_error: ";
    rosidl_generator_traits::value_to_yaml(msg.side_reprojection_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CalibrateCamera_Response & msg, bool use_flow_style = false)
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
  const roarm_anygrasp_integration::srv::CalibrateCamera_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  roarm_anygrasp_integration::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roarm_anygrasp_integration::srv::to_yaml() instead")]]
inline std::string to_yaml(const roarm_anygrasp_integration::srv::CalibrateCamera_Response & msg)
{
  return roarm_anygrasp_integration::srv::to_yaml(msg);
}

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::CalibrateCamera_Response>()
{
  return "roarm_anygrasp_integration::srv::CalibrateCamera_Response";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::CalibrateCamera_Response>()
{
  return "roarm_anygrasp_integration/srv/CalibrateCamera_Response";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::CalibrateCamera_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::CalibrateCamera_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<roarm_anygrasp_integration::srv::CalibrateCamera_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::CalibrateCamera>()
{
  return "roarm_anygrasp_integration::srv::CalibrateCamera";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::CalibrateCamera>()
{
  return "roarm_anygrasp_integration/srv/CalibrateCamera";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::CalibrateCamera>
  : std::integral_constant<
    bool,
    has_fixed_size<roarm_anygrasp_integration::srv::CalibrateCamera_Request>::value &&
    has_fixed_size<roarm_anygrasp_integration::srv::CalibrateCamera_Response>::value
  >
{
};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::CalibrateCamera>
  : std::integral_constant<
    bool,
    has_bounded_size<roarm_anygrasp_integration::srv::CalibrateCamera_Request>::value &&
    has_bounded_size<roarm_anygrasp_integration::srv::CalibrateCamera_Response>::value
  >
{
};

template<>
struct is_service<roarm_anygrasp_integration::srv::CalibrateCamera>
  : std::true_type
{
};

template<>
struct is_service_request<roarm_anygrasp_integration::srv::CalibrateCamera_Request>
  : std::true_type
{
};

template<>
struct is_service_response<roarm_anygrasp_integration::srv::CalibrateCamera_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__TRAITS_HPP_
