// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__TRAITS_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace roarm_anygrasp_integration
{

namespace srv
{

inline void to_flow_style_yaml(
  const ConfigureRoboflow_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: api_key
  {
    out << "api_key: ";
    rosidl_generator_traits::value_to_yaml(msg.api_key, out);
    out << ", ";
  }

  // member: workspace_name
  {
    out << "workspace_name: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_name, out);
    out << ", ";
  }

  // member: project_name
  {
    out << "project_name: ";
    rosidl_generator_traits::value_to_yaml(msg.project_name, out);
    out << ", ";
  }

  // member: dataset_version
  {
    out << "dataset_version: ";
    rosidl_generator_traits::value_to_yaml(msg.dataset_version, out);
    out << ", ";
  }

  // member: auto_upload
  {
    out << "auto_upload: ";
    rosidl_generator_traits::value_to_yaml(msg.auto_upload, out);
    out << ", ";
  }

  // member: include_failed_grasps
  {
    out << "include_failed_grasps: ";
    rosidl_generator_traits::value_to_yaml(msg.include_failed_grasps, out);
    out << ", ";
  }

  // member: annotation_format
  {
    out << "annotation_format: ";
    rosidl_generator_traits::value_to_yaml(msg.annotation_format, out);
    out << ", ";
  }

  // member: min_confidence_for_upload
  {
    out << "min_confidence_for_upload: ";
    rosidl_generator_traits::value_to_yaml(msg.min_confidence_for_upload, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConfigureRoboflow_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: api_key
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "api_key: ";
    rosidl_generator_traits::value_to_yaml(msg.api_key, out);
    out << "\n";
  }

  // member: workspace_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "workspace_name: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_name, out);
    out << "\n";
  }

  // member: project_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "project_name: ";
    rosidl_generator_traits::value_to_yaml(msg.project_name, out);
    out << "\n";
  }

  // member: dataset_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dataset_version: ";
    rosidl_generator_traits::value_to_yaml(msg.dataset_version, out);
    out << "\n";
  }

  // member: auto_upload
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "auto_upload: ";
    rosidl_generator_traits::value_to_yaml(msg.auto_upload, out);
    out << "\n";
  }

  // member: include_failed_grasps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "include_failed_grasps: ";
    rosidl_generator_traits::value_to_yaml(msg.include_failed_grasps, out);
    out << "\n";
  }

  // member: annotation_format
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "annotation_format: ";
    rosidl_generator_traits::value_to_yaml(msg.annotation_format, out);
    out << "\n";
  }

  // member: min_confidence_for_upload
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_confidence_for_upload: ";
    rosidl_generator_traits::value_to_yaml(msg.min_confidence_for_upload, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConfigureRoboflow_Request & msg, bool use_flow_style = false)
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
  const roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  roarm_anygrasp_integration::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roarm_anygrasp_integration::srv::to_yaml() instead")]]
inline std::string to_yaml(const roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & msg)
{
  return roarm_anygrasp_integration::srv::to_yaml(msg);
}

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request>()
{
  return "roarm_anygrasp_integration::srv::ConfigureRoboflow_Request";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request>()
{
  return "roarm_anygrasp_integration/srv/ConfigureRoboflow_Request";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace roarm_anygrasp_integration
{

namespace srv
{

inline void to_flow_style_yaml(
  const ConfigureRoboflow_Response & msg,
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

  // member: workspace_url
  {
    out << "workspace_url: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_url, out);
    out << ", ";
  }

  // member: project_url
  {
    out << "project_url: ";
    rosidl_generator_traits::value_to_yaml(msg.project_url, out);
    out << ", ";
  }

  // member: total_images_in_dataset
  {
    out << "total_images_in_dataset: ";
    rosidl_generator_traits::value_to_yaml(msg.total_images_in_dataset, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConfigureRoboflow_Response & msg,
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

  // member: workspace_url
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "workspace_url: ";
    rosidl_generator_traits::value_to_yaml(msg.workspace_url, out);
    out << "\n";
  }

  // member: project_url
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "project_url: ";
    rosidl_generator_traits::value_to_yaml(msg.project_url, out);
    out << "\n";
  }

  // member: total_images_in_dataset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_images_in_dataset: ";
    rosidl_generator_traits::value_to_yaml(msg.total_images_in_dataset, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConfigureRoboflow_Response & msg, bool use_flow_style = false)
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
  const roarm_anygrasp_integration::srv::ConfigureRoboflow_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  roarm_anygrasp_integration::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roarm_anygrasp_integration::srv::to_yaml() instead")]]
inline std::string to_yaml(const roarm_anygrasp_integration::srv::ConfigureRoboflow_Response & msg)
{
  return roarm_anygrasp_integration::srv::to_yaml(msg);
}

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response>()
{
  return "roarm_anygrasp_integration::srv::ConfigureRoboflow_Response";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response>()
{
  return "roarm_anygrasp_integration/srv/ConfigureRoboflow_Response";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::ConfigureRoboflow>()
{
  return "roarm_anygrasp_integration::srv::ConfigureRoboflow";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::ConfigureRoboflow>()
{
  return "roarm_anygrasp_integration/srv/ConfigureRoboflow";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::ConfigureRoboflow>
  : std::integral_constant<
    bool,
    has_fixed_size<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request>::value &&
    has_fixed_size<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response>::value
  >
{
};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::ConfigureRoboflow>
  : std::integral_constant<
    bool,
    has_bounded_size<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request>::value &&
    has_bounded_size<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response>::value
  >
{
};

template<>
struct is_service<roarm_anygrasp_integration::srv::ConfigureRoboflow>
  : std::true_type
{
};

template<>
struct is_service_request<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request>
  : std::true_type
{
};

template<>
struct is_service_response<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__TRAITS_HPP_
