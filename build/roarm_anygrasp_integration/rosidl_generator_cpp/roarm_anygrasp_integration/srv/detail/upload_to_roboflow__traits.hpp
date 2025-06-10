// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from roarm_anygrasp_integration:srv/UploadToRoboflow.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__TRAITS_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'rgb_image'
#include "sensor_msgs/msg/detail/image__traits.hpp"
// Member 'grasp_poses'
#include "geometry_msgs/msg/detail/pose_array__traits.hpp"

namespace roarm_anygrasp_integration
{

namespace srv
{

inline void to_flow_style_yaml(
  const UploadToRoboflow_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: rgb_image
  {
    out << "rgb_image: ";
    to_flow_style_yaml(msg.rgb_image, out);
    out << ", ";
  }

  // member: grasp_poses
  {
    out << "grasp_poses: ";
    to_flow_style_yaml(msg.grasp_poses, out);
    out << ", ";
  }

  // member: confidence_scores
  {
    if (msg.confidence_scores.size() == 0) {
      out << "confidence_scores: []";
    } else {
      out << "confidence_scores: [";
      size_t pending_items = msg.confidence_scores.size();
      for (auto item : msg.confidence_scores) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: class_labels
  {
    if (msg.class_labels.size() == 0) {
      out << "class_labels: []";
    } else {
      out << "class_labels: [";
      size_t pending_items = msg.class_labels.size();
      for (auto item : msg.class_labels) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: scene_description
  {
    out << "scene_description: ";
    rosidl_generator_traits::value_to_yaml(msg.scene_description, out);
    out << ", ";
  }

  // member: object_types
  {
    out << "object_types: ";
    rosidl_generator_traits::value_to_yaml(msg.object_types, out);
    out << ", ";
  }

  // member: include_annotations
  {
    out << "include_annotations: ";
    rosidl_generator_traits::value_to_yaml(msg.include_annotations, out);
    out << ", ";
  }

  // member: upload_immediately
  {
    out << "upload_immediately: ";
    rosidl_generator_traits::value_to_yaml(msg.upload_immediately, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UploadToRoboflow_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rgb_image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rgb_image:\n";
    to_block_style_yaml(msg.rgb_image, out, indentation + 2);
  }

  // member: grasp_poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grasp_poses:\n";
    to_block_style_yaml(msg.grasp_poses, out, indentation + 2);
  }

  // member: confidence_scores
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.confidence_scores.size() == 0) {
      out << "confidence_scores: []\n";
    } else {
      out << "confidence_scores:\n";
      for (auto item : msg.confidence_scores) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: class_labels
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.class_labels.size() == 0) {
      out << "class_labels: []\n";
    } else {
      out << "class_labels:\n";
      for (auto item : msg.class_labels) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: scene_description
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "scene_description: ";
    rosidl_generator_traits::value_to_yaml(msg.scene_description, out);
    out << "\n";
  }

  // member: object_types
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "object_types: ";
    rosidl_generator_traits::value_to_yaml(msg.object_types, out);
    out << "\n";
  }

  // member: include_annotations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "include_annotations: ";
    rosidl_generator_traits::value_to_yaml(msg.include_annotations, out);
    out << "\n";
  }

  // member: upload_immediately
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "upload_immediately: ";
    rosidl_generator_traits::value_to_yaml(msg.upload_immediately, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UploadToRoboflow_Request & msg, bool use_flow_style = false)
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
  const roarm_anygrasp_integration::srv::UploadToRoboflow_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  roarm_anygrasp_integration::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roarm_anygrasp_integration::srv::to_yaml() instead")]]
inline std::string to_yaml(const roarm_anygrasp_integration::srv::UploadToRoboflow_Request & msg)
{
  return roarm_anygrasp_integration::srv::to_yaml(msg);
}

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::UploadToRoboflow_Request>()
{
  return "roarm_anygrasp_integration::srv::UploadToRoboflow_Request";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::UploadToRoboflow_Request>()
{
  return "roarm_anygrasp_integration/srv/UploadToRoboflow_Request";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::UploadToRoboflow_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::UploadToRoboflow_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<roarm_anygrasp_integration::srv::UploadToRoboflow_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace roarm_anygrasp_integration
{

namespace srv
{

inline void to_flow_style_yaml(
  const UploadToRoboflow_Response & msg,
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

  // member: roboflow_image_id
  {
    out << "roboflow_image_id: ";
    rosidl_generator_traits::value_to_yaml(msg.roboflow_image_id, out);
    out << ", ";
  }

  // member: upload_url
  {
    out << "upload_url: ";
    rosidl_generator_traits::value_to_yaml(msg.upload_url, out);
    out << ", ";
  }

  // member: annotation_count
  {
    out << "annotation_count: ";
    rosidl_generator_traits::value_to_yaml(msg.annotation_count, out);
    out << ", ";
  }

  // member: dataset_version
  {
    out << "dataset_version: ";
    rosidl_generator_traits::value_to_yaml(msg.dataset_version, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UploadToRoboflow_Response & msg,
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

  // member: roboflow_image_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roboflow_image_id: ";
    rosidl_generator_traits::value_to_yaml(msg.roboflow_image_id, out);
    out << "\n";
  }

  // member: upload_url
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "upload_url: ";
    rosidl_generator_traits::value_to_yaml(msg.upload_url, out);
    out << "\n";
  }

  // member: annotation_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "annotation_count: ";
    rosidl_generator_traits::value_to_yaml(msg.annotation_count, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UploadToRoboflow_Response & msg, bool use_flow_style = false)
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
  const roarm_anygrasp_integration::srv::UploadToRoboflow_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  roarm_anygrasp_integration::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roarm_anygrasp_integration::srv::to_yaml() instead")]]
inline std::string to_yaml(const roarm_anygrasp_integration::srv::UploadToRoboflow_Response & msg)
{
  return roarm_anygrasp_integration::srv::to_yaml(msg);
}

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::UploadToRoboflow_Response>()
{
  return "roarm_anygrasp_integration::srv::UploadToRoboflow_Response";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::UploadToRoboflow_Response>()
{
  return "roarm_anygrasp_integration/srv/UploadToRoboflow_Response";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::UploadToRoboflow_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::UploadToRoboflow_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<roarm_anygrasp_integration::srv::UploadToRoboflow_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::UploadToRoboflow>()
{
  return "roarm_anygrasp_integration::srv::UploadToRoboflow";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::UploadToRoboflow>()
{
  return "roarm_anygrasp_integration/srv/UploadToRoboflow";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::UploadToRoboflow>
  : std::integral_constant<
    bool,
    has_fixed_size<roarm_anygrasp_integration::srv::UploadToRoboflow_Request>::value &&
    has_fixed_size<roarm_anygrasp_integration::srv::UploadToRoboflow_Response>::value
  >
{
};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::UploadToRoboflow>
  : std::integral_constant<
    bool,
    has_bounded_size<roarm_anygrasp_integration::srv::UploadToRoboflow_Request>::value &&
    has_bounded_size<roarm_anygrasp_integration::srv::UploadToRoboflow_Response>::value
  >
{
};

template<>
struct is_service<roarm_anygrasp_integration::srv::UploadToRoboflow>
  : std::true_type
{
};

template<>
struct is_service_request<roarm_anygrasp_integration::srv::UploadToRoboflow_Request>
  : std::true_type
{
};

template<>
struct is_service_response<roarm_anygrasp_integration::srv::UploadToRoboflow_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__TRAITS_HPP_
