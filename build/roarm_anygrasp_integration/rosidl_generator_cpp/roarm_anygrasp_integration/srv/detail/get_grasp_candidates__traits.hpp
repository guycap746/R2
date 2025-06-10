// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from roarm_anygrasp_integration:srv/GetGraspCandidates.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__TRAITS_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace roarm_anygrasp_integration
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetGraspCandidates_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: num_candidates
  {
    out << "num_candidates: ";
    rosidl_generator_traits::value_to_yaml(msg.num_candidates, out);
    out << ", ";
  }

  // member: min_confidence
  {
    out << "min_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.min_confidence, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetGraspCandidates_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: num_candidates
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_candidates: ";
    rosidl_generator_traits::value_to_yaml(msg.num_candidates, out);
    out << "\n";
  }

  // member: min_confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.min_confidence, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetGraspCandidates_Request & msg, bool use_flow_style = false)
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
  const roarm_anygrasp_integration::srv::GetGraspCandidates_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  roarm_anygrasp_integration::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roarm_anygrasp_integration::srv::to_yaml() instead")]]
inline std::string to_yaml(const roarm_anygrasp_integration::srv::GetGraspCandidates_Request & msg)
{
  return roarm_anygrasp_integration::srv::to_yaml(msg);
}

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::GetGraspCandidates_Request>()
{
  return "roarm_anygrasp_integration::srv::GetGraspCandidates_Request";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::GetGraspCandidates_Request>()
{
  return "roarm_anygrasp_integration/srv/GetGraspCandidates_Request";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::GetGraspCandidates_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::GetGraspCandidates_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<roarm_anygrasp_integration::srv::GetGraspCandidates_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'grasp_poses'
#include "geometry_msgs/msg/detail/pose_stamped__traits.hpp"
// Member 'detection_timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace roarm_anygrasp_integration
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetGraspCandidates_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: grasp_poses
  {
    if (msg.grasp_poses.size() == 0) {
      out << "grasp_poses: []";
    } else {
      out << "grasp_poses: [";
      size_t pending_items = msg.grasp_poses.size();
      for (auto item : msg.grasp_poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
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

  // member: grasp_widths
  {
    if (msg.grasp_widths.size() == 0) {
      out << "grasp_widths: []";
    } else {
      out << "grasp_widths: [";
      size_t pending_items = msg.grasp_widths.size();
      for (auto item : msg.grasp_widths) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: quality_scores
  {
    if (msg.quality_scores.size() == 0) {
      out << "quality_scores: []";
    } else {
      out << "quality_scores: [";
      size_t pending_items = msg.quality_scores.size();
      for (auto item : msg.quality_scores) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: original_indices
  {
    if (msg.original_indices.size() == 0) {
      out << "original_indices: []";
    } else {
      out << "original_indices: [";
      size_t pending_items = msg.original_indices.size();
      for (auto item : msg.original_indices) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: detection_status
  {
    out << "detection_status: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_status, out);
    out << ", ";
  }

  // member: total_grasps_detected
  {
    out << "total_grasps_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.total_grasps_detected, out);
    out << ", ";
  }

  // member: detection_timestamp
  {
    out << "detection_timestamp: ";
    to_flow_style_yaml(msg.detection_timestamp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetGraspCandidates_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: grasp_poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.grasp_poses.size() == 0) {
      out << "grasp_poses: []\n";
    } else {
      out << "grasp_poses:\n";
      for (auto item : msg.grasp_poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
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

  // member: grasp_widths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.grasp_widths.size() == 0) {
      out << "grasp_widths: []\n";
    } else {
      out << "grasp_widths:\n";
      for (auto item : msg.grasp_widths) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: quality_scores
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.quality_scores.size() == 0) {
      out << "quality_scores: []\n";
    } else {
      out << "quality_scores:\n";
      for (auto item : msg.quality_scores) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: original_indices
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.original_indices.size() == 0) {
      out << "original_indices: []\n";
    } else {
      out << "original_indices:\n";
      for (auto item : msg.original_indices) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: detection_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection_status: ";
    rosidl_generator_traits::value_to_yaml(msg.detection_status, out);
    out << "\n";
  }

  // member: total_grasps_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "total_grasps_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.total_grasps_detected, out);
    out << "\n";
  }

  // member: detection_timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "detection_timestamp:\n";
    to_block_style_yaml(msg.detection_timestamp, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetGraspCandidates_Response & msg, bool use_flow_style = false)
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
  const roarm_anygrasp_integration::srv::GetGraspCandidates_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  roarm_anygrasp_integration::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use roarm_anygrasp_integration::srv::to_yaml() instead")]]
inline std::string to_yaml(const roarm_anygrasp_integration::srv::GetGraspCandidates_Response & msg)
{
  return roarm_anygrasp_integration::srv::to_yaml(msg);
}

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::GetGraspCandidates_Response>()
{
  return "roarm_anygrasp_integration::srv::GetGraspCandidates_Response";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::GetGraspCandidates_Response>()
{
  return "roarm_anygrasp_integration/srv/GetGraspCandidates_Response";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::GetGraspCandidates_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::GetGraspCandidates_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<roarm_anygrasp_integration::srv::GetGraspCandidates_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<roarm_anygrasp_integration::srv::GetGraspCandidates>()
{
  return "roarm_anygrasp_integration::srv::GetGraspCandidates";
}

template<>
inline const char * name<roarm_anygrasp_integration::srv::GetGraspCandidates>()
{
  return "roarm_anygrasp_integration/srv/GetGraspCandidates";
}

template<>
struct has_fixed_size<roarm_anygrasp_integration::srv::GetGraspCandidates>
  : std::integral_constant<
    bool,
    has_fixed_size<roarm_anygrasp_integration::srv::GetGraspCandidates_Request>::value &&
    has_fixed_size<roarm_anygrasp_integration::srv::GetGraspCandidates_Response>::value
  >
{
};

template<>
struct has_bounded_size<roarm_anygrasp_integration::srv::GetGraspCandidates>
  : std::integral_constant<
    bool,
    has_bounded_size<roarm_anygrasp_integration::srv::GetGraspCandidates_Request>::value &&
    has_bounded_size<roarm_anygrasp_integration::srv::GetGraspCandidates_Response>::value
  >
{
};

template<>
struct is_service<roarm_anygrasp_integration::srv::GetGraspCandidates>
  : std::true_type
{
};

template<>
struct is_service_request<roarm_anygrasp_integration::srv::GetGraspCandidates_Request>
  : std::true_type
{
};

template<>
struct is_service_response<roarm_anygrasp_integration::srv::GetGraspCandidates_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__TRAITS_HPP_
