// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from roarm_anygrasp_integration:srv/GetGraspCandidates.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__BUILDER_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace roarm_anygrasp_integration
{

namespace srv
{

namespace builder
{

class Init_GetGraspCandidates_Request_min_confidence
{
public:
  explicit Init_GetGraspCandidates_Request_min_confidence(::roarm_anygrasp_integration::srv::GetGraspCandidates_Request & msg)
  : msg_(msg)
  {}
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Request min_confidence(::roarm_anygrasp_integration::srv::GetGraspCandidates_Request::_min_confidence_type arg)
  {
    msg_.min_confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Request msg_;
};

class Init_GetGraspCandidates_Request_num_candidates
{
public:
  Init_GetGraspCandidates_Request_num_candidates()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetGraspCandidates_Request_min_confidence num_candidates(::roarm_anygrasp_integration::srv::GetGraspCandidates_Request::_num_candidates_type arg)
  {
    msg_.num_candidates = std::move(arg);
    return Init_GetGraspCandidates_Request_min_confidence(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_anygrasp_integration::srv::GetGraspCandidates_Request>()
{
  return roarm_anygrasp_integration::srv::builder::Init_GetGraspCandidates_Request_num_candidates();
}

}  // namespace roarm_anygrasp_integration


namespace roarm_anygrasp_integration
{

namespace srv
{

namespace builder
{

class Init_GetGraspCandidates_Response_detection_timestamp
{
public:
  explicit Init_GetGraspCandidates_Response_detection_timestamp(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response & msg)
  : msg_(msg)
  {}
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Response detection_timestamp(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response::_detection_timestamp_type arg)
  {
    msg_.detection_timestamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Response msg_;
};

class Init_GetGraspCandidates_Response_total_grasps_detected
{
public:
  explicit Init_GetGraspCandidates_Response_total_grasps_detected(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response & msg)
  : msg_(msg)
  {}
  Init_GetGraspCandidates_Response_detection_timestamp total_grasps_detected(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response::_total_grasps_detected_type arg)
  {
    msg_.total_grasps_detected = std::move(arg);
    return Init_GetGraspCandidates_Response_detection_timestamp(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Response msg_;
};

class Init_GetGraspCandidates_Response_detection_status
{
public:
  explicit Init_GetGraspCandidates_Response_detection_status(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response & msg)
  : msg_(msg)
  {}
  Init_GetGraspCandidates_Response_total_grasps_detected detection_status(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response::_detection_status_type arg)
  {
    msg_.detection_status = std::move(arg);
    return Init_GetGraspCandidates_Response_total_grasps_detected(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Response msg_;
};

class Init_GetGraspCandidates_Response_original_indices
{
public:
  explicit Init_GetGraspCandidates_Response_original_indices(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response & msg)
  : msg_(msg)
  {}
  Init_GetGraspCandidates_Response_detection_status original_indices(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response::_original_indices_type arg)
  {
    msg_.original_indices = std::move(arg);
    return Init_GetGraspCandidates_Response_detection_status(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Response msg_;
};

class Init_GetGraspCandidates_Response_quality_scores
{
public:
  explicit Init_GetGraspCandidates_Response_quality_scores(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response & msg)
  : msg_(msg)
  {}
  Init_GetGraspCandidates_Response_original_indices quality_scores(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response::_quality_scores_type arg)
  {
    msg_.quality_scores = std::move(arg);
    return Init_GetGraspCandidates_Response_original_indices(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Response msg_;
};

class Init_GetGraspCandidates_Response_grasp_widths
{
public:
  explicit Init_GetGraspCandidates_Response_grasp_widths(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response & msg)
  : msg_(msg)
  {}
  Init_GetGraspCandidates_Response_quality_scores grasp_widths(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response::_grasp_widths_type arg)
  {
    msg_.grasp_widths = std::move(arg);
    return Init_GetGraspCandidates_Response_quality_scores(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Response msg_;
};

class Init_GetGraspCandidates_Response_confidence_scores
{
public:
  explicit Init_GetGraspCandidates_Response_confidence_scores(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response & msg)
  : msg_(msg)
  {}
  Init_GetGraspCandidates_Response_grasp_widths confidence_scores(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response::_confidence_scores_type arg)
  {
    msg_.confidence_scores = std::move(arg);
    return Init_GetGraspCandidates_Response_grasp_widths(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Response msg_;
};

class Init_GetGraspCandidates_Response_grasp_poses
{
public:
  Init_GetGraspCandidates_Response_grasp_poses()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetGraspCandidates_Response_confidence_scores grasp_poses(::roarm_anygrasp_integration::srv::GetGraspCandidates_Response::_grasp_poses_type arg)
  {
    msg_.grasp_poses = std::move(arg);
    return Init_GetGraspCandidates_Response_confidence_scores(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::GetGraspCandidates_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_anygrasp_integration::srv::GetGraspCandidates_Response>()
{
  return roarm_anygrasp_integration::srv::builder::Init_GetGraspCandidates_Response_grasp_poses();
}

}  // namespace roarm_anygrasp_integration

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__BUILDER_HPP_
