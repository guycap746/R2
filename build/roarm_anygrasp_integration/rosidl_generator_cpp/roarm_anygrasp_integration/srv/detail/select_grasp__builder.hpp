// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from roarm_anygrasp_integration:srv/SelectGrasp.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__BUILDER_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "roarm_anygrasp_integration/srv/detail/select_grasp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace roarm_anygrasp_integration
{

namespace srv
{

namespace builder
{

class Init_SelectGrasp_Request_show_more_candidates
{
public:
  explicit Init_SelectGrasp_Request_show_more_candidates(::roarm_anygrasp_integration::srv::SelectGrasp_Request & msg)
  : msg_(msg)
  {}
  ::roarm_anygrasp_integration::srv::SelectGrasp_Request show_more_candidates(::roarm_anygrasp_integration::srv::SelectGrasp_Request::_show_more_candidates_type arg)
  {
    msg_.show_more_candidates = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::SelectGrasp_Request msg_;
};

class Init_SelectGrasp_Request_selected_grasp_index
{
public:
  Init_SelectGrasp_Request_selected_grasp_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SelectGrasp_Request_show_more_candidates selected_grasp_index(::roarm_anygrasp_integration::srv::SelectGrasp_Request::_selected_grasp_index_type arg)
  {
    msg_.selected_grasp_index = std::move(arg);
    return Init_SelectGrasp_Request_show_more_candidates(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::SelectGrasp_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_anygrasp_integration::srv::SelectGrasp_Request>()
{
  return roarm_anygrasp_integration::srv::builder::Init_SelectGrasp_Request_selected_grasp_index();
}

}  // namespace roarm_anygrasp_integration


namespace roarm_anygrasp_integration
{

namespace srv
{

namespace builder
{

class Init_SelectGrasp_Response_grasp_width
{
public:
  explicit Init_SelectGrasp_Response_grasp_width(::roarm_anygrasp_integration::srv::SelectGrasp_Response & msg)
  : msg_(msg)
  {}
  ::roarm_anygrasp_integration::srv::SelectGrasp_Response grasp_width(::roarm_anygrasp_integration::srv::SelectGrasp_Response::_grasp_width_type arg)
  {
    msg_.grasp_width = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::SelectGrasp_Response msg_;
};

class Init_SelectGrasp_Response_confidence_score
{
public:
  explicit Init_SelectGrasp_Response_confidence_score(::roarm_anygrasp_integration::srv::SelectGrasp_Response & msg)
  : msg_(msg)
  {}
  Init_SelectGrasp_Response_grasp_width confidence_score(::roarm_anygrasp_integration::srv::SelectGrasp_Response::_confidence_score_type arg)
  {
    msg_.confidence_score = std::move(arg);
    return Init_SelectGrasp_Response_grasp_width(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::SelectGrasp_Response msg_;
};

class Init_SelectGrasp_Response_selected_pose
{
public:
  explicit Init_SelectGrasp_Response_selected_pose(::roarm_anygrasp_integration::srv::SelectGrasp_Response & msg)
  : msg_(msg)
  {}
  Init_SelectGrasp_Response_confidence_score selected_pose(::roarm_anygrasp_integration::srv::SelectGrasp_Response::_selected_pose_type arg)
  {
    msg_.selected_pose = std::move(arg);
    return Init_SelectGrasp_Response_confidence_score(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::SelectGrasp_Response msg_;
};

class Init_SelectGrasp_Response_message
{
public:
  explicit Init_SelectGrasp_Response_message(::roarm_anygrasp_integration::srv::SelectGrasp_Response & msg)
  : msg_(msg)
  {}
  Init_SelectGrasp_Response_selected_pose message(::roarm_anygrasp_integration::srv::SelectGrasp_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_SelectGrasp_Response_selected_pose(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::SelectGrasp_Response msg_;
};

class Init_SelectGrasp_Response_success
{
public:
  Init_SelectGrasp_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SelectGrasp_Response_message success(::roarm_anygrasp_integration::srv::SelectGrasp_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SelectGrasp_Response_message(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::SelectGrasp_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_anygrasp_integration::srv::SelectGrasp_Response>()
{
  return roarm_anygrasp_integration::srv::builder::Init_SelectGrasp_Response_success();
}

}  // namespace roarm_anygrasp_integration

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__BUILDER_HPP_
