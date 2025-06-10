// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from roarm_anygrasp_integration:srv/CalibrateCamera.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__BUILDER_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "roarm_anygrasp_integration/srv/detail/calibrate_camera__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace roarm_anygrasp_integration
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_anygrasp_integration::srv::CalibrateCamera_Request>()
{
  return ::roarm_anygrasp_integration::srv::CalibrateCamera_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace roarm_anygrasp_integration


namespace roarm_anygrasp_integration
{

namespace srv
{

namespace builder
{

class Init_CalibrateCamera_Response_side_reprojection_error
{
public:
  explicit Init_CalibrateCamera_Response_side_reprojection_error(::roarm_anygrasp_integration::srv::CalibrateCamera_Response & msg)
  : msg_(msg)
  {}
  ::roarm_anygrasp_integration::srv::CalibrateCamera_Response side_reprojection_error(::roarm_anygrasp_integration::srv::CalibrateCamera_Response::_side_reprojection_error_type arg)
  {
    msg_.side_reprojection_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::CalibrateCamera_Response msg_;
};

class Init_CalibrateCamera_Response_primary_reprojection_error
{
public:
  explicit Init_CalibrateCamera_Response_primary_reprojection_error(::roarm_anygrasp_integration::srv::CalibrateCamera_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCamera_Response_side_reprojection_error primary_reprojection_error(::roarm_anygrasp_integration::srv::CalibrateCamera_Response::_primary_reprojection_error_type arg)
  {
    msg_.primary_reprojection_error = std::move(arg);
    return Init_CalibrateCamera_Response_side_reprojection_error(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::CalibrateCamera_Response msg_;
};

class Init_CalibrateCamera_Response_message
{
public:
  explicit Init_CalibrateCamera_Response_message(::roarm_anygrasp_integration::srv::CalibrateCamera_Response & msg)
  : msg_(msg)
  {}
  Init_CalibrateCamera_Response_primary_reprojection_error message(::roarm_anygrasp_integration::srv::CalibrateCamera_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_CalibrateCamera_Response_primary_reprojection_error(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::CalibrateCamera_Response msg_;
};

class Init_CalibrateCamera_Response_success
{
public:
  Init_CalibrateCamera_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CalibrateCamera_Response_message success(::roarm_anygrasp_integration::srv::CalibrateCamera_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_CalibrateCamera_Response_message(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::CalibrateCamera_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_anygrasp_integration::srv::CalibrateCamera_Response>()
{
  return roarm_anygrasp_integration::srv::builder::Init_CalibrateCamera_Response_success();
}

}  // namespace roarm_anygrasp_integration

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__BUILDER_HPP_
