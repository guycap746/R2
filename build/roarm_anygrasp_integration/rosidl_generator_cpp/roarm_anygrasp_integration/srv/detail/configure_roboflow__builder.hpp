// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__BUILDER_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace roarm_anygrasp_integration
{

namespace srv
{

namespace builder
{

class Init_ConfigureRoboflow_Request_min_confidence_for_upload
{
public:
  explicit Init_ConfigureRoboflow_Request_min_confidence_for_upload(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & msg)
  : msg_(msg)
  {}
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request min_confidence_for_upload(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request::_min_confidence_for_upload_type arg)
  {
    msg_.min_confidence_for_upload = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request msg_;
};

class Init_ConfigureRoboflow_Request_annotation_format
{
public:
  explicit Init_ConfigureRoboflow_Request_annotation_format(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureRoboflow_Request_min_confidence_for_upload annotation_format(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request::_annotation_format_type arg)
  {
    msg_.annotation_format = std::move(arg);
    return Init_ConfigureRoboflow_Request_min_confidence_for_upload(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request msg_;
};

class Init_ConfigureRoboflow_Request_include_failed_grasps
{
public:
  explicit Init_ConfigureRoboflow_Request_include_failed_grasps(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureRoboflow_Request_annotation_format include_failed_grasps(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request::_include_failed_grasps_type arg)
  {
    msg_.include_failed_grasps = std::move(arg);
    return Init_ConfigureRoboflow_Request_annotation_format(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request msg_;
};

class Init_ConfigureRoboflow_Request_auto_upload
{
public:
  explicit Init_ConfigureRoboflow_Request_auto_upload(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureRoboflow_Request_include_failed_grasps auto_upload(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request::_auto_upload_type arg)
  {
    msg_.auto_upload = std::move(arg);
    return Init_ConfigureRoboflow_Request_include_failed_grasps(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request msg_;
};

class Init_ConfigureRoboflow_Request_dataset_version
{
public:
  explicit Init_ConfigureRoboflow_Request_dataset_version(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureRoboflow_Request_auto_upload dataset_version(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request::_dataset_version_type arg)
  {
    msg_.dataset_version = std::move(arg);
    return Init_ConfigureRoboflow_Request_auto_upload(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request msg_;
};

class Init_ConfigureRoboflow_Request_project_name
{
public:
  explicit Init_ConfigureRoboflow_Request_project_name(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureRoboflow_Request_dataset_version project_name(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request::_project_name_type arg)
  {
    msg_.project_name = std::move(arg);
    return Init_ConfigureRoboflow_Request_dataset_version(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request msg_;
};

class Init_ConfigureRoboflow_Request_workspace_name
{
public:
  explicit Init_ConfigureRoboflow_Request_workspace_name(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_ConfigureRoboflow_Request_project_name workspace_name(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request::_workspace_name_type arg)
  {
    msg_.workspace_name = std::move(arg);
    return Init_ConfigureRoboflow_Request_project_name(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request msg_;
};

class Init_ConfigureRoboflow_Request_api_key
{
public:
  Init_ConfigureRoboflow_Request_api_key()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConfigureRoboflow_Request_workspace_name api_key(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request::_api_key_type arg)
  {
    msg_.api_key = std::move(arg);
    return Init_ConfigureRoboflow_Request_workspace_name(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_anygrasp_integration::srv::ConfigureRoboflow_Request>()
{
  return roarm_anygrasp_integration::srv::builder::Init_ConfigureRoboflow_Request_api_key();
}

}  // namespace roarm_anygrasp_integration


namespace roarm_anygrasp_integration
{

namespace srv
{

namespace builder
{

class Init_ConfigureRoboflow_Response_total_images_in_dataset
{
public:
  explicit Init_ConfigureRoboflow_Response_total_images_in_dataset(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response & msg)
  : msg_(msg)
  {}
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response total_images_in_dataset(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response::_total_images_in_dataset_type arg)
  {
    msg_.total_images_in_dataset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response msg_;
};

class Init_ConfigureRoboflow_Response_project_url
{
public:
  explicit Init_ConfigureRoboflow_Response_project_url(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response & msg)
  : msg_(msg)
  {}
  Init_ConfigureRoboflow_Response_total_images_in_dataset project_url(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response::_project_url_type arg)
  {
    msg_.project_url = std::move(arg);
    return Init_ConfigureRoboflow_Response_total_images_in_dataset(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response msg_;
};

class Init_ConfigureRoboflow_Response_workspace_url
{
public:
  explicit Init_ConfigureRoboflow_Response_workspace_url(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response & msg)
  : msg_(msg)
  {}
  Init_ConfigureRoboflow_Response_project_url workspace_url(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response::_workspace_url_type arg)
  {
    msg_.workspace_url = std::move(arg);
    return Init_ConfigureRoboflow_Response_project_url(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response msg_;
};

class Init_ConfigureRoboflow_Response_message
{
public:
  explicit Init_ConfigureRoboflow_Response_message(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response & msg)
  : msg_(msg)
  {}
  Init_ConfigureRoboflow_Response_workspace_url message(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ConfigureRoboflow_Response_workspace_url(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response msg_;
};

class Init_ConfigureRoboflow_Response_success
{
public:
  Init_ConfigureRoboflow_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConfigureRoboflow_Response_message success(::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ConfigureRoboflow_Response_message(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_anygrasp_integration::srv::ConfigureRoboflow_Response>()
{
  return roarm_anygrasp_integration::srv::builder::Init_ConfigureRoboflow_Response_success();
}

}  // namespace roarm_anygrasp_integration

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__BUILDER_HPP_
