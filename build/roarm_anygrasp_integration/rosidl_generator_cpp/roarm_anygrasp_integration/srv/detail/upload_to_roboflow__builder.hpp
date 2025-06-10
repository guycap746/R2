// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from roarm_anygrasp_integration:srv/UploadToRoboflow.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__BUILDER_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace roarm_anygrasp_integration
{

namespace srv
{

namespace builder
{

class Init_UploadToRoboflow_Request_upload_immediately
{
public:
  explicit Init_UploadToRoboflow_Request_upload_immediately(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request & msg)
  : msg_(msg)
  {}
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Request upload_immediately(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request::_upload_immediately_type arg)
  {
    msg_.upload_immediately = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Request msg_;
};

class Init_UploadToRoboflow_Request_include_annotations
{
public:
  explicit Init_UploadToRoboflow_Request_include_annotations(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_UploadToRoboflow_Request_upload_immediately include_annotations(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request::_include_annotations_type arg)
  {
    msg_.include_annotations = std::move(arg);
    return Init_UploadToRoboflow_Request_upload_immediately(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Request msg_;
};

class Init_UploadToRoboflow_Request_object_types
{
public:
  explicit Init_UploadToRoboflow_Request_object_types(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_UploadToRoboflow_Request_include_annotations object_types(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request::_object_types_type arg)
  {
    msg_.object_types = std::move(arg);
    return Init_UploadToRoboflow_Request_include_annotations(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Request msg_;
};

class Init_UploadToRoboflow_Request_scene_description
{
public:
  explicit Init_UploadToRoboflow_Request_scene_description(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_UploadToRoboflow_Request_object_types scene_description(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request::_scene_description_type arg)
  {
    msg_.scene_description = std::move(arg);
    return Init_UploadToRoboflow_Request_object_types(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Request msg_;
};

class Init_UploadToRoboflow_Request_class_labels
{
public:
  explicit Init_UploadToRoboflow_Request_class_labels(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_UploadToRoboflow_Request_scene_description class_labels(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request::_class_labels_type arg)
  {
    msg_.class_labels = std::move(arg);
    return Init_UploadToRoboflow_Request_scene_description(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Request msg_;
};

class Init_UploadToRoboflow_Request_confidence_scores
{
public:
  explicit Init_UploadToRoboflow_Request_confidence_scores(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_UploadToRoboflow_Request_class_labels confidence_scores(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request::_confidence_scores_type arg)
  {
    msg_.confidence_scores = std::move(arg);
    return Init_UploadToRoboflow_Request_class_labels(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Request msg_;
};

class Init_UploadToRoboflow_Request_grasp_poses
{
public:
  explicit Init_UploadToRoboflow_Request_grasp_poses(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request & msg)
  : msg_(msg)
  {}
  Init_UploadToRoboflow_Request_confidence_scores grasp_poses(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request::_grasp_poses_type arg)
  {
    msg_.grasp_poses = std::move(arg);
    return Init_UploadToRoboflow_Request_confidence_scores(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Request msg_;
};

class Init_UploadToRoboflow_Request_rgb_image
{
public:
  Init_UploadToRoboflow_Request_rgb_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UploadToRoboflow_Request_grasp_poses rgb_image(::roarm_anygrasp_integration::srv::UploadToRoboflow_Request::_rgb_image_type arg)
  {
    msg_.rgb_image = std::move(arg);
    return Init_UploadToRoboflow_Request_grasp_poses(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_anygrasp_integration::srv::UploadToRoboflow_Request>()
{
  return roarm_anygrasp_integration::srv::builder::Init_UploadToRoboflow_Request_rgb_image();
}

}  // namespace roarm_anygrasp_integration


namespace roarm_anygrasp_integration
{

namespace srv
{

namespace builder
{

class Init_UploadToRoboflow_Response_dataset_version
{
public:
  explicit Init_UploadToRoboflow_Response_dataset_version(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response & msg)
  : msg_(msg)
  {}
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Response dataset_version(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response::_dataset_version_type arg)
  {
    msg_.dataset_version = std::move(arg);
    return std::move(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Response msg_;
};

class Init_UploadToRoboflow_Response_annotation_count
{
public:
  explicit Init_UploadToRoboflow_Response_annotation_count(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response & msg)
  : msg_(msg)
  {}
  Init_UploadToRoboflow_Response_dataset_version annotation_count(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response::_annotation_count_type arg)
  {
    msg_.annotation_count = std::move(arg);
    return Init_UploadToRoboflow_Response_dataset_version(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Response msg_;
};

class Init_UploadToRoboflow_Response_upload_url
{
public:
  explicit Init_UploadToRoboflow_Response_upload_url(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response & msg)
  : msg_(msg)
  {}
  Init_UploadToRoboflow_Response_annotation_count upload_url(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response::_upload_url_type arg)
  {
    msg_.upload_url = std::move(arg);
    return Init_UploadToRoboflow_Response_annotation_count(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Response msg_;
};

class Init_UploadToRoboflow_Response_roboflow_image_id
{
public:
  explicit Init_UploadToRoboflow_Response_roboflow_image_id(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response & msg)
  : msg_(msg)
  {}
  Init_UploadToRoboflow_Response_upload_url roboflow_image_id(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response::_roboflow_image_id_type arg)
  {
    msg_.roboflow_image_id = std::move(arg);
    return Init_UploadToRoboflow_Response_upload_url(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Response msg_;
};

class Init_UploadToRoboflow_Response_message
{
public:
  explicit Init_UploadToRoboflow_Response_message(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response & msg)
  : msg_(msg)
  {}
  Init_UploadToRoboflow_Response_roboflow_image_id message(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_UploadToRoboflow_Response_roboflow_image_id(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Response msg_;
};

class Init_UploadToRoboflow_Response_success
{
public:
  Init_UploadToRoboflow_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UploadToRoboflow_Response_message success(::roarm_anygrasp_integration::srv::UploadToRoboflow_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_UploadToRoboflow_Response_message(msg_);
  }

private:
  ::roarm_anygrasp_integration::srv::UploadToRoboflow_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::roarm_anygrasp_integration::srv::UploadToRoboflow_Response>()
{
  return roarm_anygrasp_integration::srv::builder::Init_UploadToRoboflow_Response_success();
}

}  // namespace roarm_anygrasp_integration

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__BUILDER_HPP_
