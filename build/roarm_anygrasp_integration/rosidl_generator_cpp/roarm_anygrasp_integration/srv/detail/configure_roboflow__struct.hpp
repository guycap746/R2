// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__STRUCT_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__roarm_anygrasp_integration__srv__ConfigureRoboflow_Request __attribute__((deprecated))
#else
# define DEPRECATED__roarm_anygrasp_integration__srv__ConfigureRoboflow_Request __declspec(deprecated)
#endif

namespace roarm_anygrasp_integration
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ConfigureRoboflow_Request_
{
  using Type = ConfigureRoboflow_Request_<ContainerAllocator>;

  explicit ConfigureRoboflow_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->api_key = "";
      this->workspace_name = "";
      this->project_name = "";
      this->dataset_version = "";
      this->auto_upload = false;
      this->include_failed_grasps = false;
      this->annotation_format = "";
      this->min_confidence_for_upload = 0.0f;
    }
  }

  explicit ConfigureRoboflow_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : api_key(_alloc),
    workspace_name(_alloc),
    project_name(_alloc),
    dataset_version(_alloc),
    annotation_format(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->api_key = "";
      this->workspace_name = "";
      this->project_name = "";
      this->dataset_version = "";
      this->auto_upload = false;
      this->include_failed_grasps = false;
      this->annotation_format = "";
      this->min_confidence_for_upload = 0.0f;
    }
  }

  // field types and members
  using _api_key_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _api_key_type api_key;
  using _workspace_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _workspace_name_type workspace_name;
  using _project_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _project_name_type project_name;
  using _dataset_version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dataset_version_type dataset_version;
  using _auto_upload_type =
    bool;
  _auto_upload_type auto_upload;
  using _include_failed_grasps_type =
    bool;
  _include_failed_grasps_type include_failed_grasps;
  using _annotation_format_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _annotation_format_type annotation_format;
  using _min_confidence_for_upload_type =
    float;
  _min_confidence_for_upload_type min_confidence_for_upload;

  // setters for named parameter idiom
  Type & set__api_key(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->api_key = _arg;
    return *this;
  }
  Type & set__workspace_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->workspace_name = _arg;
    return *this;
  }
  Type & set__project_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->project_name = _arg;
    return *this;
  }
  Type & set__dataset_version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dataset_version = _arg;
    return *this;
  }
  Type & set__auto_upload(
    const bool & _arg)
  {
    this->auto_upload = _arg;
    return *this;
  }
  Type & set__include_failed_grasps(
    const bool & _arg)
  {
    this->include_failed_grasps = _arg;
    return *this;
  }
  Type & set__annotation_format(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->annotation_format = _arg;
    return *this;
  }
  Type & set__min_confidence_for_upload(
    const float & _arg)
  {
    this->min_confidence_for_upload = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roarm_anygrasp_integration__srv__ConfigureRoboflow_Request
    std::shared_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roarm_anygrasp_integration__srv__ConfigureRoboflow_Request
    std::shared_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConfigureRoboflow_Request_ & other) const
  {
    if (this->api_key != other.api_key) {
      return false;
    }
    if (this->workspace_name != other.workspace_name) {
      return false;
    }
    if (this->project_name != other.project_name) {
      return false;
    }
    if (this->dataset_version != other.dataset_version) {
      return false;
    }
    if (this->auto_upload != other.auto_upload) {
      return false;
    }
    if (this->include_failed_grasps != other.include_failed_grasps) {
      return false;
    }
    if (this->annotation_format != other.annotation_format) {
      return false;
    }
    if (this->min_confidence_for_upload != other.min_confidence_for_upload) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConfigureRoboflow_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConfigureRoboflow_Request_

// alias to use template instance with default allocator
using ConfigureRoboflow_Request =
  roarm_anygrasp_integration::srv::ConfigureRoboflow_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roarm_anygrasp_integration


#ifndef _WIN32
# define DEPRECATED__roarm_anygrasp_integration__srv__ConfigureRoboflow_Response __attribute__((deprecated))
#else
# define DEPRECATED__roarm_anygrasp_integration__srv__ConfigureRoboflow_Response __declspec(deprecated)
#endif

namespace roarm_anygrasp_integration
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ConfigureRoboflow_Response_
{
  using Type = ConfigureRoboflow_Response_<ContainerAllocator>;

  explicit ConfigureRoboflow_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->workspace_url = "";
      this->project_url = "";
      this->total_images_in_dataset = 0l;
    }
  }

  explicit ConfigureRoboflow_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    workspace_url(_alloc),
    project_url(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->workspace_url = "";
      this->project_url = "";
      this->total_images_in_dataset = 0l;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _workspace_url_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _workspace_url_type workspace_url;
  using _project_url_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _project_url_type project_url;
  using _total_images_in_dataset_type =
    int32_t;
  _total_images_in_dataset_type total_images_in_dataset;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__workspace_url(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->workspace_url = _arg;
    return *this;
  }
  Type & set__project_url(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->project_url = _arg;
    return *this;
  }
  Type & set__total_images_in_dataset(
    const int32_t & _arg)
  {
    this->total_images_in_dataset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roarm_anygrasp_integration__srv__ConfigureRoboflow_Response
    std::shared_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roarm_anygrasp_integration__srv__ConfigureRoboflow_Response
    std::shared_ptr<roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConfigureRoboflow_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->workspace_url != other.workspace_url) {
      return false;
    }
    if (this->project_url != other.project_url) {
      return false;
    }
    if (this->total_images_in_dataset != other.total_images_in_dataset) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConfigureRoboflow_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConfigureRoboflow_Response_

// alias to use template instance with default allocator
using ConfigureRoboflow_Response =
  roarm_anygrasp_integration::srv::ConfigureRoboflow_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roarm_anygrasp_integration

namespace roarm_anygrasp_integration
{

namespace srv
{

struct ConfigureRoboflow
{
  using Request = roarm_anygrasp_integration::srv::ConfigureRoboflow_Request;
  using Response = roarm_anygrasp_integration::srv::ConfigureRoboflow_Response;
};

}  // namespace srv

}  // namespace roarm_anygrasp_integration

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__STRUCT_HPP_
