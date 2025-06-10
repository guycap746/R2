// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from roarm_anygrasp_integration:srv/CalibrateCamera.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__roarm_anygrasp_integration__srv__CalibrateCamera_Request __attribute__((deprecated))
#else
# define DEPRECATED__roarm_anygrasp_integration__srv__CalibrateCamera_Request __declspec(deprecated)
#endif

namespace roarm_anygrasp_integration
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CalibrateCamera_Request_
{
  using Type = CalibrateCamera_Request_<ContainerAllocator>;

  explicit CalibrateCamera_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit CalibrateCamera_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roarm_anygrasp_integration__srv__CalibrateCamera_Request
    std::shared_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roarm_anygrasp_integration__srv__CalibrateCamera_Request
    std::shared_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrateCamera_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrateCamera_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrateCamera_Request_

// alias to use template instance with default allocator
using CalibrateCamera_Request =
  roarm_anygrasp_integration::srv::CalibrateCamera_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roarm_anygrasp_integration


#ifndef _WIN32
# define DEPRECATED__roarm_anygrasp_integration__srv__CalibrateCamera_Response __attribute__((deprecated))
#else
# define DEPRECATED__roarm_anygrasp_integration__srv__CalibrateCamera_Response __declspec(deprecated)
#endif

namespace roarm_anygrasp_integration
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CalibrateCamera_Response_
{
  using Type = CalibrateCamera_Response_<ContainerAllocator>;

  explicit CalibrateCamera_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->primary_reprojection_error = 0.0;
      this->side_reprojection_error = 0.0;
    }
  }

  explicit CalibrateCamera_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->primary_reprojection_error = 0.0;
      this->side_reprojection_error = 0.0;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _primary_reprojection_error_type =
    double;
  _primary_reprojection_error_type primary_reprojection_error;
  using _side_reprojection_error_type =
    double;
  _side_reprojection_error_type side_reprojection_error;

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
  Type & set__primary_reprojection_error(
    const double & _arg)
  {
    this->primary_reprojection_error = _arg;
    return *this;
  }
  Type & set__side_reprojection_error(
    const double & _arg)
  {
    this->side_reprojection_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roarm_anygrasp_integration__srv__CalibrateCamera_Response
    std::shared_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roarm_anygrasp_integration__srv__CalibrateCamera_Response
    std::shared_ptr<roarm_anygrasp_integration::srv::CalibrateCamera_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CalibrateCamera_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->primary_reprojection_error != other.primary_reprojection_error) {
      return false;
    }
    if (this->side_reprojection_error != other.side_reprojection_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const CalibrateCamera_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CalibrateCamera_Response_

// alias to use template instance with default allocator
using CalibrateCamera_Response =
  roarm_anygrasp_integration::srv::CalibrateCamera_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roarm_anygrasp_integration

namespace roarm_anygrasp_integration
{

namespace srv
{

struct CalibrateCamera
{
  using Request = roarm_anygrasp_integration::srv::CalibrateCamera_Request;
  using Response = roarm_anygrasp_integration::srv::CalibrateCamera_Response;
};

}  // namespace srv

}  // namespace roarm_anygrasp_integration

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CALIBRATE_CAMERA__STRUCT_HPP_
