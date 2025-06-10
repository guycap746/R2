// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from roarm_anygrasp_integration:srv/SelectGrasp.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__STRUCT_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__roarm_anygrasp_integration__srv__SelectGrasp_Request __attribute__((deprecated))
#else
# define DEPRECATED__roarm_anygrasp_integration__srv__SelectGrasp_Request __declspec(deprecated)
#endif

namespace roarm_anygrasp_integration
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SelectGrasp_Request_
{
  using Type = SelectGrasp_Request_<ContainerAllocator>;

  explicit SelectGrasp_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->selected_grasp_index = 0l;
      this->show_more_candidates = false;
    }
  }

  explicit SelectGrasp_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->selected_grasp_index = 0l;
      this->show_more_candidates = false;
    }
  }

  // field types and members
  using _selected_grasp_index_type =
    int32_t;
  _selected_grasp_index_type selected_grasp_index;
  using _show_more_candidates_type =
    bool;
  _show_more_candidates_type show_more_candidates;

  // setters for named parameter idiom
  Type & set__selected_grasp_index(
    const int32_t & _arg)
  {
    this->selected_grasp_index = _arg;
    return *this;
  }
  Type & set__show_more_candidates(
    const bool & _arg)
  {
    this->show_more_candidates = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roarm_anygrasp_integration__srv__SelectGrasp_Request
    std::shared_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roarm_anygrasp_integration__srv__SelectGrasp_Request
    std::shared_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SelectGrasp_Request_ & other) const
  {
    if (this->selected_grasp_index != other.selected_grasp_index) {
      return false;
    }
    if (this->show_more_candidates != other.show_more_candidates) {
      return false;
    }
    return true;
  }
  bool operator!=(const SelectGrasp_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SelectGrasp_Request_

// alias to use template instance with default allocator
using SelectGrasp_Request =
  roarm_anygrasp_integration::srv::SelectGrasp_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roarm_anygrasp_integration


// Include directives for member types
// Member 'selected_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__roarm_anygrasp_integration__srv__SelectGrasp_Response __attribute__((deprecated))
#else
# define DEPRECATED__roarm_anygrasp_integration__srv__SelectGrasp_Response __declspec(deprecated)
#endif

namespace roarm_anygrasp_integration
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SelectGrasp_Response_
{
  using Type = SelectGrasp_Response_<ContainerAllocator>;

  explicit SelectGrasp_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : selected_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->confidence_score = 0.0f;
      this->grasp_width = 0.0f;
    }
  }

  explicit SelectGrasp_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    selected_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->confidence_score = 0.0f;
      this->grasp_width = 0.0f;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _selected_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _selected_pose_type selected_pose;
  using _confidence_score_type =
    float;
  _confidence_score_type confidence_score;
  using _grasp_width_type =
    float;
  _grasp_width_type grasp_width;

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
  Type & set__selected_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->selected_pose = _arg;
    return *this;
  }
  Type & set__confidence_score(
    const float & _arg)
  {
    this->confidence_score = _arg;
    return *this;
  }
  Type & set__grasp_width(
    const float & _arg)
  {
    this->grasp_width = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roarm_anygrasp_integration__srv__SelectGrasp_Response
    std::shared_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roarm_anygrasp_integration__srv__SelectGrasp_Response
    std::shared_ptr<roarm_anygrasp_integration::srv::SelectGrasp_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SelectGrasp_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->selected_pose != other.selected_pose) {
      return false;
    }
    if (this->confidence_score != other.confidence_score) {
      return false;
    }
    if (this->grasp_width != other.grasp_width) {
      return false;
    }
    return true;
  }
  bool operator!=(const SelectGrasp_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SelectGrasp_Response_

// alias to use template instance with default allocator
using SelectGrasp_Response =
  roarm_anygrasp_integration::srv::SelectGrasp_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roarm_anygrasp_integration

namespace roarm_anygrasp_integration
{

namespace srv
{

struct SelectGrasp
{
  using Request = roarm_anygrasp_integration::srv::SelectGrasp_Request;
  using Response = roarm_anygrasp_integration::srv::SelectGrasp_Response;
};

}  // namespace srv

}  // namespace roarm_anygrasp_integration

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__SELECT_GRASP__STRUCT_HPP_
