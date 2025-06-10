// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from roarm_anygrasp_integration:srv/GetGraspCandidates.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__STRUCT_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__roarm_anygrasp_integration__srv__GetGraspCandidates_Request __attribute__((deprecated))
#else
# define DEPRECATED__roarm_anygrasp_integration__srv__GetGraspCandidates_Request __declspec(deprecated)
#endif

namespace roarm_anygrasp_integration
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetGraspCandidates_Request_
{
  using Type = GetGraspCandidates_Request_<ContainerAllocator>;

  explicit GetGraspCandidates_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_candidates = 0l;
      this->min_confidence = 0.0f;
    }
  }

  explicit GetGraspCandidates_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_candidates = 0l;
      this->min_confidence = 0.0f;
    }
  }

  // field types and members
  using _num_candidates_type =
    int32_t;
  _num_candidates_type num_candidates;
  using _min_confidence_type =
    float;
  _min_confidence_type min_confidence;

  // setters for named parameter idiom
  Type & set__num_candidates(
    const int32_t & _arg)
  {
    this->num_candidates = _arg;
    return *this;
  }
  Type & set__min_confidence(
    const float & _arg)
  {
    this->min_confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roarm_anygrasp_integration__srv__GetGraspCandidates_Request
    std::shared_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roarm_anygrasp_integration__srv__GetGraspCandidates_Request
    std::shared_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetGraspCandidates_Request_ & other) const
  {
    if (this->num_candidates != other.num_candidates) {
      return false;
    }
    if (this->min_confidence != other.min_confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetGraspCandidates_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetGraspCandidates_Request_

// alias to use template instance with default allocator
using GetGraspCandidates_Request =
  roarm_anygrasp_integration::srv::GetGraspCandidates_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roarm_anygrasp_integration


// Include directives for member types
// Member 'grasp_poses'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'detection_timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__roarm_anygrasp_integration__srv__GetGraspCandidates_Response __attribute__((deprecated))
#else
# define DEPRECATED__roarm_anygrasp_integration__srv__GetGraspCandidates_Response __declspec(deprecated)
#endif

namespace roarm_anygrasp_integration
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetGraspCandidates_Response_
{
  using Type = GetGraspCandidates_Response_<ContainerAllocator>;

  explicit GetGraspCandidates_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : detection_timestamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection_status = "";
      this->total_grasps_detected = 0l;
    }
  }

  explicit GetGraspCandidates_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : detection_status(_alloc),
    detection_timestamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->detection_status = "";
      this->total_grasps_detected = 0l;
    }
  }

  // field types and members
  using _grasp_poses_type =
    std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>>;
  _grasp_poses_type grasp_poses;
  using _confidence_scores_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _confidence_scores_type confidence_scores;
  using _grasp_widths_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _grasp_widths_type grasp_widths;
  using _quality_scores_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _quality_scores_type quality_scores;
  using _original_indices_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _original_indices_type original_indices;
  using _detection_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _detection_status_type detection_status;
  using _total_grasps_detected_type =
    int32_t;
  _total_grasps_detected_type total_grasps_detected;
  using _detection_timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _detection_timestamp_type detection_timestamp;

  // setters for named parameter idiom
  Type & set__grasp_poses(
    const std::vector<geometry_msgs::msg::PoseStamped_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::PoseStamped_<ContainerAllocator>>> & _arg)
  {
    this->grasp_poses = _arg;
    return *this;
  }
  Type & set__confidence_scores(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->confidence_scores = _arg;
    return *this;
  }
  Type & set__grasp_widths(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->grasp_widths = _arg;
    return *this;
  }
  Type & set__quality_scores(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->quality_scores = _arg;
    return *this;
  }
  Type & set__original_indices(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->original_indices = _arg;
    return *this;
  }
  Type & set__detection_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->detection_status = _arg;
    return *this;
  }
  Type & set__total_grasps_detected(
    const int32_t & _arg)
  {
    this->total_grasps_detected = _arg;
    return *this;
  }
  Type & set__detection_timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->detection_timestamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roarm_anygrasp_integration__srv__GetGraspCandidates_Response
    std::shared_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roarm_anygrasp_integration__srv__GetGraspCandidates_Response
    std::shared_ptr<roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetGraspCandidates_Response_ & other) const
  {
    if (this->grasp_poses != other.grasp_poses) {
      return false;
    }
    if (this->confidence_scores != other.confidence_scores) {
      return false;
    }
    if (this->grasp_widths != other.grasp_widths) {
      return false;
    }
    if (this->quality_scores != other.quality_scores) {
      return false;
    }
    if (this->original_indices != other.original_indices) {
      return false;
    }
    if (this->detection_status != other.detection_status) {
      return false;
    }
    if (this->total_grasps_detected != other.total_grasps_detected) {
      return false;
    }
    if (this->detection_timestamp != other.detection_timestamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetGraspCandidates_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetGraspCandidates_Response_

// alias to use template instance with default allocator
using GetGraspCandidates_Response =
  roarm_anygrasp_integration::srv::GetGraspCandidates_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roarm_anygrasp_integration

namespace roarm_anygrasp_integration
{

namespace srv
{

struct GetGraspCandidates
{
  using Request = roarm_anygrasp_integration::srv::GetGraspCandidates_Request;
  using Response = roarm_anygrasp_integration::srv::GetGraspCandidates_Response;
};

}  // namespace srv

}  // namespace roarm_anygrasp_integration

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__GET_GRASP_CANDIDATES__STRUCT_HPP_
