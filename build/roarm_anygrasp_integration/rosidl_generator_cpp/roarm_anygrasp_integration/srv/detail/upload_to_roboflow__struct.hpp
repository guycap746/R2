// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from roarm_anygrasp_integration:srv/UploadToRoboflow.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__STRUCT_HPP_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'rgb_image'
#include "sensor_msgs/msg/detail/image__struct.hpp"
// Member 'grasp_poses'
#include "geometry_msgs/msg/detail/pose_array__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__roarm_anygrasp_integration__srv__UploadToRoboflow_Request __attribute__((deprecated))
#else
# define DEPRECATED__roarm_anygrasp_integration__srv__UploadToRoboflow_Request __declspec(deprecated)
#endif

namespace roarm_anygrasp_integration
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UploadToRoboflow_Request_
{
  using Type = UploadToRoboflow_Request_<ContainerAllocator>;

  explicit UploadToRoboflow_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : rgb_image(_init),
    grasp_poses(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->scene_description = "";
      this->object_types = "";
      this->include_annotations = false;
      this->upload_immediately = false;
    }
  }

  explicit UploadToRoboflow_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : rgb_image(_alloc, _init),
    grasp_poses(_alloc, _init),
    scene_description(_alloc),
    object_types(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->scene_description = "";
      this->object_types = "";
      this->include_annotations = false;
      this->upload_immediately = false;
    }
  }

  // field types and members
  using _rgb_image_type =
    sensor_msgs::msg::Image_<ContainerAllocator>;
  _rgb_image_type rgb_image;
  using _grasp_poses_type =
    geometry_msgs::msg::PoseArray_<ContainerAllocator>;
  _grasp_poses_type grasp_poses;
  using _confidence_scores_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _confidence_scores_type confidence_scores;
  using _class_labels_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _class_labels_type class_labels;
  using _scene_description_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _scene_description_type scene_description;
  using _object_types_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _object_types_type object_types;
  using _include_annotations_type =
    bool;
  _include_annotations_type include_annotations;
  using _upload_immediately_type =
    bool;
  _upload_immediately_type upload_immediately;

  // setters for named parameter idiom
  Type & set__rgb_image(
    const sensor_msgs::msg::Image_<ContainerAllocator> & _arg)
  {
    this->rgb_image = _arg;
    return *this;
  }
  Type & set__grasp_poses(
    const geometry_msgs::msg::PoseArray_<ContainerAllocator> & _arg)
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
  Type & set__class_labels(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->class_labels = _arg;
    return *this;
  }
  Type & set__scene_description(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->scene_description = _arg;
    return *this;
  }
  Type & set__object_types(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->object_types = _arg;
    return *this;
  }
  Type & set__include_annotations(
    const bool & _arg)
  {
    this->include_annotations = _arg;
    return *this;
  }
  Type & set__upload_immediately(
    const bool & _arg)
  {
    this->upload_immediately = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roarm_anygrasp_integration__srv__UploadToRoboflow_Request
    std::shared_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roarm_anygrasp_integration__srv__UploadToRoboflow_Request
    std::shared_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UploadToRoboflow_Request_ & other) const
  {
    if (this->rgb_image != other.rgb_image) {
      return false;
    }
    if (this->grasp_poses != other.grasp_poses) {
      return false;
    }
    if (this->confidence_scores != other.confidence_scores) {
      return false;
    }
    if (this->class_labels != other.class_labels) {
      return false;
    }
    if (this->scene_description != other.scene_description) {
      return false;
    }
    if (this->object_types != other.object_types) {
      return false;
    }
    if (this->include_annotations != other.include_annotations) {
      return false;
    }
    if (this->upload_immediately != other.upload_immediately) {
      return false;
    }
    return true;
  }
  bool operator!=(const UploadToRoboflow_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UploadToRoboflow_Request_

// alias to use template instance with default allocator
using UploadToRoboflow_Request =
  roarm_anygrasp_integration::srv::UploadToRoboflow_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roarm_anygrasp_integration


#ifndef _WIN32
# define DEPRECATED__roarm_anygrasp_integration__srv__UploadToRoboflow_Response __attribute__((deprecated))
#else
# define DEPRECATED__roarm_anygrasp_integration__srv__UploadToRoboflow_Response __declspec(deprecated)
#endif

namespace roarm_anygrasp_integration
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UploadToRoboflow_Response_
{
  using Type = UploadToRoboflow_Response_<ContainerAllocator>;

  explicit UploadToRoboflow_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->roboflow_image_id = "";
      this->upload_url = "";
      this->annotation_count = 0l;
      this->dataset_version = "";
    }
  }

  explicit UploadToRoboflow_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc),
    roboflow_image_id(_alloc),
    upload_url(_alloc),
    dataset_version(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
      this->roboflow_image_id = "";
      this->upload_url = "";
      this->annotation_count = 0l;
      this->dataset_version = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _roboflow_image_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _roboflow_image_id_type roboflow_image_id;
  using _upload_url_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _upload_url_type upload_url;
  using _annotation_count_type =
    int32_t;
  _annotation_count_type annotation_count;
  using _dataset_version_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dataset_version_type dataset_version;

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
  Type & set__roboflow_image_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->roboflow_image_id = _arg;
    return *this;
  }
  Type & set__upload_url(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->upload_url = _arg;
    return *this;
  }
  Type & set__annotation_count(
    const int32_t & _arg)
  {
    this->annotation_count = _arg;
    return *this;
  }
  Type & set__dataset_version(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dataset_version = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__roarm_anygrasp_integration__srv__UploadToRoboflow_Response
    std::shared_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__roarm_anygrasp_integration__srv__UploadToRoboflow_Response
    std::shared_ptr<roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UploadToRoboflow_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->roboflow_image_id != other.roboflow_image_id) {
      return false;
    }
    if (this->upload_url != other.upload_url) {
      return false;
    }
    if (this->annotation_count != other.annotation_count) {
      return false;
    }
    if (this->dataset_version != other.dataset_version) {
      return false;
    }
    return true;
  }
  bool operator!=(const UploadToRoboflow_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UploadToRoboflow_Response_

// alias to use template instance with default allocator
using UploadToRoboflow_Response =
  roarm_anygrasp_integration::srv::UploadToRoboflow_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace roarm_anygrasp_integration

namespace roarm_anygrasp_integration
{

namespace srv
{

struct UploadToRoboflow
{
  using Request = roarm_anygrasp_integration::srv::UploadToRoboflow_Request;
  using Response = roarm_anygrasp_integration::srv::UploadToRoboflow_Response;
};

}  // namespace srv

}  // namespace roarm_anygrasp_integration

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__UPLOAD_TO_ROBOFLOW__STRUCT_HPP_
