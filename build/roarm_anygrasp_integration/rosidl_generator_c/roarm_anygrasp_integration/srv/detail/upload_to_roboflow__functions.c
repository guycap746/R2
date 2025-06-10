// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from roarm_anygrasp_integration:srv/UploadToRoboflow.idl
// generated code does not contain a copyright notice
#include "roarm_anygrasp_integration/srv/detail/upload_to_roboflow__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `rgb_image`
#include "sensor_msgs/msg/detail/image__functions.h"
// Member `grasp_poses`
#include "geometry_msgs/msg/detail/pose_array__functions.h"
// Member `confidence_scores`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `class_labels`
// Member `scene_description`
// Member `object_types`
#include "rosidl_runtime_c/string_functions.h"

bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__init(roarm_anygrasp_integration__srv__UploadToRoboflow_Request * msg)
{
  if (!msg) {
    return false;
  }
  // rgb_image
  if (!sensor_msgs__msg__Image__init(&msg->rgb_image)) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(msg);
    return false;
  }
  // grasp_poses
  if (!geometry_msgs__msg__PoseArray__init(&msg->grasp_poses)) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(msg);
    return false;
  }
  // confidence_scores
  if (!rosidl_runtime_c__float__Sequence__init(&msg->confidence_scores, 0)) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(msg);
    return false;
  }
  // class_labels
  if (!rosidl_runtime_c__String__Sequence__init(&msg->class_labels, 0)) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(msg);
    return false;
  }
  // scene_description
  if (!rosidl_runtime_c__String__init(&msg->scene_description)) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(msg);
    return false;
  }
  // object_types
  if (!rosidl_runtime_c__String__init(&msg->object_types)) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(msg);
    return false;
  }
  // include_annotations
  // upload_immediately
  return true;
}

void
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(roarm_anygrasp_integration__srv__UploadToRoboflow_Request * msg)
{
  if (!msg) {
    return;
  }
  // rgb_image
  sensor_msgs__msg__Image__fini(&msg->rgb_image);
  // grasp_poses
  geometry_msgs__msg__PoseArray__fini(&msg->grasp_poses);
  // confidence_scores
  rosidl_runtime_c__float__Sequence__fini(&msg->confidence_scores);
  // class_labels
  rosidl_runtime_c__String__Sequence__fini(&msg->class_labels);
  // scene_description
  rosidl_runtime_c__String__fini(&msg->scene_description);
  // object_types
  rosidl_runtime_c__String__fini(&msg->object_types);
  // include_annotations
  // upload_immediately
}

bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__are_equal(const roarm_anygrasp_integration__srv__UploadToRoboflow_Request * lhs, const roarm_anygrasp_integration__srv__UploadToRoboflow_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rgb_image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->rgb_image), &(rhs->rgb_image)))
  {
    return false;
  }
  // grasp_poses
  if (!geometry_msgs__msg__PoseArray__are_equal(
      &(lhs->grasp_poses), &(rhs->grasp_poses)))
  {
    return false;
  }
  // confidence_scores
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->confidence_scores), &(rhs->confidence_scores)))
  {
    return false;
  }
  // class_labels
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->class_labels), &(rhs->class_labels)))
  {
    return false;
  }
  // scene_description
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->scene_description), &(rhs->scene_description)))
  {
    return false;
  }
  // object_types
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->object_types), &(rhs->object_types)))
  {
    return false;
  }
  // include_annotations
  if (lhs->include_annotations != rhs->include_annotations) {
    return false;
  }
  // upload_immediately
  if (lhs->upload_immediately != rhs->upload_immediately) {
    return false;
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__copy(
  const roarm_anygrasp_integration__srv__UploadToRoboflow_Request * input,
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // rgb_image
  if (!sensor_msgs__msg__Image__copy(
      &(input->rgb_image), &(output->rgb_image)))
  {
    return false;
  }
  // grasp_poses
  if (!geometry_msgs__msg__PoseArray__copy(
      &(input->grasp_poses), &(output->grasp_poses)))
  {
    return false;
  }
  // confidence_scores
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->confidence_scores), &(output->confidence_scores)))
  {
    return false;
  }
  // class_labels
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->class_labels), &(output->class_labels)))
  {
    return false;
  }
  // scene_description
  if (!rosidl_runtime_c__String__copy(
      &(input->scene_description), &(output->scene_description)))
  {
    return false;
  }
  // object_types
  if (!rosidl_runtime_c__String__copy(
      &(input->object_types), &(output->object_types)))
  {
    return false;
  }
  // include_annotations
  output->include_annotations = input->include_annotations;
  // upload_immediately
  output->upload_immediately = input->upload_immediately;
  return true;
}

roarm_anygrasp_integration__srv__UploadToRoboflow_Request *
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request * msg = (roarm_anygrasp_integration__srv__UploadToRoboflow_Request *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request));
  bool success = roarm_anygrasp_integration__srv__UploadToRoboflow_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__destroy(roarm_anygrasp_integration__srv__UploadToRoboflow_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence__init(roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request * data = NULL;

  if (size) {
    data = (roarm_anygrasp_integration__srv__UploadToRoboflow_Request *)allocator.zero_allocate(size, sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = roarm_anygrasp_integration__srv__UploadToRoboflow_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence__fini(roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence *
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence * array = (roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence__destroy(roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence__are_equal(const roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence * lhs, const roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!roarm_anygrasp_integration__srv__UploadToRoboflow_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence__copy(
  const roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence * input,
  roarm_anygrasp_integration__srv__UploadToRoboflow_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    roarm_anygrasp_integration__srv__UploadToRoboflow_Request * data =
      (roarm_anygrasp_integration__srv__UploadToRoboflow_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!roarm_anygrasp_integration__srv__UploadToRoboflow_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          roarm_anygrasp_integration__srv__UploadToRoboflow_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!roarm_anygrasp_integration__srv__UploadToRoboflow_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `roboflow_image_id`
// Member `upload_url`
// Member `dataset_version`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__init(roarm_anygrasp_integration__srv__UploadToRoboflow_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Response__fini(msg);
    return false;
  }
  // roboflow_image_id
  if (!rosidl_runtime_c__String__init(&msg->roboflow_image_id)) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Response__fini(msg);
    return false;
  }
  // upload_url
  if (!rosidl_runtime_c__String__init(&msg->upload_url)) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Response__fini(msg);
    return false;
  }
  // annotation_count
  // dataset_version
  if (!rosidl_runtime_c__String__init(&msg->dataset_version)) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Response__fini(msg);
    return false;
  }
  return true;
}

void
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__fini(roarm_anygrasp_integration__srv__UploadToRoboflow_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // roboflow_image_id
  rosidl_runtime_c__String__fini(&msg->roboflow_image_id);
  // upload_url
  rosidl_runtime_c__String__fini(&msg->upload_url);
  // annotation_count
  // dataset_version
  rosidl_runtime_c__String__fini(&msg->dataset_version);
}

bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__are_equal(const roarm_anygrasp_integration__srv__UploadToRoboflow_Response * lhs, const roarm_anygrasp_integration__srv__UploadToRoboflow_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // roboflow_image_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->roboflow_image_id), &(rhs->roboflow_image_id)))
  {
    return false;
  }
  // upload_url
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->upload_url), &(rhs->upload_url)))
  {
    return false;
  }
  // annotation_count
  if (lhs->annotation_count != rhs->annotation_count) {
    return false;
  }
  // dataset_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->dataset_version), &(rhs->dataset_version)))
  {
    return false;
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__copy(
  const roarm_anygrasp_integration__srv__UploadToRoboflow_Response * input,
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // roboflow_image_id
  if (!rosidl_runtime_c__String__copy(
      &(input->roboflow_image_id), &(output->roboflow_image_id)))
  {
    return false;
  }
  // upload_url
  if (!rosidl_runtime_c__String__copy(
      &(input->upload_url), &(output->upload_url)))
  {
    return false;
  }
  // annotation_count
  output->annotation_count = input->annotation_count;
  // dataset_version
  if (!rosidl_runtime_c__String__copy(
      &(input->dataset_version), &(output->dataset_version)))
  {
    return false;
  }
  return true;
}

roarm_anygrasp_integration__srv__UploadToRoboflow_Response *
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response * msg = (roarm_anygrasp_integration__srv__UploadToRoboflow_Response *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response));
  bool success = roarm_anygrasp_integration__srv__UploadToRoboflow_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__destroy(roarm_anygrasp_integration__srv__UploadToRoboflow_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence__init(roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response * data = NULL;

  if (size) {
    data = (roarm_anygrasp_integration__srv__UploadToRoboflow_Response *)allocator.zero_allocate(size, sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = roarm_anygrasp_integration__srv__UploadToRoboflow_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        roarm_anygrasp_integration__srv__UploadToRoboflow_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence__fini(roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      roarm_anygrasp_integration__srv__UploadToRoboflow_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence *
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence * array = (roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence__destroy(roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence__are_equal(const roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence * lhs, const roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!roarm_anygrasp_integration__srv__UploadToRoboflow_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence__copy(
  const roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence * input,
  roarm_anygrasp_integration__srv__UploadToRoboflow_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(roarm_anygrasp_integration__srv__UploadToRoboflow_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    roarm_anygrasp_integration__srv__UploadToRoboflow_Response * data =
      (roarm_anygrasp_integration__srv__UploadToRoboflow_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!roarm_anygrasp_integration__srv__UploadToRoboflow_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          roarm_anygrasp_integration__srv__UploadToRoboflow_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!roarm_anygrasp_integration__srv__UploadToRoboflow_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
