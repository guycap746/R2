// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from roarm_anygrasp_integration:srv/GetGraspCandidates.idl
// generated code does not contain a copyright notice
#include "roarm_anygrasp_integration/srv/detail/get_grasp_candidates__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__init(roarm_anygrasp_integration__srv__GetGraspCandidates_Request * msg)
{
  if (!msg) {
    return false;
  }
  // num_candidates
  // min_confidence
  return true;
}

void
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__fini(roarm_anygrasp_integration__srv__GetGraspCandidates_Request * msg)
{
  if (!msg) {
    return;
  }
  // num_candidates
  // min_confidence
}

bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__are_equal(const roarm_anygrasp_integration__srv__GetGraspCandidates_Request * lhs, const roarm_anygrasp_integration__srv__GetGraspCandidates_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // num_candidates
  if (lhs->num_candidates != rhs->num_candidates) {
    return false;
  }
  // min_confidence
  if (lhs->min_confidence != rhs->min_confidence) {
    return false;
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__copy(
  const roarm_anygrasp_integration__srv__GetGraspCandidates_Request * input,
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // num_candidates
  output->num_candidates = input->num_candidates;
  // min_confidence
  output->min_confidence = input->min_confidence;
  return true;
}

roarm_anygrasp_integration__srv__GetGraspCandidates_Request *
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request * msg = (roarm_anygrasp_integration__srv__GetGraspCandidates_Request *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Request));
  bool success = roarm_anygrasp_integration__srv__GetGraspCandidates_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__destroy(roarm_anygrasp_integration__srv__GetGraspCandidates_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence__init(roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request * data = NULL;

  if (size) {
    data = (roarm_anygrasp_integration__srv__GetGraspCandidates_Request *)allocator.zero_allocate(size, sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = roarm_anygrasp_integration__srv__GetGraspCandidates_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        roarm_anygrasp_integration__srv__GetGraspCandidates_Request__fini(&data[i - 1]);
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
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence__fini(roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence * array)
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
      roarm_anygrasp_integration__srv__GetGraspCandidates_Request__fini(&array->data[i]);
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

roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence *
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence * array = (roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence__destroy(roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence__are_equal(const roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence * lhs, const roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!roarm_anygrasp_integration__srv__GetGraspCandidates_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence__copy(
  const roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence * input,
  roarm_anygrasp_integration__srv__GetGraspCandidates_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    roarm_anygrasp_integration__srv__GetGraspCandidates_Request * data =
      (roarm_anygrasp_integration__srv__GetGraspCandidates_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!roarm_anygrasp_integration__srv__GetGraspCandidates_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          roarm_anygrasp_integration__srv__GetGraspCandidates_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!roarm_anygrasp_integration__srv__GetGraspCandidates_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `grasp_poses`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `confidence_scores`
// Member `grasp_widths`
// Member `quality_scores`
// Member `original_indices`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `detection_status`
#include "rosidl_runtime_c/string_functions.h"
// Member `detection_timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__init(roarm_anygrasp_integration__srv__GetGraspCandidates_Response * msg)
{
  if (!msg) {
    return false;
  }
  // grasp_poses
  if (!geometry_msgs__msg__PoseStamped__Sequence__init(&msg->grasp_poses, 0)) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(msg);
    return false;
  }
  // confidence_scores
  if (!rosidl_runtime_c__float__Sequence__init(&msg->confidence_scores, 0)) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(msg);
    return false;
  }
  // grasp_widths
  if (!rosidl_runtime_c__float__Sequence__init(&msg->grasp_widths, 0)) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(msg);
    return false;
  }
  // quality_scores
  if (!rosidl_runtime_c__float__Sequence__init(&msg->quality_scores, 0)) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(msg);
    return false;
  }
  // original_indices
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->original_indices, 0)) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(msg);
    return false;
  }
  // detection_status
  if (!rosidl_runtime_c__String__init(&msg->detection_status)) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(msg);
    return false;
  }
  // total_grasps_detected
  // detection_timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->detection_timestamp)) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(msg);
    return false;
  }
  return true;
}

void
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(roarm_anygrasp_integration__srv__GetGraspCandidates_Response * msg)
{
  if (!msg) {
    return;
  }
  // grasp_poses
  geometry_msgs__msg__PoseStamped__Sequence__fini(&msg->grasp_poses);
  // confidence_scores
  rosidl_runtime_c__float__Sequence__fini(&msg->confidence_scores);
  // grasp_widths
  rosidl_runtime_c__float__Sequence__fini(&msg->grasp_widths);
  // quality_scores
  rosidl_runtime_c__float__Sequence__fini(&msg->quality_scores);
  // original_indices
  rosidl_runtime_c__int32__Sequence__fini(&msg->original_indices);
  // detection_status
  rosidl_runtime_c__String__fini(&msg->detection_status);
  // total_grasps_detected
  // detection_timestamp
  builtin_interfaces__msg__Time__fini(&msg->detection_timestamp);
}

bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__are_equal(const roarm_anygrasp_integration__srv__GetGraspCandidates_Response * lhs, const roarm_anygrasp_integration__srv__GetGraspCandidates_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // grasp_poses
  if (!geometry_msgs__msg__PoseStamped__Sequence__are_equal(
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
  // grasp_widths
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->grasp_widths), &(rhs->grasp_widths)))
  {
    return false;
  }
  // quality_scores
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->quality_scores), &(rhs->quality_scores)))
  {
    return false;
  }
  // original_indices
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->original_indices), &(rhs->original_indices)))
  {
    return false;
  }
  // detection_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->detection_status), &(rhs->detection_status)))
  {
    return false;
  }
  // total_grasps_detected
  if (lhs->total_grasps_detected != rhs->total_grasps_detected) {
    return false;
  }
  // detection_timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->detection_timestamp), &(rhs->detection_timestamp)))
  {
    return false;
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__copy(
  const roarm_anygrasp_integration__srv__GetGraspCandidates_Response * input,
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // grasp_poses
  if (!geometry_msgs__msg__PoseStamped__Sequence__copy(
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
  // grasp_widths
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->grasp_widths), &(output->grasp_widths)))
  {
    return false;
  }
  // quality_scores
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->quality_scores), &(output->quality_scores)))
  {
    return false;
  }
  // original_indices
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->original_indices), &(output->original_indices)))
  {
    return false;
  }
  // detection_status
  if (!rosidl_runtime_c__String__copy(
      &(input->detection_status), &(output->detection_status)))
  {
    return false;
  }
  // total_grasps_detected
  output->total_grasps_detected = input->total_grasps_detected;
  // detection_timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->detection_timestamp), &(output->detection_timestamp)))
  {
    return false;
  }
  return true;
}

roarm_anygrasp_integration__srv__GetGraspCandidates_Response *
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response * msg = (roarm_anygrasp_integration__srv__GetGraspCandidates_Response *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response));
  bool success = roarm_anygrasp_integration__srv__GetGraspCandidates_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__destroy(roarm_anygrasp_integration__srv__GetGraspCandidates_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence__init(roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response * data = NULL;

  if (size) {
    data = (roarm_anygrasp_integration__srv__GetGraspCandidates_Response *)allocator.zero_allocate(size, sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = roarm_anygrasp_integration__srv__GetGraspCandidates_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(&data[i - 1]);
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
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence__fini(roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence * array)
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
      roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(&array->data[i]);
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

roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence *
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence * array = (roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence__destroy(roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence__are_equal(const roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence * lhs, const roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!roarm_anygrasp_integration__srv__GetGraspCandidates_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence__copy(
  const roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence * input,
  roarm_anygrasp_integration__srv__GetGraspCandidates_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(roarm_anygrasp_integration__srv__GetGraspCandidates_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    roarm_anygrasp_integration__srv__GetGraspCandidates_Response * data =
      (roarm_anygrasp_integration__srv__GetGraspCandidates_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!roarm_anygrasp_integration__srv__GetGraspCandidates_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          roarm_anygrasp_integration__srv__GetGraspCandidates_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!roarm_anygrasp_integration__srv__GetGraspCandidates_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
