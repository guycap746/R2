// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from roarm_anygrasp_integration:srv/SelectGrasp.idl
// generated code does not contain a copyright notice
#include "roarm_anygrasp_integration/srv/detail/select_grasp__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
roarm_anygrasp_integration__srv__SelectGrasp_Request__init(roarm_anygrasp_integration__srv__SelectGrasp_Request * msg)
{
  if (!msg) {
    return false;
  }
  // selected_grasp_index
  // show_more_candidates
  return true;
}

void
roarm_anygrasp_integration__srv__SelectGrasp_Request__fini(roarm_anygrasp_integration__srv__SelectGrasp_Request * msg)
{
  if (!msg) {
    return;
  }
  // selected_grasp_index
  // show_more_candidates
}

bool
roarm_anygrasp_integration__srv__SelectGrasp_Request__are_equal(const roarm_anygrasp_integration__srv__SelectGrasp_Request * lhs, const roarm_anygrasp_integration__srv__SelectGrasp_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // selected_grasp_index
  if (lhs->selected_grasp_index != rhs->selected_grasp_index) {
    return false;
  }
  // show_more_candidates
  if (lhs->show_more_candidates != rhs->show_more_candidates) {
    return false;
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__SelectGrasp_Request__copy(
  const roarm_anygrasp_integration__srv__SelectGrasp_Request * input,
  roarm_anygrasp_integration__srv__SelectGrasp_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // selected_grasp_index
  output->selected_grasp_index = input->selected_grasp_index;
  // show_more_candidates
  output->show_more_candidates = input->show_more_candidates;
  return true;
}

roarm_anygrasp_integration__srv__SelectGrasp_Request *
roarm_anygrasp_integration__srv__SelectGrasp_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__SelectGrasp_Request * msg = (roarm_anygrasp_integration__srv__SelectGrasp_Request *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__SelectGrasp_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(roarm_anygrasp_integration__srv__SelectGrasp_Request));
  bool success = roarm_anygrasp_integration__srv__SelectGrasp_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
roarm_anygrasp_integration__srv__SelectGrasp_Request__destroy(roarm_anygrasp_integration__srv__SelectGrasp_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    roarm_anygrasp_integration__srv__SelectGrasp_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence__init(roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__SelectGrasp_Request * data = NULL;

  if (size) {
    data = (roarm_anygrasp_integration__srv__SelectGrasp_Request *)allocator.zero_allocate(size, sizeof(roarm_anygrasp_integration__srv__SelectGrasp_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = roarm_anygrasp_integration__srv__SelectGrasp_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        roarm_anygrasp_integration__srv__SelectGrasp_Request__fini(&data[i - 1]);
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
roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence__fini(roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence * array)
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
      roarm_anygrasp_integration__srv__SelectGrasp_Request__fini(&array->data[i]);
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

roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence *
roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence * array = (roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence__destroy(roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence__are_equal(const roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence * lhs, const roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!roarm_anygrasp_integration__srv__SelectGrasp_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence__copy(
  const roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence * input,
  roarm_anygrasp_integration__srv__SelectGrasp_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(roarm_anygrasp_integration__srv__SelectGrasp_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    roarm_anygrasp_integration__srv__SelectGrasp_Request * data =
      (roarm_anygrasp_integration__srv__SelectGrasp_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!roarm_anygrasp_integration__srv__SelectGrasp_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          roarm_anygrasp_integration__srv__SelectGrasp_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!roarm_anygrasp_integration__srv__SelectGrasp_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"
// Member `selected_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
roarm_anygrasp_integration__srv__SelectGrasp_Response__init(roarm_anygrasp_integration__srv__SelectGrasp_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    roarm_anygrasp_integration__srv__SelectGrasp_Response__fini(msg);
    return false;
  }
  // selected_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->selected_pose)) {
    roarm_anygrasp_integration__srv__SelectGrasp_Response__fini(msg);
    return false;
  }
  // confidence_score
  // grasp_width
  return true;
}

void
roarm_anygrasp_integration__srv__SelectGrasp_Response__fini(roarm_anygrasp_integration__srv__SelectGrasp_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // selected_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->selected_pose);
  // confidence_score
  // grasp_width
}

bool
roarm_anygrasp_integration__srv__SelectGrasp_Response__are_equal(const roarm_anygrasp_integration__srv__SelectGrasp_Response * lhs, const roarm_anygrasp_integration__srv__SelectGrasp_Response * rhs)
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
  // selected_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->selected_pose), &(rhs->selected_pose)))
  {
    return false;
  }
  // confidence_score
  if (lhs->confidence_score != rhs->confidence_score) {
    return false;
  }
  // grasp_width
  if (lhs->grasp_width != rhs->grasp_width) {
    return false;
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__SelectGrasp_Response__copy(
  const roarm_anygrasp_integration__srv__SelectGrasp_Response * input,
  roarm_anygrasp_integration__srv__SelectGrasp_Response * output)
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
  // selected_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->selected_pose), &(output->selected_pose)))
  {
    return false;
  }
  // confidence_score
  output->confidence_score = input->confidence_score;
  // grasp_width
  output->grasp_width = input->grasp_width;
  return true;
}

roarm_anygrasp_integration__srv__SelectGrasp_Response *
roarm_anygrasp_integration__srv__SelectGrasp_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__SelectGrasp_Response * msg = (roarm_anygrasp_integration__srv__SelectGrasp_Response *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__SelectGrasp_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(roarm_anygrasp_integration__srv__SelectGrasp_Response));
  bool success = roarm_anygrasp_integration__srv__SelectGrasp_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
roarm_anygrasp_integration__srv__SelectGrasp_Response__destroy(roarm_anygrasp_integration__srv__SelectGrasp_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    roarm_anygrasp_integration__srv__SelectGrasp_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence__init(roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__SelectGrasp_Response * data = NULL;

  if (size) {
    data = (roarm_anygrasp_integration__srv__SelectGrasp_Response *)allocator.zero_allocate(size, sizeof(roarm_anygrasp_integration__srv__SelectGrasp_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = roarm_anygrasp_integration__srv__SelectGrasp_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        roarm_anygrasp_integration__srv__SelectGrasp_Response__fini(&data[i - 1]);
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
roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence__fini(roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence * array)
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
      roarm_anygrasp_integration__srv__SelectGrasp_Response__fini(&array->data[i]);
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

roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence *
roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence * array = (roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence__destroy(roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence__are_equal(const roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence * lhs, const roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!roarm_anygrasp_integration__srv__SelectGrasp_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence__copy(
  const roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence * input,
  roarm_anygrasp_integration__srv__SelectGrasp_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(roarm_anygrasp_integration__srv__SelectGrasp_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    roarm_anygrasp_integration__srv__SelectGrasp_Response * data =
      (roarm_anygrasp_integration__srv__SelectGrasp_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!roarm_anygrasp_integration__srv__SelectGrasp_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          roarm_anygrasp_integration__srv__SelectGrasp_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!roarm_anygrasp_integration__srv__SelectGrasp_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
