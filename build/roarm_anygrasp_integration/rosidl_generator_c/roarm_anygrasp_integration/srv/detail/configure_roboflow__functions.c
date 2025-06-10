// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
// generated code does not contain a copyright notice
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `api_key`
// Member `workspace_name`
// Member `project_name`
// Member `dataset_version`
// Member `annotation_format`
#include "rosidl_runtime_c/string_functions.h"

bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__init(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * msg)
{
  if (!msg) {
    return false;
  }
  // api_key
  if (!rosidl_runtime_c__String__init(&msg->api_key)) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(msg);
    return false;
  }
  // workspace_name
  if (!rosidl_runtime_c__String__init(&msg->workspace_name)) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(msg);
    return false;
  }
  // project_name
  if (!rosidl_runtime_c__String__init(&msg->project_name)) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(msg);
    return false;
  }
  // dataset_version
  if (!rosidl_runtime_c__String__init(&msg->dataset_version)) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(msg);
    return false;
  }
  // auto_upload
  // include_failed_grasps
  // annotation_format
  if (!rosidl_runtime_c__String__init(&msg->annotation_format)) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(msg);
    return false;
  }
  // min_confidence_for_upload
  return true;
}

void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * msg)
{
  if (!msg) {
    return;
  }
  // api_key
  rosidl_runtime_c__String__fini(&msg->api_key);
  // workspace_name
  rosidl_runtime_c__String__fini(&msg->workspace_name);
  // project_name
  rosidl_runtime_c__String__fini(&msg->project_name);
  // dataset_version
  rosidl_runtime_c__String__fini(&msg->dataset_version);
  // auto_upload
  // include_failed_grasps
  // annotation_format
  rosidl_runtime_c__String__fini(&msg->annotation_format);
  // min_confidence_for_upload
}

bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__are_equal(const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * lhs, const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // api_key
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->api_key), &(rhs->api_key)))
  {
    return false;
  }
  // workspace_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->workspace_name), &(rhs->workspace_name)))
  {
    return false;
  }
  // project_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->project_name), &(rhs->project_name)))
  {
    return false;
  }
  // dataset_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->dataset_version), &(rhs->dataset_version)))
  {
    return false;
  }
  // auto_upload
  if (lhs->auto_upload != rhs->auto_upload) {
    return false;
  }
  // include_failed_grasps
  if (lhs->include_failed_grasps != rhs->include_failed_grasps) {
    return false;
  }
  // annotation_format
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->annotation_format), &(rhs->annotation_format)))
  {
    return false;
  }
  // min_confidence_for_upload
  if (lhs->min_confidence_for_upload != rhs->min_confidence_for_upload) {
    return false;
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__copy(
  const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * input,
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // api_key
  if (!rosidl_runtime_c__String__copy(
      &(input->api_key), &(output->api_key)))
  {
    return false;
  }
  // workspace_name
  if (!rosidl_runtime_c__String__copy(
      &(input->workspace_name), &(output->workspace_name)))
  {
    return false;
  }
  // project_name
  if (!rosidl_runtime_c__String__copy(
      &(input->project_name), &(output->project_name)))
  {
    return false;
  }
  // dataset_version
  if (!rosidl_runtime_c__String__copy(
      &(input->dataset_version), &(output->dataset_version)))
  {
    return false;
  }
  // auto_upload
  output->auto_upload = input->auto_upload;
  // include_failed_grasps
  output->include_failed_grasps = input->include_failed_grasps;
  // annotation_format
  if (!rosidl_runtime_c__String__copy(
      &(input->annotation_format), &(output->annotation_format)))
  {
    return false;
  }
  // min_confidence_for_upload
  output->min_confidence_for_upload = input->min_confidence_for_upload;
  return true;
}

roarm_anygrasp_integration__srv__ConfigureRoboflow_Request *
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * msg = (roarm_anygrasp_integration__srv__ConfigureRoboflow_Request *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request));
  bool success = roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__destroy(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__init(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * data = NULL;

  if (size) {
    data = (roarm_anygrasp_integration__srv__ConfigureRoboflow_Request *)allocator.zero_allocate(size, sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(&data[i - 1]);
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
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__fini(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * array)
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
      roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(&array->data[i]);
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

roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence *
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * array = (roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__destroy(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__are_equal(const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * lhs, const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__copy(
  const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * input,
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * data =
      (roarm_anygrasp_integration__srv__ConfigureRoboflow_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// Member `workspace_url`
// Member `project_url`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__init(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini(msg);
    return false;
  }
  // workspace_url
  if (!rosidl_runtime_c__String__init(&msg->workspace_url)) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini(msg);
    return false;
  }
  // project_url
  if (!rosidl_runtime_c__String__init(&msg->project_url)) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini(msg);
    return false;
  }
  // total_images_in_dataset
  return true;
}

void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // workspace_url
  rosidl_runtime_c__String__fini(&msg->workspace_url);
  // project_url
  rosidl_runtime_c__String__fini(&msg->project_url);
  // total_images_in_dataset
}

bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__are_equal(const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * lhs, const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * rhs)
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
  // workspace_url
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->workspace_url), &(rhs->workspace_url)))
  {
    return false;
  }
  // project_url
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->project_url), &(rhs->project_url)))
  {
    return false;
  }
  // total_images_in_dataset
  if (lhs->total_images_in_dataset != rhs->total_images_in_dataset) {
    return false;
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__copy(
  const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * input,
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * output)
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
  // workspace_url
  if (!rosidl_runtime_c__String__copy(
      &(input->workspace_url), &(output->workspace_url)))
  {
    return false;
  }
  // project_url
  if (!rosidl_runtime_c__String__copy(
      &(input->project_url), &(output->project_url)))
  {
    return false;
  }
  // total_images_in_dataset
  output->total_images_in_dataset = input->total_images_in_dataset;
  return true;
}

roarm_anygrasp_integration__srv__ConfigureRoboflow_Response *
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * msg = (roarm_anygrasp_integration__srv__ConfigureRoboflow_Response *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response));
  bool success = roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__destroy(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__init(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * data = NULL;

  if (size) {
    data = (roarm_anygrasp_integration__srv__ConfigureRoboflow_Response *)allocator.zero_allocate(size, sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini(&data[i - 1]);
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
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__fini(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * array)
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
      roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini(&array->data[i]);
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

roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence *
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * array = (roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence *)allocator.allocate(sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__destroy(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__are_equal(const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * lhs, const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__copy(
  const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * input,
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * data =
      (roarm_anygrasp_integration__srv__ConfigureRoboflow_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
