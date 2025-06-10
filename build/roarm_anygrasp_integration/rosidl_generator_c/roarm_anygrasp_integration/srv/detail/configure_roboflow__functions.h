// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__FUNCTIONS_H_
#define ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "roarm_anygrasp_integration/msg/rosidl_generator_c__visibility_control.h"

#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__struct.h"

/// Initialize srv/ConfigureRoboflow message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Request
 * )) before or use
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__init(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * msg);

/// Finalize srv/ConfigureRoboflow message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * msg);

/// Create srv/ConfigureRoboflow message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request *
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__create();

/// Destroy srv/ConfigureRoboflow message.
/**
 * It calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__destroy(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * msg);

/// Check for srv/ConfigureRoboflow message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__are_equal(const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * lhs, const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * rhs);

/// Copy a srv/ConfigureRoboflow message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__copy(
  const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * input,
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * output);

/// Initialize array of srv/ConfigureRoboflow messages.
/**
 * It allocates the memory for the number of elements and calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__init(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * array, size_t size);

/// Finalize array of srv/ConfigureRoboflow messages.
/**
 * It calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__fini(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * array);

/// Create array of srv/ConfigureRoboflow messages.
/**
 * It allocates the memory for the array and calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence *
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__create(size_t size);

/// Destroy array of srv/ConfigureRoboflow messages.
/**
 * It calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__destroy(roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * array);

/// Check for srv/ConfigureRoboflow message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__are_equal(const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * lhs, const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * rhs);

/// Copy an array of srv/ConfigureRoboflow messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence__copy(
  const roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * input,
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request__Sequence * output);

/// Initialize srv/ConfigureRoboflow message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Response
 * )) before or use
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__init(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * msg);

/// Finalize srv/ConfigureRoboflow message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * msg);

/// Create srv/ConfigureRoboflow message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response *
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__create();

/// Destroy srv/ConfigureRoboflow message.
/**
 * It calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__destroy(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * msg);

/// Check for srv/ConfigureRoboflow message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__are_equal(const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * lhs, const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * rhs);

/// Copy a srv/ConfigureRoboflow message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__copy(
  const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * input,
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * output);

/// Initialize array of srv/ConfigureRoboflow messages.
/**
 * It allocates the memory for the number of elements and calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__init(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * array, size_t size);

/// Finalize array of srv/ConfigureRoboflow messages.
/**
 * It calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__fini(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * array);

/// Create array of srv/ConfigureRoboflow messages.
/**
 * It allocates the memory for the array and calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence *
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__create(size_t size);

/// Destroy array of srv/ConfigureRoboflow messages.
/**
 * It calls
 * roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
void
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__destroy(roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * array);

/// Check for srv/ConfigureRoboflow message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__are_equal(const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * lhs, const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * rhs);

/// Copy an array of srv/ConfigureRoboflow messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_roarm_anygrasp_integration
bool
roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence__copy(
  const roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * input,
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROARM_ANYGRASP_INTEGRATION__SRV__DETAIL__CONFIGURE_ROBOFLOW__FUNCTIONS_H_
