// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dofbot_interface:msg/ArmJoint.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__ARM_JOINT__FUNCTIONS_H_
#define DOFBOT_INTERFACE__MSG__DETAIL__ARM_JOINT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dofbot_interface/msg/rosidl_generator_c__visibility_control.h"

#include "dofbot_interface/msg/detail/arm_joint__struct.h"

/// Initialize msg/ArmJoint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dofbot_interface__msg__ArmJoint
 * )) before or use
 * dofbot_interface__msg__ArmJoint__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
bool
dofbot_interface__msg__ArmJoint__init(dofbot_interface__msg__ArmJoint * msg);

/// Finalize msg/ArmJoint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
void
dofbot_interface__msg__ArmJoint__fini(dofbot_interface__msg__ArmJoint * msg);

/// Create msg/ArmJoint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dofbot_interface__msg__ArmJoint__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
dofbot_interface__msg__ArmJoint *
dofbot_interface__msg__ArmJoint__create();

/// Destroy msg/ArmJoint message.
/**
 * It calls
 * dofbot_interface__msg__ArmJoint__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
void
dofbot_interface__msg__ArmJoint__destroy(dofbot_interface__msg__ArmJoint * msg);

/// Check for msg/ArmJoint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
bool
dofbot_interface__msg__ArmJoint__are_equal(const dofbot_interface__msg__ArmJoint * lhs, const dofbot_interface__msg__ArmJoint * rhs);

/// Copy a msg/ArmJoint message.
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
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
bool
dofbot_interface__msg__ArmJoint__copy(
  const dofbot_interface__msg__ArmJoint * input,
  dofbot_interface__msg__ArmJoint * output);

/// Initialize array of msg/ArmJoint messages.
/**
 * It allocates the memory for the number of elements and calls
 * dofbot_interface__msg__ArmJoint__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
bool
dofbot_interface__msg__ArmJoint__Sequence__init(dofbot_interface__msg__ArmJoint__Sequence * array, size_t size);

/// Finalize array of msg/ArmJoint messages.
/**
 * It calls
 * dofbot_interface__msg__ArmJoint__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
void
dofbot_interface__msg__ArmJoint__Sequence__fini(dofbot_interface__msg__ArmJoint__Sequence * array);

/// Create array of msg/ArmJoint messages.
/**
 * It allocates the memory for the array and calls
 * dofbot_interface__msg__ArmJoint__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
dofbot_interface__msg__ArmJoint__Sequence *
dofbot_interface__msg__ArmJoint__Sequence__create(size_t size);

/// Destroy array of msg/ArmJoint messages.
/**
 * It calls
 * dofbot_interface__msg__ArmJoint__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
void
dofbot_interface__msg__ArmJoint__Sequence__destroy(dofbot_interface__msg__ArmJoint__Sequence * array);

/// Check for msg/ArmJoint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
bool
dofbot_interface__msg__ArmJoint__Sequence__are_equal(const dofbot_interface__msg__ArmJoint__Sequence * lhs, const dofbot_interface__msg__ArmJoint__Sequence * rhs);

/// Copy an array of msg/ArmJoint messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dofbot_interface
bool
dofbot_interface__msg__ArmJoint__Sequence__copy(
  const dofbot_interface__msg__ArmJoint__Sequence * input,
  dofbot_interface__msg__ArmJoint__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__ARM_JOINT__FUNCTIONS_H_
