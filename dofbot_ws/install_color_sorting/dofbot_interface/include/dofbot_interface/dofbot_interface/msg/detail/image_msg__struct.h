// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dofbot_interface:msg/ImageMsg.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__IMAGE_MSG__STRUCT_H_
#define DOFBOT_INTERFACE__MSG__DETAIL__IMAGE_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ImageMsg in the package dofbot_interface.
typedef struct dofbot_interface__msg__ImageMsg
{
  int32_t height;
  int32_t width;
  int32_t channels;
  rosidl_runtime_c__uint8__Sequence data;
} dofbot_interface__msg__ImageMsg;

// Struct for a sequence of dofbot_interface__msg__ImageMsg.
typedef struct dofbot_interface__msg__ImageMsg__Sequence
{
  dofbot_interface__msg__ImageMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dofbot_interface__msg__ImageMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__IMAGE_MSG__STRUCT_H_
