// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dofbot_interface:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__POSITION__STRUCT_H_
#define DOFBOT_INTERFACE__MSG__DETAIL__POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Position in the package dofbot_interface.
typedef struct dofbot_interface__msg__Position
{
  float x;
  float y;
  float z;
} dofbot_interface__msg__Position;

// Struct for a sequence of dofbot_interface__msg__Position.
typedef struct dofbot_interface__msg__Position__Sequence
{
  dofbot_interface__msg__Position * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dofbot_interface__msg__Position__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__POSITION__STRUCT_H_
