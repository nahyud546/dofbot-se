// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dofbot_interface:msg/WidthInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__STRUCT_H_
#define DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/WidthInfo in the package dofbot_interface.
typedef struct dofbot_interface__msg__WidthInfo
{
  float l_x;
  float l_y;
  float r_x;
  float r_y;
} dofbot_interface__msg__WidthInfo;

// Struct for a sequence of dofbot_interface__msg__WidthInfo.
typedef struct dofbot_interface__msg__WidthInfo__Sequence
{
  dofbot_interface__msg__WidthInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dofbot_interface__msg__WidthInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__STRUCT_H_
