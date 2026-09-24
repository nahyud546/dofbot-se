// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dofbot_interface:msg/Yolov11Detect.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__STRUCT_H_
#define DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'result'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Yolov11Detect in the package dofbot_interface.
typedef struct dofbot_interface__msg__Yolov11Detect
{
  rosidl_runtime_c__String result;
  float centerx;
  float centery;
} dofbot_interface__msg__Yolov11Detect;

// Struct for a sequence of dofbot_interface__msg__Yolov11Detect.
typedef struct dofbot_interface__msg__Yolov11Detect__Sequence
{
  dofbot_interface__msg__Yolov11Detect * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dofbot_interface__msg__Yolov11Detect__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__STRUCT_H_
