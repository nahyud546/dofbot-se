// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dofbot_interface:msg/JointInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__STRUCT_H_
#define DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/JointInfo in the package dofbot_interface.
typedef struct dofbot_interface__msg__JointInfo
{
  rosidl_runtime_c__String name;
  double joint1;
  double joint2;
  double joint3;
  double joint4;
  double joint5;
  double joint6;
} dofbot_interface__msg__JointInfo;

// Struct for a sequence of dofbot_interface__msg__JointInfo.
typedef struct dofbot_interface__msg__JointInfo__Sequence
{
  dofbot_interface__msg__JointInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dofbot_interface__msg__JointInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__STRUCT_H_
