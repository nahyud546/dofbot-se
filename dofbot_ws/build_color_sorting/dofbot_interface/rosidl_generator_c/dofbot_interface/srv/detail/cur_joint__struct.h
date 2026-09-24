// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dofbot_interface:srv/CurJoint.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__STRUCT_H_
#define DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'srv_joints'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/CurJoint in the package dofbot_interface.
typedef struct dofbot_interface__srv__CurJoint_Request
{
  rosidl_runtime_c__String srv_joints;
} dofbot_interface__srv__CurJoint_Request;

// Struct for a sequence of dofbot_interface__srv__CurJoint_Request.
typedef struct dofbot_interface__srv__CurJoint_Request__Sequence
{
  dofbot_interface__srv__CurJoint_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dofbot_interface__srv__CurJoint_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/CurJoint in the package dofbot_interface.
typedef struct dofbot_interface__srv__CurJoint_Response
{
  double srv_joint1;
  double srv_joint2;
  double srv_joint3;
  double srv_joint4;
  double srv_joint5;
  double srv_joint6;
} dofbot_interface__srv__CurJoint_Response;

// Struct for a sequence of dofbot_interface__srv__CurJoint_Response.
typedef struct dofbot_interface__srv__CurJoint_Response__Sequence
{
  dofbot_interface__srv__CurJoint_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dofbot_interface__srv__CurJoint_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__STRUCT_H_
