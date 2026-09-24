// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dofbot_interface:msg/Yolov11Detect.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dofbot_interface/msg/detail/yolov11_detect__rosidl_typesupport_introspection_c.h"
#include "dofbot_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dofbot_interface/msg/detail/yolov11_detect__functions.h"
#include "dofbot_interface/msg/detail/yolov11_detect__struct.h"


// Include directives for member types
// Member `result`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dofbot_interface__msg__Yolov11Detect__init(message_memory);
}

void dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_fini_function(void * message_memory)
{
  dofbot_interface__msg__Yolov11Detect__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_message_member_array[3] = {
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dofbot_interface__msg__Yolov11Detect, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "centerx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dofbot_interface__msg__Yolov11Detect, centerx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "centery",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dofbot_interface__msg__Yolov11Detect, centery),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_message_members = {
  "dofbot_interface__msg",  // message namespace
  "Yolov11Detect",  // message name
  3,  // number of fields
  sizeof(dofbot_interface__msg__Yolov11Detect),
  dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_message_member_array,  // message members
  dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_init_function,  // function to initialize message memory (memory has to be allocated)
  dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_message_type_support_handle = {
  0,
  &dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dofbot_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dofbot_interface, msg, Yolov11Detect)() {
  if (!dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_message_type_support_handle.typesupport_identifier) {
    dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dofbot_interface__msg__Yolov11Detect__rosidl_typesupport_introspection_c__Yolov11Detect_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
