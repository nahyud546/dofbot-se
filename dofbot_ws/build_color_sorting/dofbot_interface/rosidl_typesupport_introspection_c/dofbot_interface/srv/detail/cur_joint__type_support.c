// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dofbot_interface:srv/CurJoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dofbot_interface/srv/detail/cur_joint__rosidl_typesupport_introspection_c.h"
#include "dofbot_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dofbot_interface/srv/detail/cur_joint__functions.h"
#include "dofbot_interface/srv/detail/cur_joint__struct.h"


// Include directives for member types
// Member `srv_joints`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dofbot_interface__srv__CurJoint_Request__init(message_memory);
}

void dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_fini_function(void * message_memory)
{
  dofbot_interface__srv__CurJoint_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_message_member_array[1] = {
  {
    "srv_joints",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dofbot_interface__srv__CurJoint_Request, srv_joints),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_message_members = {
  "dofbot_interface__srv",  // message namespace
  "CurJoint_Request",  // message name
  1,  // number of fields
  sizeof(dofbot_interface__srv__CurJoint_Request),
  dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_message_member_array,  // message members
  dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_message_type_support_handle = {
  0,
  &dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dofbot_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dofbot_interface, srv, CurJoint_Request)() {
  if (!dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_message_type_support_handle.typesupport_identifier) {
    dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dofbot_interface__srv__CurJoint_Request__rosidl_typesupport_introspection_c__CurJoint_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "dofbot_interface/srv/detail/cur_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "dofbot_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "dofbot_interface/srv/detail/cur_joint__functions.h"
// already included above
// #include "dofbot_interface/srv/detail/cur_joint__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dofbot_interface__srv__CurJoint_Response__init(message_memory);
}

void dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_fini_function(void * message_memory)
{
  dofbot_interface__srv__CurJoint_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_message_member_array[6] = {
  {
    "srv_joint1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dofbot_interface__srv__CurJoint_Response, srv_joint1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "srv_joint2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dofbot_interface__srv__CurJoint_Response, srv_joint2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "srv_joint3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dofbot_interface__srv__CurJoint_Response, srv_joint3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "srv_joint4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dofbot_interface__srv__CurJoint_Response, srv_joint4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "srv_joint5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dofbot_interface__srv__CurJoint_Response, srv_joint5),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "srv_joint6",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dofbot_interface__srv__CurJoint_Response, srv_joint6),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_message_members = {
  "dofbot_interface__srv",  // message namespace
  "CurJoint_Response",  // message name
  6,  // number of fields
  sizeof(dofbot_interface__srv__CurJoint_Response),
  dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_message_member_array,  // message members
  dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_message_type_support_handle = {
  0,
  &dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dofbot_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dofbot_interface, srv, CurJoint_Response)() {
  if (!dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_message_type_support_handle.typesupport_identifier) {
    dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dofbot_interface__srv__CurJoint_Response__rosidl_typesupport_introspection_c__CurJoint_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "dofbot_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "dofbot_interface/srv/detail/cur_joint__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers dofbot_interface__srv__detail__cur_joint__rosidl_typesupport_introspection_c__CurJoint_service_members = {
  "dofbot_interface__srv",  // service namespace
  "CurJoint",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // dofbot_interface__srv__detail__cur_joint__rosidl_typesupport_introspection_c__CurJoint_Request_message_type_support_handle,
  NULL  // response message
  // dofbot_interface__srv__detail__cur_joint__rosidl_typesupport_introspection_c__CurJoint_Response_message_type_support_handle
};

static rosidl_service_type_support_t dofbot_interface__srv__detail__cur_joint__rosidl_typesupport_introspection_c__CurJoint_service_type_support_handle = {
  0,
  &dofbot_interface__srv__detail__cur_joint__rosidl_typesupport_introspection_c__CurJoint_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dofbot_interface, srv, CurJoint_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dofbot_interface, srv, CurJoint_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dofbot_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dofbot_interface, srv, CurJoint)() {
  if (!dofbot_interface__srv__detail__cur_joint__rosidl_typesupport_introspection_c__CurJoint_service_type_support_handle.typesupport_identifier) {
    dofbot_interface__srv__detail__cur_joint__rosidl_typesupport_introspection_c__CurJoint_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)dofbot_interface__srv__detail__cur_joint__rosidl_typesupport_introspection_c__CurJoint_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dofbot_interface, srv, CurJoint_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dofbot_interface, srv, CurJoint_Response)()->data;
  }

  return &dofbot_interface__srv__detail__cur_joint__rosidl_typesupport_introspection_c__CurJoint_service_type_support_handle;
}
