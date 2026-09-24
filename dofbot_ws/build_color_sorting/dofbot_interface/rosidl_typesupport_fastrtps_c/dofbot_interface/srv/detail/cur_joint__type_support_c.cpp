// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dofbot_interface:srv/CurJoint.idl
// generated code does not contain a copyright notice
#include "dofbot_interface/srv/detail/cur_joint__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dofbot_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dofbot_interface/srv/detail/cur_joint__struct.h"
#include "dofbot_interface/srv/detail/cur_joint__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // srv_joints
#include "rosidl_runtime_c/string_functions.h"  // srv_joints

// forward declare type support functions


using _CurJoint_Request__ros_msg_type = dofbot_interface__srv__CurJoint_Request;

static bool _CurJoint_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CurJoint_Request__ros_msg_type * ros_message = static_cast<const _CurJoint_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: srv_joints
  {
    const rosidl_runtime_c__String * str = &ros_message->srv_joints;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _CurJoint_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CurJoint_Request__ros_msg_type * ros_message = static_cast<_CurJoint_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: srv_joints
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->srv_joints.data) {
      rosidl_runtime_c__String__init(&ros_message->srv_joints);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->srv_joints,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'srv_joints'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dofbot_interface
size_t get_serialized_size_dofbot_interface__srv__CurJoint_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CurJoint_Request__ros_msg_type * ros_message = static_cast<const _CurJoint_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name srv_joints
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->srv_joints.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _CurJoint_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dofbot_interface__srv__CurJoint_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dofbot_interface
size_t max_serialized_size_dofbot_interface__srv__CurJoint_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: srv_joints
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dofbot_interface__srv__CurJoint_Request;
    is_plain =
      (
      offsetof(DataType, srv_joints) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _CurJoint_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dofbot_interface__srv__CurJoint_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CurJoint_Request = {
  "dofbot_interface::srv",
  "CurJoint_Request",
  _CurJoint_Request__cdr_serialize,
  _CurJoint_Request__cdr_deserialize,
  _CurJoint_Request__get_serialized_size,
  _CurJoint_Request__max_serialized_size
};

static rosidl_message_type_support_t _CurJoint_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CurJoint_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dofbot_interface, srv, CurJoint_Request)() {
  return &_CurJoint_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "dofbot_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "dofbot_interface/srv/detail/cur_joint__struct.h"
// already included above
// #include "dofbot_interface/srv/detail/cur_joint__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _CurJoint_Response__ros_msg_type = dofbot_interface__srv__CurJoint_Response;

static bool _CurJoint_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CurJoint_Response__ros_msg_type * ros_message = static_cast<const _CurJoint_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: srv_joint1
  {
    cdr << ros_message->srv_joint1;
  }

  // Field name: srv_joint2
  {
    cdr << ros_message->srv_joint2;
  }

  // Field name: srv_joint3
  {
    cdr << ros_message->srv_joint3;
  }

  // Field name: srv_joint4
  {
    cdr << ros_message->srv_joint4;
  }

  // Field name: srv_joint5
  {
    cdr << ros_message->srv_joint5;
  }

  // Field name: srv_joint6
  {
    cdr << ros_message->srv_joint6;
  }

  return true;
}

static bool _CurJoint_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CurJoint_Response__ros_msg_type * ros_message = static_cast<_CurJoint_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: srv_joint1
  {
    cdr >> ros_message->srv_joint1;
  }

  // Field name: srv_joint2
  {
    cdr >> ros_message->srv_joint2;
  }

  // Field name: srv_joint3
  {
    cdr >> ros_message->srv_joint3;
  }

  // Field name: srv_joint4
  {
    cdr >> ros_message->srv_joint4;
  }

  // Field name: srv_joint5
  {
    cdr >> ros_message->srv_joint5;
  }

  // Field name: srv_joint6
  {
    cdr >> ros_message->srv_joint6;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dofbot_interface
size_t get_serialized_size_dofbot_interface__srv__CurJoint_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CurJoint_Response__ros_msg_type * ros_message = static_cast<const _CurJoint_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name srv_joint1
  {
    size_t item_size = sizeof(ros_message->srv_joint1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name srv_joint2
  {
    size_t item_size = sizeof(ros_message->srv_joint2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name srv_joint3
  {
    size_t item_size = sizeof(ros_message->srv_joint3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name srv_joint4
  {
    size_t item_size = sizeof(ros_message->srv_joint4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name srv_joint5
  {
    size_t item_size = sizeof(ros_message->srv_joint5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name srv_joint6
  {
    size_t item_size = sizeof(ros_message->srv_joint6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CurJoint_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dofbot_interface__srv__CurJoint_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dofbot_interface
size_t max_serialized_size_dofbot_interface__srv__CurJoint_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: srv_joint1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: srv_joint2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: srv_joint3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: srv_joint4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: srv_joint5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: srv_joint6
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dofbot_interface__srv__CurJoint_Response;
    is_plain =
      (
      offsetof(DataType, srv_joint6) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _CurJoint_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dofbot_interface__srv__CurJoint_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CurJoint_Response = {
  "dofbot_interface::srv",
  "CurJoint_Response",
  _CurJoint_Response__cdr_serialize,
  _CurJoint_Response__cdr_deserialize,
  _CurJoint_Response__get_serialized_size,
  _CurJoint_Response__max_serialized_size
};

static rosidl_message_type_support_t _CurJoint_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CurJoint_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dofbot_interface, srv, CurJoint_Response)() {
  return &_CurJoint_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "dofbot_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dofbot_interface/srv/cur_joint.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t CurJoint__callbacks = {
  "dofbot_interface::srv",
  "CurJoint",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dofbot_interface, srv, CurJoint_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dofbot_interface, srv, CurJoint_Response)(),
};

static rosidl_service_type_support_t CurJoint__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &CurJoint__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dofbot_interface, srv, CurJoint)() {
  return &CurJoint__handle;
}

#if defined(__cplusplus)
}
#endif
