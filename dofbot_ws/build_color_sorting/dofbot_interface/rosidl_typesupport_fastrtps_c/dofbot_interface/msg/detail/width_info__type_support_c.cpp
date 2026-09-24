// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dofbot_interface:msg/WidthInfo.idl
// generated code does not contain a copyright notice
#include "dofbot_interface/msg/detail/width_info__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dofbot_interface/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dofbot_interface/msg/detail/width_info__struct.h"
#include "dofbot_interface/msg/detail/width_info__functions.h"
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


// forward declare type support functions


using _WidthInfo__ros_msg_type = dofbot_interface__msg__WidthInfo;

static bool _WidthInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _WidthInfo__ros_msg_type * ros_message = static_cast<const _WidthInfo__ros_msg_type *>(untyped_ros_message);
  // Field name: l_x
  {
    cdr << ros_message->l_x;
  }

  // Field name: l_y
  {
    cdr << ros_message->l_y;
  }

  // Field name: r_x
  {
    cdr << ros_message->r_x;
  }

  // Field name: r_y
  {
    cdr << ros_message->r_y;
  }

  return true;
}

static bool _WidthInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _WidthInfo__ros_msg_type * ros_message = static_cast<_WidthInfo__ros_msg_type *>(untyped_ros_message);
  // Field name: l_x
  {
    cdr >> ros_message->l_x;
  }

  // Field name: l_y
  {
    cdr >> ros_message->l_y;
  }

  // Field name: r_x
  {
    cdr >> ros_message->r_x;
  }

  // Field name: r_y
  {
    cdr >> ros_message->r_y;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dofbot_interface
size_t get_serialized_size_dofbot_interface__msg__WidthInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _WidthInfo__ros_msg_type * ros_message = static_cast<const _WidthInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name l_x
  {
    size_t item_size = sizeof(ros_message->l_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l_y
  {
    size_t item_size = sizeof(ros_message->l_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r_x
  {
    size_t item_size = sizeof(ros_message->r_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r_y
  {
    size_t item_size = sizeof(ros_message->r_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _WidthInfo__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dofbot_interface__msg__WidthInfo(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dofbot_interface
size_t max_serialized_size_dofbot_interface__msg__WidthInfo(
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

  // member: l_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: l_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: r_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: r_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dofbot_interface__msg__WidthInfo;
    is_plain =
      (
      offsetof(DataType, r_y) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _WidthInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dofbot_interface__msg__WidthInfo(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_WidthInfo = {
  "dofbot_interface::msg",
  "WidthInfo",
  _WidthInfo__cdr_serialize,
  _WidthInfo__cdr_deserialize,
  _WidthInfo__get_serialized_size,
  _WidthInfo__max_serialized_size
};

static rosidl_message_type_support_t _WidthInfo__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_WidthInfo,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dofbot_interface, msg, WidthInfo)() {
  return &_WidthInfo__type_support;
}

#if defined(__cplusplus)
}
#endif
