// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dofbot_interface:msg/PosInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__POS_INFO__TRAITS_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__POS_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dofbot_interface/msg/detail/pos_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dofbot_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const PosInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: pos1
  {
    out << "pos1: ";
    rosidl_generator_traits::value_to_yaml(msg.pos1, out);
    out << ", ";
  }

  // member: pos2
  {
    out << "pos2: ";
    rosidl_generator_traits::value_to_yaml(msg.pos2, out);
    out << ", ";
  }

  // member: pos3
  {
    out << "pos3: ";
    rosidl_generator_traits::value_to_yaml(msg.pos3, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PosInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: pos1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos1: ";
    rosidl_generator_traits::value_to_yaml(msg.pos1, out);
    out << "\n";
  }

  // member: pos2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos2: ";
    rosidl_generator_traits::value_to_yaml(msg.pos2, out);
    out << "\n";
  }

  // member: pos3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos3: ";
    rosidl_generator_traits::value_to_yaml(msg.pos3, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PosInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dofbot_interface

namespace rosidl_generator_traits
{

[[deprecated("use dofbot_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dofbot_interface::msg::PosInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  dofbot_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dofbot_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const dofbot_interface::msg::PosInfo & msg)
{
  return dofbot_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dofbot_interface::msg::PosInfo>()
{
  return "dofbot_interface::msg::PosInfo";
}

template<>
inline const char * name<dofbot_interface::msg::PosInfo>()
{
  return "dofbot_interface/msg/PosInfo";
}

template<>
struct has_fixed_size<dofbot_interface::msg::PosInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dofbot_interface::msg::PosInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dofbot_interface::msg::PosInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__POS_INFO__TRAITS_HPP_
