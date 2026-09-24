// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dofbot_interface:msg/JointInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__TRAITS_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dofbot_interface/msg/detail/joint_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dofbot_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: joint1
  {
    out << "joint1: ";
    rosidl_generator_traits::value_to_yaml(msg.joint1, out);
    out << ", ";
  }

  // member: joint2
  {
    out << "joint2: ";
    rosidl_generator_traits::value_to_yaml(msg.joint2, out);
    out << ", ";
  }

  // member: joint3
  {
    out << "joint3: ";
    rosidl_generator_traits::value_to_yaml(msg.joint3, out);
    out << ", ";
  }

  // member: joint4
  {
    out << "joint4: ";
    rosidl_generator_traits::value_to_yaml(msg.joint4, out);
    out << ", ";
  }

  // member: joint5
  {
    out << "joint5: ";
    rosidl_generator_traits::value_to_yaml(msg.joint5, out);
    out << ", ";
  }

  // member: joint6
  {
    out << "joint6: ";
    rosidl_generator_traits::value_to_yaml(msg.joint6, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointInfo & msg,
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

  // member: joint1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint1: ";
    rosidl_generator_traits::value_to_yaml(msg.joint1, out);
    out << "\n";
  }

  // member: joint2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint2: ";
    rosidl_generator_traits::value_to_yaml(msg.joint2, out);
    out << "\n";
  }

  // member: joint3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint3: ";
    rosidl_generator_traits::value_to_yaml(msg.joint3, out);
    out << "\n";
  }

  // member: joint4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint4: ";
    rosidl_generator_traits::value_to_yaml(msg.joint4, out);
    out << "\n";
  }

  // member: joint5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint5: ";
    rosidl_generator_traits::value_to_yaml(msg.joint5, out);
    out << "\n";
  }

  // member: joint6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint6: ";
    rosidl_generator_traits::value_to_yaml(msg.joint6, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointInfo & msg, bool use_flow_style = false)
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
  const dofbot_interface::msg::JointInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  dofbot_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dofbot_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const dofbot_interface::msg::JointInfo & msg)
{
  return dofbot_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dofbot_interface::msg::JointInfo>()
{
  return "dofbot_interface::msg::JointInfo";
}

template<>
inline const char * name<dofbot_interface::msg::JointInfo>()
{
  return "dofbot_interface/msg/JointInfo";
}

template<>
struct has_fixed_size<dofbot_interface::msg::JointInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dofbot_interface::msg::JointInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dofbot_interface::msg::JointInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__TRAITS_HPP_
