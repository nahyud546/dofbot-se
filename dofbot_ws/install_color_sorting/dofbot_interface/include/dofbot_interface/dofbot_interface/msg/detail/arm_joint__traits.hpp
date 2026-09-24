// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dofbot_interface:msg/ArmJoint.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__ARM_JOINT__TRAITS_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__ARM_JOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dofbot_interface/msg/detail/arm_joint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dofbot_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArmJoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: run_time
  {
    out << "run_time: ";
    rosidl_generator_traits::value_to_yaml(msg.run_time, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << ", ";
  }

  // member: joints
  {
    if (msg.joints.size() == 0) {
      out << "joints: []";
    } else {
      out << "joints: [";
      size_t pending_items = msg.joints.size();
      for (auto item : msg.joints) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArmJoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: run_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "run_time: ";
    rosidl_generator_traits::value_to_yaml(msg.run_time, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }

  // member: joints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joints.size() == 0) {
      out << "joints: []\n";
    } else {
      out << "joints:\n";
      for (auto item : msg.joints) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArmJoint & msg, bool use_flow_style = false)
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
  const dofbot_interface::msg::ArmJoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  dofbot_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dofbot_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const dofbot_interface::msg::ArmJoint & msg)
{
  return dofbot_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dofbot_interface::msg::ArmJoint>()
{
  return "dofbot_interface::msg::ArmJoint";
}

template<>
inline const char * name<dofbot_interface::msg::ArmJoint>()
{
  return "dofbot_interface/msg/ArmJoint";
}

template<>
struct has_fixed_size<dofbot_interface::msg::ArmJoint>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dofbot_interface::msg::ArmJoint>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dofbot_interface::msg::ArmJoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__ARM_JOINT__TRAITS_HPP_
