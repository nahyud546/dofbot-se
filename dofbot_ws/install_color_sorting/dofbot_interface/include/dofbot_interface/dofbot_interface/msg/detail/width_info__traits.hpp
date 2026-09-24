// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dofbot_interface:msg/WidthInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__TRAITS_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dofbot_interface/msg/detail/width_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dofbot_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const WidthInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: l_x
  {
    out << "l_x: ";
    rosidl_generator_traits::value_to_yaml(msg.l_x, out);
    out << ", ";
  }

  // member: l_y
  {
    out << "l_y: ";
    rosidl_generator_traits::value_to_yaml(msg.l_y, out);
    out << ", ";
  }

  // member: r_x
  {
    out << "r_x: ";
    rosidl_generator_traits::value_to_yaml(msg.r_x, out);
    out << ", ";
  }

  // member: r_y
  {
    out << "r_y: ";
    rosidl_generator_traits::value_to_yaml(msg.r_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WidthInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: l_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "l_x: ";
    rosidl_generator_traits::value_to_yaml(msg.l_x, out);
    out << "\n";
  }

  // member: l_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "l_y: ";
    rosidl_generator_traits::value_to_yaml(msg.l_y, out);
    out << "\n";
  }

  // member: r_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r_x: ";
    rosidl_generator_traits::value_to_yaml(msg.r_x, out);
    out << "\n";
  }

  // member: r_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r_y: ";
    rosidl_generator_traits::value_to_yaml(msg.r_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WidthInfo & msg, bool use_flow_style = false)
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
  const dofbot_interface::msg::WidthInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  dofbot_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dofbot_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const dofbot_interface::msg::WidthInfo & msg)
{
  return dofbot_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dofbot_interface::msg::WidthInfo>()
{
  return "dofbot_interface::msg::WidthInfo";
}

template<>
inline const char * name<dofbot_interface::msg::WidthInfo>()
{
  return "dofbot_interface/msg/WidthInfo";
}

template<>
struct has_fixed_size<dofbot_interface::msg::WidthInfo>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dofbot_interface::msg::WidthInfo>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dofbot_interface::msg::WidthInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__TRAITS_HPP_
