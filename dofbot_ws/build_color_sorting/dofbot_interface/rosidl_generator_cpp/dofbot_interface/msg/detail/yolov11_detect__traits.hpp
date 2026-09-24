// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dofbot_interface:msg/Yolov11Detect.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__TRAITS_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dofbot_interface/msg/detail/yolov11_detect__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dofbot_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Yolov11Detect & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << ", ";
  }

  // member: centerx
  {
    out << "centerx: ";
    rosidl_generator_traits::value_to_yaml(msg.centerx, out);
    out << ", ";
  }

  // member: centery
  {
    out << "centery: ";
    rosidl_generator_traits::value_to_yaml(msg.centery, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Yolov11Detect & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }

  // member: centerx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "centerx: ";
    rosidl_generator_traits::value_to_yaml(msg.centerx, out);
    out << "\n";
  }

  // member: centery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "centery: ";
    rosidl_generator_traits::value_to_yaml(msg.centery, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Yolov11Detect & msg, bool use_flow_style = false)
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
  const dofbot_interface::msg::Yolov11Detect & msg,
  std::ostream & out, size_t indentation = 0)
{
  dofbot_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dofbot_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const dofbot_interface::msg::Yolov11Detect & msg)
{
  return dofbot_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dofbot_interface::msg::Yolov11Detect>()
{
  return "dofbot_interface::msg::Yolov11Detect";
}

template<>
inline const char * name<dofbot_interface::msg::Yolov11Detect>()
{
  return "dofbot_interface/msg/Yolov11Detect";
}

template<>
struct has_fixed_size<dofbot_interface::msg::Yolov11Detect>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dofbot_interface::msg::Yolov11Detect>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dofbot_interface::msg::Yolov11Detect>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__TRAITS_HPP_
