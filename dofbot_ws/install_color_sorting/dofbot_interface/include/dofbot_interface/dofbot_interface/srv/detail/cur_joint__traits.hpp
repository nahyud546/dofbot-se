// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dofbot_interface:srv/CurJoint.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__TRAITS_HPP_
#define DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dofbot_interface/srv/detail/cur_joint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dofbot_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const CurJoint_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: srv_joints
  {
    out << "srv_joints: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joints, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CurJoint_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: srv_joints
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "srv_joints: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joints, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CurJoint_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dofbot_interface

namespace rosidl_generator_traits
{

[[deprecated("use dofbot_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dofbot_interface::srv::CurJoint_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dofbot_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dofbot_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const dofbot_interface::srv::CurJoint_Request & msg)
{
  return dofbot_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dofbot_interface::srv::CurJoint_Request>()
{
  return "dofbot_interface::srv::CurJoint_Request";
}

template<>
inline const char * name<dofbot_interface::srv::CurJoint_Request>()
{
  return "dofbot_interface/srv/CurJoint_Request";
}

template<>
struct has_fixed_size<dofbot_interface::srv::CurJoint_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dofbot_interface::srv::CurJoint_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dofbot_interface::srv::CurJoint_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dofbot_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const CurJoint_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: srv_joint1
  {
    out << "srv_joint1: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint1, out);
    out << ", ";
  }

  // member: srv_joint2
  {
    out << "srv_joint2: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint2, out);
    out << ", ";
  }

  // member: srv_joint3
  {
    out << "srv_joint3: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint3, out);
    out << ", ";
  }

  // member: srv_joint4
  {
    out << "srv_joint4: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint4, out);
    out << ", ";
  }

  // member: srv_joint5
  {
    out << "srv_joint5: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint5, out);
    out << ", ";
  }

  // member: srv_joint6
  {
    out << "srv_joint6: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint6, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CurJoint_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: srv_joint1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "srv_joint1: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint1, out);
    out << "\n";
  }

  // member: srv_joint2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "srv_joint2: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint2, out);
    out << "\n";
  }

  // member: srv_joint3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "srv_joint3: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint3, out);
    out << "\n";
  }

  // member: srv_joint4
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "srv_joint4: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint4, out);
    out << "\n";
  }

  // member: srv_joint5
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "srv_joint5: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint5, out);
    out << "\n";
  }

  // member: srv_joint6
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "srv_joint6: ";
    rosidl_generator_traits::value_to_yaml(msg.srv_joint6, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CurJoint_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace dofbot_interface

namespace rosidl_generator_traits
{

[[deprecated("use dofbot_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dofbot_interface::srv::CurJoint_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dofbot_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dofbot_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const dofbot_interface::srv::CurJoint_Response & msg)
{
  return dofbot_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dofbot_interface::srv::CurJoint_Response>()
{
  return "dofbot_interface::srv::CurJoint_Response";
}

template<>
inline const char * name<dofbot_interface::srv::CurJoint_Response>()
{
  return "dofbot_interface/srv/CurJoint_Response";
}

template<>
struct has_fixed_size<dofbot_interface::srv::CurJoint_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dofbot_interface::srv::CurJoint_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dofbot_interface::srv::CurJoint_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dofbot_interface::srv::CurJoint>()
{
  return "dofbot_interface::srv::CurJoint";
}

template<>
inline const char * name<dofbot_interface::srv::CurJoint>()
{
  return "dofbot_interface/srv/CurJoint";
}

template<>
struct has_fixed_size<dofbot_interface::srv::CurJoint>
  : std::integral_constant<
    bool,
    has_fixed_size<dofbot_interface::srv::CurJoint_Request>::value &&
    has_fixed_size<dofbot_interface::srv::CurJoint_Response>::value
  >
{
};

template<>
struct has_bounded_size<dofbot_interface::srv::CurJoint>
  : std::integral_constant<
    bool,
    has_bounded_size<dofbot_interface::srv::CurJoint_Request>::value &&
    has_bounded_size<dofbot_interface::srv::CurJoint_Response>::value
  >
{
};

template<>
struct is_service<dofbot_interface::srv::CurJoint>
  : std::true_type
{
};

template<>
struct is_service_request<dofbot_interface::srv::CurJoint_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dofbot_interface::srv::CurJoint_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__TRAITS_HPP_
