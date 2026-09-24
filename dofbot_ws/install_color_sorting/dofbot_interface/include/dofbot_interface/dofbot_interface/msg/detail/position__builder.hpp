// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dofbot_interface:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__POSITION__BUILDER_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dofbot_interface/msg/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dofbot_interface
{

namespace msg
{

namespace builder
{

class Init_Position_z
{
public:
  explicit Init_Position_z(::dofbot_interface::msg::Position & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::msg::Position z(::dofbot_interface::msg::Position::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::msg::Position msg_;
};

class Init_Position_y
{
public:
  explicit Init_Position_y(::dofbot_interface::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_z y(::dofbot_interface::msg::Position::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Position_z(msg_);
  }

private:
  ::dofbot_interface::msg::Position msg_;
};

class Init_Position_x
{
public:
  Init_Position_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_y x(::dofbot_interface::msg::Position::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Position_y(msg_);
  }

private:
  ::dofbot_interface::msg::Position msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::msg::Position>()
{
  return dofbot_interface::msg::builder::Init_Position_x();
}

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__POSITION__BUILDER_HPP_
