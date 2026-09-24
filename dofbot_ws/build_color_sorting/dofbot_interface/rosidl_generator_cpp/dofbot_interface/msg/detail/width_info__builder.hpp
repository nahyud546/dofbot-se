// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dofbot_interface:msg/WidthInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__BUILDER_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dofbot_interface/msg/detail/width_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dofbot_interface
{

namespace msg
{

namespace builder
{

class Init_WidthInfo_r_y
{
public:
  explicit Init_WidthInfo_r_y(::dofbot_interface::msg::WidthInfo & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::msg::WidthInfo r_y(::dofbot_interface::msg::WidthInfo::_r_y_type arg)
  {
    msg_.r_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::msg::WidthInfo msg_;
};

class Init_WidthInfo_r_x
{
public:
  explicit Init_WidthInfo_r_x(::dofbot_interface::msg::WidthInfo & msg)
  : msg_(msg)
  {}
  Init_WidthInfo_r_y r_x(::dofbot_interface::msg::WidthInfo::_r_x_type arg)
  {
    msg_.r_x = std::move(arg);
    return Init_WidthInfo_r_y(msg_);
  }

private:
  ::dofbot_interface::msg::WidthInfo msg_;
};

class Init_WidthInfo_l_y
{
public:
  explicit Init_WidthInfo_l_y(::dofbot_interface::msg::WidthInfo & msg)
  : msg_(msg)
  {}
  Init_WidthInfo_r_x l_y(::dofbot_interface::msg::WidthInfo::_l_y_type arg)
  {
    msg_.l_y = std::move(arg);
    return Init_WidthInfo_r_x(msg_);
  }

private:
  ::dofbot_interface::msg::WidthInfo msg_;
};

class Init_WidthInfo_l_x
{
public:
  Init_WidthInfo_l_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WidthInfo_l_y l_x(::dofbot_interface::msg::WidthInfo::_l_x_type arg)
  {
    msg_.l_x = std::move(arg);
    return Init_WidthInfo_l_y(msg_);
  }

private:
  ::dofbot_interface::msg::WidthInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::msg::WidthInfo>()
{
  return dofbot_interface::msg::builder::Init_WidthInfo_l_x();
}

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__BUILDER_HPP_
