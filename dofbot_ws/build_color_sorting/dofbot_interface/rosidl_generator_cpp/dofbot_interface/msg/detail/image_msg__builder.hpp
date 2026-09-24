// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dofbot_interface:msg/ImageMsg.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__IMAGE_MSG__BUILDER_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__IMAGE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dofbot_interface/msg/detail/image_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dofbot_interface
{

namespace msg
{

namespace builder
{

class Init_ImageMsg_data
{
public:
  explicit Init_ImageMsg_data(::dofbot_interface::msg::ImageMsg & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::msg::ImageMsg data(::dofbot_interface::msg::ImageMsg::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::msg::ImageMsg msg_;
};

class Init_ImageMsg_channels
{
public:
  explicit Init_ImageMsg_channels(::dofbot_interface::msg::ImageMsg & msg)
  : msg_(msg)
  {}
  Init_ImageMsg_data channels(::dofbot_interface::msg::ImageMsg::_channels_type arg)
  {
    msg_.channels = std::move(arg);
    return Init_ImageMsg_data(msg_);
  }

private:
  ::dofbot_interface::msg::ImageMsg msg_;
};

class Init_ImageMsg_width
{
public:
  explicit Init_ImageMsg_width(::dofbot_interface::msg::ImageMsg & msg)
  : msg_(msg)
  {}
  Init_ImageMsg_channels width(::dofbot_interface::msg::ImageMsg::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_ImageMsg_channels(msg_);
  }

private:
  ::dofbot_interface::msg::ImageMsg msg_;
};

class Init_ImageMsg_height
{
public:
  Init_ImageMsg_height()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ImageMsg_width height(::dofbot_interface::msg::ImageMsg::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_ImageMsg_width(msg_);
  }

private:
  ::dofbot_interface::msg::ImageMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::msg::ImageMsg>()
{
  return dofbot_interface::msg::builder::Init_ImageMsg_height();
}

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__IMAGE_MSG__BUILDER_HPP_
