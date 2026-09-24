// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dofbot_interface:msg/AprilTagInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__APRIL_TAG_INFO__BUILDER_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__APRIL_TAG_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dofbot_interface/msg/detail/april_tag_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dofbot_interface
{

namespace msg
{

namespace builder
{

class Init_AprilTagInfo_z
{
public:
  explicit Init_AprilTagInfo_z(::dofbot_interface::msg::AprilTagInfo & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::msg::AprilTagInfo z(::dofbot_interface::msg::AprilTagInfo::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::msg::AprilTagInfo msg_;
};

class Init_AprilTagInfo_y
{
public:
  explicit Init_AprilTagInfo_y(::dofbot_interface::msg::AprilTagInfo & msg)
  : msg_(msg)
  {}
  Init_AprilTagInfo_z y(::dofbot_interface::msg::AprilTagInfo::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_AprilTagInfo_z(msg_);
  }

private:
  ::dofbot_interface::msg::AprilTagInfo msg_;
};

class Init_AprilTagInfo_x
{
public:
  explicit Init_AprilTagInfo_x(::dofbot_interface::msg::AprilTagInfo & msg)
  : msg_(msg)
  {}
  Init_AprilTagInfo_y x(::dofbot_interface::msg::AprilTagInfo::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_AprilTagInfo_y(msg_);
  }

private:
  ::dofbot_interface::msg::AprilTagInfo msg_;
};

class Init_AprilTagInfo_id
{
public:
  Init_AprilTagInfo_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AprilTagInfo_x id(::dofbot_interface::msg::AprilTagInfo::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_AprilTagInfo_x(msg_);
  }

private:
  ::dofbot_interface::msg::AprilTagInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::msg::AprilTagInfo>()
{
  return dofbot_interface::msg::builder::Init_AprilTagInfo_id();
}

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__APRIL_TAG_INFO__BUILDER_HPP_
