// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dofbot_interface:msg/JointInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__BUILDER_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dofbot_interface/msg/detail/joint_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dofbot_interface
{

namespace msg
{

namespace builder
{

class Init_JointInfo_joint6
{
public:
  explicit Init_JointInfo_joint6(::dofbot_interface::msg::JointInfo & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::msg::JointInfo joint6(::dofbot_interface::msg::JointInfo::_joint6_type arg)
  {
    msg_.joint6 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::msg::JointInfo msg_;
};

class Init_JointInfo_joint5
{
public:
  explicit Init_JointInfo_joint5(::dofbot_interface::msg::JointInfo & msg)
  : msg_(msg)
  {}
  Init_JointInfo_joint6 joint5(::dofbot_interface::msg::JointInfo::_joint5_type arg)
  {
    msg_.joint5 = std::move(arg);
    return Init_JointInfo_joint6(msg_);
  }

private:
  ::dofbot_interface::msg::JointInfo msg_;
};

class Init_JointInfo_joint4
{
public:
  explicit Init_JointInfo_joint4(::dofbot_interface::msg::JointInfo & msg)
  : msg_(msg)
  {}
  Init_JointInfo_joint5 joint4(::dofbot_interface::msg::JointInfo::_joint4_type arg)
  {
    msg_.joint4 = std::move(arg);
    return Init_JointInfo_joint5(msg_);
  }

private:
  ::dofbot_interface::msg::JointInfo msg_;
};

class Init_JointInfo_joint3
{
public:
  explicit Init_JointInfo_joint3(::dofbot_interface::msg::JointInfo & msg)
  : msg_(msg)
  {}
  Init_JointInfo_joint4 joint3(::dofbot_interface::msg::JointInfo::_joint3_type arg)
  {
    msg_.joint3 = std::move(arg);
    return Init_JointInfo_joint4(msg_);
  }

private:
  ::dofbot_interface::msg::JointInfo msg_;
};

class Init_JointInfo_joint2
{
public:
  explicit Init_JointInfo_joint2(::dofbot_interface::msg::JointInfo & msg)
  : msg_(msg)
  {}
  Init_JointInfo_joint3 joint2(::dofbot_interface::msg::JointInfo::_joint2_type arg)
  {
    msg_.joint2 = std::move(arg);
    return Init_JointInfo_joint3(msg_);
  }

private:
  ::dofbot_interface::msg::JointInfo msg_;
};

class Init_JointInfo_joint1
{
public:
  explicit Init_JointInfo_joint1(::dofbot_interface::msg::JointInfo & msg)
  : msg_(msg)
  {}
  Init_JointInfo_joint2 joint1(::dofbot_interface::msg::JointInfo::_joint1_type arg)
  {
    msg_.joint1 = std::move(arg);
    return Init_JointInfo_joint2(msg_);
  }

private:
  ::dofbot_interface::msg::JointInfo msg_;
};

class Init_JointInfo_name
{
public:
  Init_JointInfo_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointInfo_joint1 name(::dofbot_interface::msg::JointInfo::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_JointInfo_joint1(msg_);
  }

private:
  ::dofbot_interface::msg::JointInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::msg::JointInfo>()
{
  return dofbot_interface::msg::builder::Init_JointInfo_name();
}

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__BUILDER_HPP_
