// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dofbot_interface:msg/ArmJoint.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__ARM_JOINT__BUILDER_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__ARM_JOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dofbot_interface/msg/detail/arm_joint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dofbot_interface
{

namespace msg
{

namespace builder
{

class Init_ArmJoint_joints
{
public:
  explicit Init_ArmJoint_joints(::dofbot_interface::msg::ArmJoint & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::msg::ArmJoint joints(::dofbot_interface::msg::ArmJoint::_joints_type arg)
  {
    msg_.joints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::msg::ArmJoint msg_;
};

class Init_ArmJoint_angle
{
public:
  explicit Init_ArmJoint_angle(::dofbot_interface::msg::ArmJoint & msg)
  : msg_(msg)
  {}
  Init_ArmJoint_joints angle(::dofbot_interface::msg::ArmJoint::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return Init_ArmJoint_joints(msg_);
  }

private:
  ::dofbot_interface::msg::ArmJoint msg_;
};

class Init_ArmJoint_run_time
{
public:
  explicit Init_ArmJoint_run_time(::dofbot_interface::msg::ArmJoint & msg)
  : msg_(msg)
  {}
  Init_ArmJoint_angle run_time(::dofbot_interface::msg::ArmJoint::_run_time_type arg)
  {
    msg_.run_time = std::move(arg);
    return Init_ArmJoint_angle(msg_);
  }

private:
  ::dofbot_interface::msg::ArmJoint msg_;
};

class Init_ArmJoint_id
{
public:
  Init_ArmJoint_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmJoint_run_time id(::dofbot_interface::msg::ArmJoint::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_ArmJoint_run_time(msg_);
  }

private:
  ::dofbot_interface::msg::ArmJoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::msg::ArmJoint>()
{
  return dofbot_interface::msg::builder::Init_ArmJoint_id();
}

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__ARM_JOINT__BUILDER_HPP_
