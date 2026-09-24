// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dofbot_interface:msg/PosInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__POS_INFO__BUILDER_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__POS_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dofbot_interface/msg/detail/pos_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dofbot_interface
{

namespace msg
{

namespace builder
{

class Init_PosInfo_yaw
{
public:
  explicit Init_PosInfo_yaw(::dofbot_interface::msg::PosInfo & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::msg::PosInfo yaw(::dofbot_interface::msg::PosInfo::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::msg::PosInfo msg_;
};

class Init_PosInfo_pitch
{
public:
  explicit Init_PosInfo_pitch(::dofbot_interface::msg::PosInfo & msg)
  : msg_(msg)
  {}
  Init_PosInfo_yaw pitch(::dofbot_interface::msg::PosInfo::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_PosInfo_yaw(msg_);
  }

private:
  ::dofbot_interface::msg::PosInfo msg_;
};

class Init_PosInfo_roll
{
public:
  explicit Init_PosInfo_roll(::dofbot_interface::msg::PosInfo & msg)
  : msg_(msg)
  {}
  Init_PosInfo_pitch roll(::dofbot_interface::msg::PosInfo::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_PosInfo_pitch(msg_);
  }

private:
  ::dofbot_interface::msg::PosInfo msg_;
};

class Init_PosInfo_pos3
{
public:
  explicit Init_PosInfo_pos3(::dofbot_interface::msg::PosInfo & msg)
  : msg_(msg)
  {}
  Init_PosInfo_roll pos3(::dofbot_interface::msg::PosInfo::_pos3_type arg)
  {
    msg_.pos3 = std::move(arg);
    return Init_PosInfo_roll(msg_);
  }

private:
  ::dofbot_interface::msg::PosInfo msg_;
};

class Init_PosInfo_pos2
{
public:
  explicit Init_PosInfo_pos2(::dofbot_interface::msg::PosInfo & msg)
  : msg_(msg)
  {}
  Init_PosInfo_pos3 pos2(::dofbot_interface::msg::PosInfo::_pos2_type arg)
  {
    msg_.pos2 = std::move(arg);
    return Init_PosInfo_pos3(msg_);
  }

private:
  ::dofbot_interface::msg::PosInfo msg_;
};

class Init_PosInfo_pos1
{
public:
  explicit Init_PosInfo_pos1(::dofbot_interface::msg::PosInfo & msg)
  : msg_(msg)
  {}
  Init_PosInfo_pos2 pos1(::dofbot_interface::msg::PosInfo::_pos1_type arg)
  {
    msg_.pos1 = std::move(arg);
    return Init_PosInfo_pos2(msg_);
  }

private:
  ::dofbot_interface::msg::PosInfo msg_;
};

class Init_PosInfo_name
{
public:
  Init_PosInfo_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PosInfo_pos1 name(::dofbot_interface::msg::PosInfo::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_PosInfo_pos1(msg_);
  }

private:
  ::dofbot_interface::msg::PosInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::msg::PosInfo>()
{
  return dofbot_interface::msg::builder::Init_PosInfo_name();
}

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__POS_INFO__BUILDER_HPP_
