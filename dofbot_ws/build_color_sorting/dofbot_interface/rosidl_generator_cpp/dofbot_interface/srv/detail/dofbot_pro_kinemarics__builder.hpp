// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dofbot_interface:srv/DofbotProKinemarics.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__SRV__DETAIL__DOFBOT_PRO_KINEMARICS__BUILDER_HPP_
#define DOFBOT_INTERFACE__SRV__DETAIL__DOFBOT_PRO_KINEMARICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dofbot_interface/srv/detail/dofbot_pro_kinemarics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dofbot_interface
{

namespace srv
{

namespace builder
{

class Init_DofbotProKinemarics_Request_kin_name
{
public:
  explicit Init_DofbotProKinemarics_Request_kin_name(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::srv::DofbotProKinemarics_Request kin_name(::dofbot_interface::srv::DofbotProKinemarics_Request::_kin_name_type arg)
  {
    msg_.kin_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_cur_joint6
{
public:
  explicit Init_DofbotProKinemarics_Request_cur_joint6(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_kin_name cur_joint6(::dofbot_interface::srv::DofbotProKinemarics_Request::_cur_joint6_type arg)
  {
    msg_.cur_joint6 = std::move(arg);
    return Init_DofbotProKinemarics_Request_kin_name(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_cur_joint5
{
public:
  explicit Init_DofbotProKinemarics_Request_cur_joint5(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_cur_joint6 cur_joint5(::dofbot_interface::srv::DofbotProKinemarics_Request::_cur_joint5_type arg)
  {
    msg_.cur_joint5 = std::move(arg);
    return Init_DofbotProKinemarics_Request_cur_joint6(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_cur_joint4
{
public:
  explicit Init_DofbotProKinemarics_Request_cur_joint4(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_cur_joint5 cur_joint4(::dofbot_interface::srv::DofbotProKinemarics_Request::_cur_joint4_type arg)
  {
    msg_.cur_joint4 = std::move(arg);
    return Init_DofbotProKinemarics_Request_cur_joint5(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_cur_joint3
{
public:
  explicit Init_DofbotProKinemarics_Request_cur_joint3(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_cur_joint4 cur_joint3(::dofbot_interface::srv::DofbotProKinemarics_Request::_cur_joint3_type arg)
  {
    msg_.cur_joint3 = std::move(arg);
    return Init_DofbotProKinemarics_Request_cur_joint4(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_cur_joint2
{
public:
  explicit Init_DofbotProKinemarics_Request_cur_joint2(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_cur_joint3 cur_joint2(::dofbot_interface::srv::DofbotProKinemarics_Request::_cur_joint2_type arg)
  {
    msg_.cur_joint2 = std::move(arg);
    return Init_DofbotProKinemarics_Request_cur_joint3(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_cur_joint1
{
public:
  explicit Init_DofbotProKinemarics_Request_cur_joint1(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_cur_joint2 cur_joint1(::dofbot_interface::srv::DofbotProKinemarics_Request::_cur_joint1_type arg)
  {
    msg_.cur_joint1 = std::move(arg);
    return Init_DofbotProKinemarics_Request_cur_joint2(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_yaw
{
public:
  explicit Init_DofbotProKinemarics_Request_yaw(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_cur_joint1 yaw(::dofbot_interface::srv::DofbotProKinemarics_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_DofbotProKinemarics_Request_cur_joint1(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_pitch
{
public:
  explicit Init_DofbotProKinemarics_Request_pitch(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_yaw pitch(::dofbot_interface::srv::DofbotProKinemarics_Request::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_DofbotProKinemarics_Request_yaw(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_roll
{
public:
  explicit Init_DofbotProKinemarics_Request_roll(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_pitch roll(::dofbot_interface::srv::DofbotProKinemarics_Request::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_DofbotProKinemarics_Request_pitch(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_tar_z
{
public:
  explicit Init_DofbotProKinemarics_Request_tar_z(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_roll tar_z(::dofbot_interface::srv::DofbotProKinemarics_Request::_tar_z_type arg)
  {
    msg_.tar_z = std::move(arg);
    return Init_DofbotProKinemarics_Request_roll(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_tar_y
{
public:
  explicit Init_DofbotProKinemarics_Request_tar_y(::dofbot_interface::srv::DofbotProKinemarics_Request & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Request_tar_z tar_y(::dofbot_interface::srv::DofbotProKinemarics_Request::_tar_y_type arg)
  {
    msg_.tar_y = std::move(arg);
    return Init_DofbotProKinemarics_Request_tar_z(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

class Init_DofbotProKinemarics_Request_tar_x
{
public:
  Init_DofbotProKinemarics_Request_tar_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DofbotProKinemarics_Request_tar_y tar_x(::dofbot_interface::srv::DofbotProKinemarics_Request::_tar_x_type arg)
  {
    msg_.tar_x = std::move(arg);
    return Init_DofbotProKinemarics_Request_tar_y(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::srv::DofbotProKinemarics_Request>()
{
  return dofbot_interface::srv::builder::Init_DofbotProKinemarics_Request_tar_x();
}

}  // namespace dofbot_interface


namespace dofbot_interface
{

namespace srv
{

namespace builder
{

class Init_DofbotProKinemarics_Response_yaw
{
public:
  explicit Init_DofbotProKinemarics_Response_yaw(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::srv::DofbotProKinemarics_Response yaw(::dofbot_interface::srv::DofbotProKinemarics_Response::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_pitch
{
public:
  explicit Init_DofbotProKinemarics_Response_pitch(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Response_yaw pitch(::dofbot_interface::srv::DofbotProKinemarics_Response::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_DofbotProKinemarics_Response_yaw(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_roll
{
public:
  explicit Init_DofbotProKinemarics_Response_roll(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Response_pitch roll(::dofbot_interface::srv::DofbotProKinemarics_Response::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_DofbotProKinemarics_Response_pitch(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_z
{
public:
  explicit Init_DofbotProKinemarics_Response_z(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Response_roll z(::dofbot_interface::srv::DofbotProKinemarics_Response::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_DofbotProKinemarics_Response_roll(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_y
{
public:
  explicit Init_DofbotProKinemarics_Response_y(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Response_z y(::dofbot_interface::srv::DofbotProKinemarics_Response::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_DofbotProKinemarics_Response_z(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_x
{
public:
  explicit Init_DofbotProKinemarics_Response_x(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Response_y x(::dofbot_interface::srv::DofbotProKinemarics_Response::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_DofbotProKinemarics_Response_y(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_joint6
{
public:
  explicit Init_DofbotProKinemarics_Response_joint6(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Response_x joint6(::dofbot_interface::srv::DofbotProKinemarics_Response::_joint6_type arg)
  {
    msg_.joint6 = std::move(arg);
    return Init_DofbotProKinemarics_Response_x(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_joint5
{
public:
  explicit Init_DofbotProKinemarics_Response_joint5(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Response_joint6 joint5(::dofbot_interface::srv::DofbotProKinemarics_Response::_joint5_type arg)
  {
    msg_.joint5 = std::move(arg);
    return Init_DofbotProKinemarics_Response_joint6(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_joint4
{
public:
  explicit Init_DofbotProKinemarics_Response_joint4(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Response_joint5 joint4(::dofbot_interface::srv::DofbotProKinemarics_Response::_joint4_type arg)
  {
    msg_.joint4 = std::move(arg);
    return Init_DofbotProKinemarics_Response_joint5(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_joint3
{
public:
  explicit Init_DofbotProKinemarics_Response_joint3(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Response_joint4 joint3(::dofbot_interface::srv::DofbotProKinemarics_Response::_joint3_type arg)
  {
    msg_.joint3 = std::move(arg);
    return Init_DofbotProKinemarics_Response_joint4(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_joint2
{
public:
  explicit Init_DofbotProKinemarics_Response_joint2(::dofbot_interface::srv::DofbotProKinemarics_Response & msg)
  : msg_(msg)
  {}
  Init_DofbotProKinemarics_Response_joint3 joint2(::dofbot_interface::srv::DofbotProKinemarics_Response::_joint2_type arg)
  {
    msg_.joint2 = std::move(arg);
    return Init_DofbotProKinemarics_Response_joint3(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

class Init_DofbotProKinemarics_Response_joint1
{
public:
  Init_DofbotProKinemarics_Response_joint1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DofbotProKinemarics_Response_joint2 joint1(::dofbot_interface::srv::DofbotProKinemarics_Response::_joint1_type arg)
  {
    msg_.joint1 = std::move(arg);
    return Init_DofbotProKinemarics_Response_joint2(msg_);
  }

private:
  ::dofbot_interface::srv::DofbotProKinemarics_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::srv::DofbotProKinemarics_Response>()
{
  return dofbot_interface::srv::builder::Init_DofbotProKinemarics_Response_joint1();
}

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__SRV__DETAIL__DOFBOT_PRO_KINEMARICS__BUILDER_HPP_
