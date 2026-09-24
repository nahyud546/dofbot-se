// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dofbot_interface:srv/CurJoint.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__BUILDER_HPP_
#define DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dofbot_interface/srv/detail/cur_joint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dofbot_interface
{

namespace srv
{

namespace builder
{

class Init_CurJoint_Request_srv_joints
{
public:
  Init_CurJoint_Request_srv_joints()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dofbot_interface::srv::CurJoint_Request srv_joints(::dofbot_interface::srv::CurJoint_Request::_srv_joints_type arg)
  {
    msg_.srv_joints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::srv::CurJoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::srv::CurJoint_Request>()
{
  return dofbot_interface::srv::builder::Init_CurJoint_Request_srv_joints();
}

}  // namespace dofbot_interface


namespace dofbot_interface
{

namespace srv
{

namespace builder
{

class Init_CurJoint_Response_srv_joint6
{
public:
  explicit Init_CurJoint_Response_srv_joint6(::dofbot_interface::srv::CurJoint_Response & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::srv::CurJoint_Response srv_joint6(::dofbot_interface::srv::CurJoint_Response::_srv_joint6_type arg)
  {
    msg_.srv_joint6 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::srv::CurJoint_Response msg_;
};

class Init_CurJoint_Response_srv_joint5
{
public:
  explicit Init_CurJoint_Response_srv_joint5(::dofbot_interface::srv::CurJoint_Response & msg)
  : msg_(msg)
  {}
  Init_CurJoint_Response_srv_joint6 srv_joint5(::dofbot_interface::srv::CurJoint_Response::_srv_joint5_type arg)
  {
    msg_.srv_joint5 = std::move(arg);
    return Init_CurJoint_Response_srv_joint6(msg_);
  }

private:
  ::dofbot_interface::srv::CurJoint_Response msg_;
};

class Init_CurJoint_Response_srv_joint4
{
public:
  explicit Init_CurJoint_Response_srv_joint4(::dofbot_interface::srv::CurJoint_Response & msg)
  : msg_(msg)
  {}
  Init_CurJoint_Response_srv_joint5 srv_joint4(::dofbot_interface::srv::CurJoint_Response::_srv_joint4_type arg)
  {
    msg_.srv_joint4 = std::move(arg);
    return Init_CurJoint_Response_srv_joint5(msg_);
  }

private:
  ::dofbot_interface::srv::CurJoint_Response msg_;
};

class Init_CurJoint_Response_srv_joint3
{
public:
  explicit Init_CurJoint_Response_srv_joint3(::dofbot_interface::srv::CurJoint_Response & msg)
  : msg_(msg)
  {}
  Init_CurJoint_Response_srv_joint4 srv_joint3(::dofbot_interface::srv::CurJoint_Response::_srv_joint3_type arg)
  {
    msg_.srv_joint3 = std::move(arg);
    return Init_CurJoint_Response_srv_joint4(msg_);
  }

private:
  ::dofbot_interface::srv::CurJoint_Response msg_;
};

class Init_CurJoint_Response_srv_joint2
{
public:
  explicit Init_CurJoint_Response_srv_joint2(::dofbot_interface::srv::CurJoint_Response & msg)
  : msg_(msg)
  {}
  Init_CurJoint_Response_srv_joint3 srv_joint2(::dofbot_interface::srv::CurJoint_Response::_srv_joint2_type arg)
  {
    msg_.srv_joint2 = std::move(arg);
    return Init_CurJoint_Response_srv_joint3(msg_);
  }

private:
  ::dofbot_interface::srv::CurJoint_Response msg_;
};

class Init_CurJoint_Response_srv_joint1
{
public:
  Init_CurJoint_Response_srv_joint1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CurJoint_Response_srv_joint2 srv_joint1(::dofbot_interface::srv::CurJoint_Response::_srv_joint1_type arg)
  {
    msg_.srv_joint1 = std::move(arg);
    return Init_CurJoint_Response_srv_joint2(msg_);
  }

private:
  ::dofbot_interface::srv::CurJoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::srv::CurJoint_Response>()
{
  return dofbot_interface::srv::builder::Init_CurJoint_Response_srv_joint1();
}

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__BUILDER_HPP_
