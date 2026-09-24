// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dofbot_interface:msg/Yolov11Detect.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__BUILDER_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dofbot_interface/msg/detail/yolov11_detect__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dofbot_interface
{

namespace msg
{

namespace builder
{

class Init_Yolov11Detect_centery
{
public:
  explicit Init_Yolov11Detect_centery(::dofbot_interface::msg::Yolov11Detect & msg)
  : msg_(msg)
  {}
  ::dofbot_interface::msg::Yolov11Detect centery(::dofbot_interface::msg::Yolov11Detect::_centery_type arg)
  {
    msg_.centery = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dofbot_interface::msg::Yolov11Detect msg_;
};

class Init_Yolov11Detect_centerx
{
public:
  explicit Init_Yolov11Detect_centerx(::dofbot_interface::msg::Yolov11Detect & msg)
  : msg_(msg)
  {}
  Init_Yolov11Detect_centery centerx(::dofbot_interface::msg::Yolov11Detect::_centerx_type arg)
  {
    msg_.centerx = std::move(arg);
    return Init_Yolov11Detect_centery(msg_);
  }

private:
  ::dofbot_interface::msg::Yolov11Detect msg_;
};

class Init_Yolov11Detect_result
{
public:
  Init_Yolov11Detect_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Yolov11Detect_centerx result(::dofbot_interface::msg::Yolov11Detect::_result_type arg)
  {
    msg_.result = std::move(arg);
    return Init_Yolov11Detect_centerx(msg_);
  }

private:
  ::dofbot_interface::msg::Yolov11Detect msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dofbot_interface::msg::Yolov11Detect>()
{
  return dofbot_interface::msg::builder::Init_Yolov11Detect_result();
}

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__BUILDER_HPP_
