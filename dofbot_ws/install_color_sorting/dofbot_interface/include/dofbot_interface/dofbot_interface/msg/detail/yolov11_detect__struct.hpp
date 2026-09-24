// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dofbot_interface:msg/Yolov11Detect.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__STRUCT_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dofbot_interface__msg__Yolov11Detect __attribute__((deprecated))
#else
# define DEPRECATED__dofbot_interface__msg__Yolov11Detect __declspec(deprecated)
#endif

namespace dofbot_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Yolov11Detect_
{
  using Type = Yolov11Detect_<ContainerAllocator>;

  explicit Yolov11Detect_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = "";
      this->centerx = 0.0f;
      this->centery = 0.0f;
    }
  }

  explicit Yolov11Detect_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = "";
      this->centerx = 0.0f;
      this->centery = 0.0f;
    }
  }

  // field types and members
  using _result_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _result_type result;
  using _centerx_type =
    float;
  _centerx_type centerx;
  using _centery_type =
    float;
  _centery_type centery;

  // setters for named parameter idiom
  Type & set__result(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__centerx(
    const float & _arg)
  {
    this->centerx = _arg;
    return *this;
  }
  Type & set__centery(
    const float & _arg)
  {
    this->centery = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dofbot_interface::msg::Yolov11Detect_<ContainerAllocator> *;
  using ConstRawPtr =
    const dofbot_interface::msg::Yolov11Detect_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dofbot_interface::msg::Yolov11Detect_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dofbot_interface::msg::Yolov11Detect_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::msg::Yolov11Detect_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::msg::Yolov11Detect_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::msg::Yolov11Detect_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::msg::Yolov11Detect_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dofbot_interface::msg::Yolov11Detect_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dofbot_interface::msg::Yolov11Detect_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dofbot_interface__msg__Yolov11Detect
    std::shared_ptr<dofbot_interface::msg::Yolov11Detect_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dofbot_interface__msg__Yolov11Detect
    std::shared_ptr<dofbot_interface::msg::Yolov11Detect_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Yolov11Detect_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->centerx != other.centerx) {
      return false;
    }
    if (this->centery != other.centery) {
      return false;
    }
    return true;
  }
  bool operator!=(const Yolov11Detect_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Yolov11Detect_

// alias to use template instance with default allocator
using Yolov11Detect =
  dofbot_interface::msg::Yolov11Detect_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__YOLOV11_DETECT__STRUCT_HPP_
