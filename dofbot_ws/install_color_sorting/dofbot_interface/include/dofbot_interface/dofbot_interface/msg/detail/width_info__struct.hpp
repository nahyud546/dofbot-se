// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dofbot_interface:msg/WidthInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__STRUCT_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dofbot_interface__msg__WidthInfo __attribute__((deprecated))
#else
# define DEPRECATED__dofbot_interface__msg__WidthInfo __declspec(deprecated)
#endif

namespace dofbot_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WidthInfo_
{
  using Type = WidthInfo_<ContainerAllocator>;

  explicit WidthInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->l_x = 0.0f;
      this->l_y = 0.0f;
      this->r_x = 0.0f;
      this->r_y = 0.0f;
    }
  }

  explicit WidthInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->l_x = 0.0f;
      this->l_y = 0.0f;
      this->r_x = 0.0f;
      this->r_y = 0.0f;
    }
  }

  // field types and members
  using _l_x_type =
    float;
  _l_x_type l_x;
  using _l_y_type =
    float;
  _l_y_type l_y;
  using _r_x_type =
    float;
  _r_x_type r_x;
  using _r_y_type =
    float;
  _r_y_type r_y;

  // setters for named parameter idiom
  Type & set__l_x(
    const float & _arg)
  {
    this->l_x = _arg;
    return *this;
  }
  Type & set__l_y(
    const float & _arg)
  {
    this->l_y = _arg;
    return *this;
  }
  Type & set__r_x(
    const float & _arg)
  {
    this->r_x = _arg;
    return *this;
  }
  Type & set__r_y(
    const float & _arg)
  {
    this->r_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dofbot_interface::msg::WidthInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const dofbot_interface::msg::WidthInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dofbot_interface::msg::WidthInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dofbot_interface::msg::WidthInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::msg::WidthInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::msg::WidthInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::msg::WidthInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::msg::WidthInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dofbot_interface::msg::WidthInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dofbot_interface::msg::WidthInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dofbot_interface__msg__WidthInfo
    std::shared_ptr<dofbot_interface::msg::WidthInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dofbot_interface__msg__WidthInfo
    std::shared_ptr<dofbot_interface::msg::WidthInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WidthInfo_ & other) const
  {
    if (this->l_x != other.l_x) {
      return false;
    }
    if (this->l_y != other.l_y) {
      return false;
    }
    if (this->r_x != other.r_x) {
      return false;
    }
    if (this->r_y != other.r_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const WidthInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WidthInfo_

// alias to use template instance with default allocator
using WidthInfo =
  dofbot_interface::msg::WidthInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__WIDTH_INFO__STRUCT_HPP_
