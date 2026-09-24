// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dofbot_interface:msg/PosInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__POS_INFO__STRUCT_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__POS_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dofbot_interface__msg__PosInfo __attribute__((deprecated))
#else
# define DEPRECATED__dofbot_interface__msg__PosInfo __declspec(deprecated)
#endif

namespace dofbot_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PosInfo_
{
  using Type = PosInfo_<ContainerAllocator>;

  explicit PosInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->pos1 = 0.0;
      this->pos2 = 0.0;
      this->pos3 = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
    }
  }

  explicit PosInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->pos1 = 0.0;
      this->pos2 = 0.0;
      this->pos3 = 0.0;
      this->roll = 0.0;
      this->pitch = 0.0;
      this->yaw = 0.0;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _pos1_type =
    double;
  _pos1_type pos1;
  using _pos2_type =
    double;
  _pos2_type pos2;
  using _pos3_type =
    double;
  _pos3_type pos3;
  using _roll_type =
    double;
  _roll_type roll;
  using _pitch_type =
    double;
  _pitch_type pitch;
  using _yaw_type =
    double;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__pos1(
    const double & _arg)
  {
    this->pos1 = _arg;
    return *this;
  }
  Type & set__pos2(
    const double & _arg)
  {
    this->pos2 = _arg;
    return *this;
  }
  Type & set__pos3(
    const double & _arg)
  {
    this->pos3 = _arg;
    return *this;
  }
  Type & set__roll(
    const double & _arg)
  {
    this->roll = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dofbot_interface::msg::PosInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const dofbot_interface::msg::PosInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dofbot_interface::msg::PosInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dofbot_interface::msg::PosInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::msg::PosInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::msg::PosInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::msg::PosInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::msg::PosInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dofbot_interface::msg::PosInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dofbot_interface::msg::PosInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dofbot_interface__msg__PosInfo
    std::shared_ptr<dofbot_interface::msg::PosInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dofbot_interface__msg__PosInfo
    std::shared_ptr<dofbot_interface::msg::PosInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PosInfo_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->pos1 != other.pos1) {
      return false;
    }
    if (this->pos2 != other.pos2) {
      return false;
    }
    if (this->pos3 != other.pos3) {
      return false;
    }
    if (this->roll != other.roll) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const PosInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PosInfo_

// alias to use template instance with default allocator
using PosInfo =
  dofbot_interface::msg::PosInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__POS_INFO__STRUCT_HPP_
