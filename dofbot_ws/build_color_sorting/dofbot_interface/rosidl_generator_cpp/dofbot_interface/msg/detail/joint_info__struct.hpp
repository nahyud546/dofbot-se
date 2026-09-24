// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dofbot_interface:msg/JointInfo.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__STRUCT_HPP_
#define DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dofbot_interface__msg__JointInfo __attribute__((deprecated))
#else
# define DEPRECATED__dofbot_interface__msg__JointInfo __declspec(deprecated)
#endif

namespace dofbot_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointInfo_
{
  using Type = JointInfo_<ContainerAllocator>;

  explicit JointInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->joint1 = 0.0;
      this->joint2 = 0.0;
      this->joint3 = 0.0;
      this->joint4 = 0.0;
      this->joint5 = 0.0;
      this->joint6 = 0.0;
    }
  }

  explicit JointInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->joint1 = 0.0;
      this->joint2 = 0.0;
      this->joint3 = 0.0;
      this->joint4 = 0.0;
      this->joint5 = 0.0;
      this->joint6 = 0.0;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _joint1_type =
    double;
  _joint1_type joint1;
  using _joint2_type =
    double;
  _joint2_type joint2;
  using _joint3_type =
    double;
  _joint3_type joint3;
  using _joint4_type =
    double;
  _joint4_type joint4;
  using _joint5_type =
    double;
  _joint5_type joint5;
  using _joint6_type =
    double;
  _joint6_type joint6;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__joint1(
    const double & _arg)
  {
    this->joint1 = _arg;
    return *this;
  }
  Type & set__joint2(
    const double & _arg)
  {
    this->joint2 = _arg;
    return *this;
  }
  Type & set__joint3(
    const double & _arg)
  {
    this->joint3 = _arg;
    return *this;
  }
  Type & set__joint4(
    const double & _arg)
  {
    this->joint4 = _arg;
    return *this;
  }
  Type & set__joint5(
    const double & _arg)
  {
    this->joint5 = _arg;
    return *this;
  }
  Type & set__joint6(
    const double & _arg)
  {
    this->joint6 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dofbot_interface::msg::JointInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const dofbot_interface::msg::JointInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dofbot_interface::msg::JointInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dofbot_interface::msg::JointInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::msg::JointInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::msg::JointInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::msg::JointInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::msg::JointInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dofbot_interface::msg::JointInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dofbot_interface::msg::JointInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dofbot_interface__msg__JointInfo
    std::shared_ptr<dofbot_interface::msg::JointInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dofbot_interface__msg__JointInfo
    std::shared_ptr<dofbot_interface::msg::JointInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointInfo_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->joint1 != other.joint1) {
      return false;
    }
    if (this->joint2 != other.joint2) {
      return false;
    }
    if (this->joint3 != other.joint3) {
      return false;
    }
    if (this->joint4 != other.joint4) {
      return false;
    }
    if (this->joint5 != other.joint5) {
      return false;
    }
    if (this->joint6 != other.joint6) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointInfo_

// alias to use template instance with default allocator
using JointInfo =
  dofbot_interface::msg::JointInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__MSG__DETAIL__JOINT_INFO__STRUCT_HPP_
