// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dofbot_interface:srv/CurJoint.idl
// generated code does not contain a copyright notice

#ifndef DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__STRUCT_HPP_
#define DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dofbot_interface__srv__CurJoint_Request __attribute__((deprecated))
#else
# define DEPRECATED__dofbot_interface__srv__CurJoint_Request __declspec(deprecated)
#endif

namespace dofbot_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CurJoint_Request_
{
  using Type = CurJoint_Request_<ContainerAllocator>;

  explicit CurJoint_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->srv_joints = "";
    }
  }

  explicit CurJoint_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : srv_joints(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->srv_joints = "";
    }
  }

  // field types and members
  using _srv_joints_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _srv_joints_type srv_joints;

  // setters for named parameter idiom
  Type & set__srv_joints(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->srv_joints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dofbot_interface::srv::CurJoint_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dofbot_interface::srv::CurJoint_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dofbot_interface::srv::CurJoint_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dofbot_interface::srv::CurJoint_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::srv::CurJoint_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::srv::CurJoint_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::srv::CurJoint_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::srv::CurJoint_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dofbot_interface::srv::CurJoint_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dofbot_interface::srv::CurJoint_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dofbot_interface__srv__CurJoint_Request
    std::shared_ptr<dofbot_interface::srv::CurJoint_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dofbot_interface__srv__CurJoint_Request
    std::shared_ptr<dofbot_interface::srv::CurJoint_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CurJoint_Request_ & other) const
  {
    if (this->srv_joints != other.srv_joints) {
      return false;
    }
    return true;
  }
  bool operator!=(const CurJoint_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CurJoint_Request_

// alias to use template instance with default allocator
using CurJoint_Request =
  dofbot_interface::srv::CurJoint_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dofbot_interface


#ifndef _WIN32
# define DEPRECATED__dofbot_interface__srv__CurJoint_Response __attribute__((deprecated))
#else
# define DEPRECATED__dofbot_interface__srv__CurJoint_Response __declspec(deprecated)
#endif

namespace dofbot_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CurJoint_Response_
{
  using Type = CurJoint_Response_<ContainerAllocator>;

  explicit CurJoint_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->srv_joint1 = 0.0;
      this->srv_joint2 = 0.0;
      this->srv_joint3 = 0.0;
      this->srv_joint4 = 0.0;
      this->srv_joint5 = 0.0;
      this->srv_joint6 = 0.0;
    }
  }

  explicit CurJoint_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->srv_joint1 = 0.0;
      this->srv_joint2 = 0.0;
      this->srv_joint3 = 0.0;
      this->srv_joint4 = 0.0;
      this->srv_joint5 = 0.0;
      this->srv_joint6 = 0.0;
    }
  }

  // field types and members
  using _srv_joint1_type =
    double;
  _srv_joint1_type srv_joint1;
  using _srv_joint2_type =
    double;
  _srv_joint2_type srv_joint2;
  using _srv_joint3_type =
    double;
  _srv_joint3_type srv_joint3;
  using _srv_joint4_type =
    double;
  _srv_joint4_type srv_joint4;
  using _srv_joint5_type =
    double;
  _srv_joint5_type srv_joint5;
  using _srv_joint6_type =
    double;
  _srv_joint6_type srv_joint6;

  // setters for named parameter idiom
  Type & set__srv_joint1(
    const double & _arg)
  {
    this->srv_joint1 = _arg;
    return *this;
  }
  Type & set__srv_joint2(
    const double & _arg)
  {
    this->srv_joint2 = _arg;
    return *this;
  }
  Type & set__srv_joint3(
    const double & _arg)
  {
    this->srv_joint3 = _arg;
    return *this;
  }
  Type & set__srv_joint4(
    const double & _arg)
  {
    this->srv_joint4 = _arg;
    return *this;
  }
  Type & set__srv_joint5(
    const double & _arg)
  {
    this->srv_joint5 = _arg;
    return *this;
  }
  Type & set__srv_joint6(
    const double & _arg)
  {
    this->srv_joint6 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dofbot_interface::srv::CurJoint_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dofbot_interface::srv::CurJoint_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dofbot_interface::srv::CurJoint_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dofbot_interface::srv::CurJoint_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::srv::CurJoint_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::srv::CurJoint_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dofbot_interface::srv::CurJoint_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dofbot_interface::srv::CurJoint_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dofbot_interface::srv::CurJoint_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dofbot_interface::srv::CurJoint_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dofbot_interface__srv__CurJoint_Response
    std::shared_ptr<dofbot_interface::srv::CurJoint_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dofbot_interface__srv__CurJoint_Response
    std::shared_ptr<dofbot_interface::srv::CurJoint_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CurJoint_Response_ & other) const
  {
    if (this->srv_joint1 != other.srv_joint1) {
      return false;
    }
    if (this->srv_joint2 != other.srv_joint2) {
      return false;
    }
    if (this->srv_joint3 != other.srv_joint3) {
      return false;
    }
    if (this->srv_joint4 != other.srv_joint4) {
      return false;
    }
    if (this->srv_joint5 != other.srv_joint5) {
      return false;
    }
    if (this->srv_joint6 != other.srv_joint6) {
      return false;
    }
    return true;
  }
  bool operator!=(const CurJoint_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CurJoint_Response_

// alias to use template instance with default allocator
using CurJoint_Response =
  dofbot_interface::srv::CurJoint_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dofbot_interface

namespace dofbot_interface
{

namespace srv
{

struct CurJoint
{
  using Request = dofbot_interface::srv::CurJoint_Request;
  using Response = dofbot_interface::srv::CurJoint_Response;
};

}  // namespace srv

}  // namespace dofbot_interface

#endif  // DOFBOT_INTERFACE__SRV__DETAIL__CUR_JOINT__STRUCT_HPP_
