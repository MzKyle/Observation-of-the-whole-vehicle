// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from buff_interfaces:msg/RuneInfo.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__STRUCT_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'blade'
// Member 'center'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__buff_interfaces__msg__RuneInfo __attribute__((deprecated))
#else
# define DEPRECATED__buff_interfaces__msg__RuneInfo __declspec(deprecated)
#endif

namespace buff_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RuneInfo_
{
  using Type = RuneInfo_<ContainerAllocator>;

  explicit RuneInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    blade(_init),
    center(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0;
      this->predicted_speed = 0.0;
    }
  }

  explicit RuneInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    blade(_alloc, _init),
    center(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0;
      this->predicted_speed = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _blade_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _blade_type blade;
  using _center_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _center_type center;
  using _speed_type =
    double;
  _speed_type speed;
  using _predicted_speed_type =
    double;
  _predicted_speed_type predicted_speed;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__blade(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->blade = _arg;
    return *this;
  }
  Type & set__center(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->center = _arg;
    return *this;
  }
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__predicted_speed(
    const double & _arg)
  {
    this->predicted_speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    buff_interfaces::msg::RuneInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const buff_interfaces::msg::RuneInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<buff_interfaces::msg::RuneInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<buff_interfaces::msg::RuneInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      buff_interfaces::msg::RuneInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<buff_interfaces::msg::RuneInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      buff_interfaces::msg::RuneInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<buff_interfaces::msg::RuneInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<buff_interfaces::msg::RuneInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<buff_interfaces::msg::RuneInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__buff_interfaces__msg__RuneInfo
    std::shared_ptr<buff_interfaces::msg::RuneInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__buff_interfaces__msg__RuneInfo
    std::shared_ptr<buff_interfaces::msg::RuneInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RuneInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->blade != other.blade) {
      return false;
    }
    if (this->center != other.center) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->predicted_speed != other.predicted_speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const RuneInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RuneInfo_

// alias to use template instance with default allocator
using RuneInfo =
  buff_interfaces::msg::RuneInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace buff_interfaces

#endif  // BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__STRUCT_HPP_
