// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/TrackerInfo.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__TrackerInfo __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__TrackerInfo __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackerInfo_
{
  using Type = TrackerInfo_<ContainerAllocator>;

  explicit TrackerInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position_diff = 0.0;
      this->yaw_diff = 0.0;
      this->yaw = 0.0;
    }
  }

  explicit TrackerInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position_diff = 0.0;
      this->yaw_diff = 0.0;
      this->yaw = 0.0;
    }
  }

  // field types and members
  using _position_diff_type =
    double;
  _position_diff_type position_diff;
  using _yaw_diff_type =
    double;
  _yaw_diff_type yaw_diff;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _yaw_type =
    double;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__position_diff(
    const double & _arg)
  {
    this->position_diff = _arg;
    return *this;
  }
  Type & set__yaw_diff(
    const double & _arg)
  {
    this->yaw_diff = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
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
    auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__TrackerInfo
    std::shared_ptr<auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__TrackerInfo
    std::shared_ptr<auto_aim_interfaces::msg::TrackerInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackerInfo_ & other) const
  {
    if (this->position_diff != other.position_diff) {
      return false;
    }
    if (this->yaw_diff != other.yaw_diff) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackerInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackerInfo_

// alias to use template instance with default allocator
using TrackerInfo =
  auto_aim_interfaces::msg::TrackerInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__STRUCT_HPP_
