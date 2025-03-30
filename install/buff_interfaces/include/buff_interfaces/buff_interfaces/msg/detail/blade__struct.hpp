// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from buff_interfaces:msg/Blade.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__BLADE__STRUCT_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__BLADE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__buff_interfaces__msg__Blade __attribute__((deprecated))
#else
# define DEPRECATED__buff_interfaces__msg__Blade __declspec(deprecated)
#endif

namespace buff_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Blade_
{
  using Type = Blade_<ContainerAllocator>;

  explicit Blade_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = 0ll;
      this->prob = 0.0;
    }
  }

  explicit Blade_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = 0ll;
      this->prob = 0.0;
    }
  }

  // field types and members
  using _label_type =
    int64_t;
  _label_type label;
  using _prob_type =
    double;
  _prob_type prob;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__label(
    const int64_t & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__prob(
    const double & _arg)
  {
    this->prob = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    buff_interfaces::msg::Blade_<ContainerAllocator> *;
  using ConstRawPtr =
    const buff_interfaces::msg::Blade_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<buff_interfaces::msg::Blade_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<buff_interfaces::msg::Blade_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      buff_interfaces::msg::Blade_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<buff_interfaces::msg::Blade_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      buff_interfaces::msg::Blade_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<buff_interfaces::msg::Blade_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<buff_interfaces::msg::Blade_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<buff_interfaces::msg::Blade_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__buff_interfaces__msg__Blade
    std::shared_ptr<buff_interfaces::msg::Blade_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__buff_interfaces__msg__Blade
    std::shared_ptr<buff_interfaces::msg::Blade_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Blade_ & other) const
  {
    if (this->label != other.label) {
      return false;
    }
    if (this->prob != other.prob) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const Blade_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Blade_

// alias to use template instance with default allocator
using Blade =
  buff_interfaces::msg::Blade_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace buff_interfaces

#endif  // BUFF_INTERFACES__MSG__DETAIL__BLADE__STRUCT_HPP_
