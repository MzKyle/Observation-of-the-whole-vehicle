// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from buff_interfaces:msg/Rune.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__RUNE__STRUCT_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__RUNE__STRUCT_HPP_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__buff_interfaces__msg__Rune __attribute__((deprecated))
#else
# define DEPRECATED__buff_interfaces__msg__Rune __declspec(deprecated)
#endif

namespace buff_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Rune_
{
  using Type = Rune_<ContainerAllocator>;

  explicit Rune_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    velocity(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tracking = false;
      this->r = 0.0;
      this->theta = 0.0;
      this->a = 0.0;
      this->w = 0.0;
      this->b = 0.0;
      this->c = 0.0;
      this->offset_id = 0l;
      this->t_offset = 0ll;
    }
  }

  explicit Rune_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->tracking = false;
      this->r = 0.0;
      this->theta = 0.0;
      this->a = 0.0;
      this->w = 0.0;
      this->b = 0.0;
      this->c = 0.0;
      this->offset_id = 0l;
      this->t_offset = 0ll;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _tracking_type =
    bool;
  _tracking_type tracking;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _r_type =
    double;
  _r_type r;
  using _theta_type =
    double;
  _theta_type theta;
  using _a_type =
    double;
  _a_type a;
  using _w_type =
    double;
  _w_type w;
  using _b_type =
    double;
  _b_type b;
  using _c_type =
    double;
  _c_type c;
  using _offset_id_type =
    int32_t;
  _offset_id_type offset_id;
  using _t_offset_type =
    int64_t;
  _t_offset_type t_offset;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__tracking(
    const bool & _arg)
  {
    this->tracking = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__r(
    const double & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__theta(
    const double & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__a(
    const double & _arg)
  {
    this->a = _arg;
    return *this;
  }
  Type & set__w(
    const double & _arg)
  {
    this->w = _arg;
    return *this;
  }
  Type & set__b(
    const double & _arg)
  {
    this->b = _arg;
    return *this;
  }
  Type & set__c(
    const double & _arg)
  {
    this->c = _arg;
    return *this;
  }
  Type & set__offset_id(
    const int32_t & _arg)
  {
    this->offset_id = _arg;
    return *this;
  }
  Type & set__t_offset(
    const int64_t & _arg)
  {
    this->t_offset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    buff_interfaces::msg::Rune_<ContainerAllocator> *;
  using ConstRawPtr =
    const buff_interfaces::msg::Rune_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<buff_interfaces::msg::Rune_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<buff_interfaces::msg::Rune_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      buff_interfaces::msg::Rune_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<buff_interfaces::msg::Rune_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      buff_interfaces::msg::Rune_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<buff_interfaces::msg::Rune_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<buff_interfaces::msg::Rune_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<buff_interfaces::msg::Rune_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__buff_interfaces__msg__Rune
    std::shared_ptr<buff_interfaces::msg::Rune_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__buff_interfaces__msg__Rune
    std::shared_ptr<buff_interfaces::msg::Rune_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Rune_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->tracking != other.tracking) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    if (this->a != other.a) {
      return false;
    }
    if (this->w != other.w) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    if (this->c != other.c) {
      return false;
    }
    if (this->offset_id != other.offset_id) {
      return false;
    }
    if (this->t_offset != other.t_offset) {
      return false;
    }
    return true;
  }
  bool operator!=(const Rune_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Rune_

// alias to use template instance with default allocator
using Rune =
  buff_interfaces::msg::Rune_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace buff_interfaces

#endif  // BUFF_INTERFACES__MSG__DETAIL__RUNE__STRUCT_HPP_
