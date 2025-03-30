// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/Referee.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__Referee __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__Referee __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Referee_
{
  using Type = Referee_<ContainerAllocator>;

  explicit Referee_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->event_data = 0ul;
      this->time = 0;
      this->rfid = 0ul;
      this->base_hp = 0;
      this->sentry_hp = 0;
      this->outpost_hp = 0;
      this->projectile_allowance_17mm = 0;
    }
  }

  explicit Referee_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->event_data = 0ul;
      this->time = 0;
      this->rfid = 0ul;
      this->base_hp = 0;
      this->sentry_hp = 0;
      this->outpost_hp = 0;
      this->projectile_allowance_17mm = 0;
    }
  }

  // field types and members
  using _event_data_type =
    uint32_t;
  _event_data_type event_data;
  using _time_type =
    uint16_t;
  _time_type time;
  using _rfid_type =
    uint32_t;
  _rfid_type rfid;
  using _base_hp_type =
    uint16_t;
  _base_hp_type base_hp;
  using _sentry_hp_type =
    uint16_t;
  _sentry_hp_type sentry_hp;
  using _outpost_hp_type =
    uint16_t;
  _outpost_hp_type outpost_hp;
  using _projectile_allowance_17mm_type =
    uint16_t;
  _projectile_allowance_17mm_type projectile_allowance_17mm;

  // setters for named parameter idiom
  Type & set__event_data(
    const uint32_t & _arg)
  {
    this->event_data = _arg;
    return *this;
  }
  Type & set__time(
    const uint16_t & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__rfid(
    const uint32_t & _arg)
  {
    this->rfid = _arg;
    return *this;
  }
  Type & set__base_hp(
    const uint16_t & _arg)
  {
    this->base_hp = _arg;
    return *this;
  }
  Type & set__sentry_hp(
    const uint16_t & _arg)
  {
    this->sentry_hp = _arg;
    return *this;
  }
  Type & set__outpost_hp(
    const uint16_t & _arg)
  {
    this->outpost_hp = _arg;
    return *this;
  }
  Type & set__projectile_allowance_17mm(
    const uint16_t & _arg)
  {
    this->projectile_allowance_17mm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::Referee_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::Referee_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Referee_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Referee_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Referee_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Referee_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Referee_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Referee_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Referee_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Referee_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__Referee
    std::shared_ptr<auto_aim_interfaces::msg::Referee_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__Referee
    std::shared_ptr<auto_aim_interfaces::msg::Referee_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Referee_ & other) const
  {
    if (this->event_data != other.event_data) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    if (this->rfid != other.rfid) {
      return false;
    }
    if (this->base_hp != other.base_hp) {
      return false;
    }
    if (this->sentry_hp != other.sentry_hp) {
      return false;
    }
    if (this->outpost_hp != other.outpost_hp) {
      return false;
    }
    if (this->projectile_allowance_17mm != other.projectile_allowance_17mm) {
      return false;
    }
    return true;
  }
  bool operator!=(const Referee_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Referee_

// alias to use template instance with default allocator
using Referee =
  auto_aim_interfaces::msg::Referee_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__STRUCT_HPP_
