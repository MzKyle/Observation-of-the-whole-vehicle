// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Referee.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/referee__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Referee_projectile_allowance_17mm
{
public:
  explicit Init_Referee_projectile_allowance_17mm(::auto_aim_interfaces::msg::Referee & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Referee projectile_allowance_17mm(::auto_aim_interfaces::msg::Referee::_projectile_allowance_17mm_type arg)
  {
    msg_.projectile_allowance_17mm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Referee msg_;
};

class Init_Referee_outpost_hp
{
public:
  explicit Init_Referee_outpost_hp(::auto_aim_interfaces::msg::Referee & msg)
  : msg_(msg)
  {}
  Init_Referee_projectile_allowance_17mm outpost_hp(::auto_aim_interfaces::msg::Referee::_outpost_hp_type arg)
  {
    msg_.outpost_hp = std::move(arg);
    return Init_Referee_projectile_allowance_17mm(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Referee msg_;
};

class Init_Referee_sentry_hp
{
public:
  explicit Init_Referee_sentry_hp(::auto_aim_interfaces::msg::Referee & msg)
  : msg_(msg)
  {}
  Init_Referee_outpost_hp sentry_hp(::auto_aim_interfaces::msg::Referee::_sentry_hp_type arg)
  {
    msg_.sentry_hp = std::move(arg);
    return Init_Referee_outpost_hp(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Referee msg_;
};

class Init_Referee_base_hp
{
public:
  explicit Init_Referee_base_hp(::auto_aim_interfaces::msg::Referee & msg)
  : msg_(msg)
  {}
  Init_Referee_sentry_hp base_hp(::auto_aim_interfaces::msg::Referee::_base_hp_type arg)
  {
    msg_.base_hp = std::move(arg);
    return Init_Referee_sentry_hp(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Referee msg_;
};

class Init_Referee_rfid
{
public:
  explicit Init_Referee_rfid(::auto_aim_interfaces::msg::Referee & msg)
  : msg_(msg)
  {}
  Init_Referee_base_hp rfid(::auto_aim_interfaces::msg::Referee::_rfid_type arg)
  {
    msg_.rfid = std::move(arg);
    return Init_Referee_base_hp(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Referee msg_;
};

class Init_Referee_time
{
public:
  explicit Init_Referee_time(::auto_aim_interfaces::msg::Referee & msg)
  : msg_(msg)
  {}
  Init_Referee_rfid time(::auto_aim_interfaces::msg::Referee::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_Referee_rfid(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Referee msg_;
};

class Init_Referee_event_data
{
public:
  Init_Referee_event_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Referee_time event_data(::auto_aim_interfaces::msg::Referee::_event_data_type arg)
  {
    msg_.event_data = std::move(arg);
    return Init_Referee_time(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Referee msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Referee>()
{
  return auto_aim_interfaces::msg::builder::Init_Referee_event_data();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__BUILDER_HPP_
