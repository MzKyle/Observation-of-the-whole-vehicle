// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Send.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SEND__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SEND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/send__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Send_yaw
{
public:
  explicit Init_Send_yaw(::auto_aim_interfaces::msg::Send & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Send yaw(::auto_aim_interfaces::msg::Send::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Send msg_;
};

class Init_Send_pitch
{
public:
  explicit Init_Send_pitch(::auto_aim_interfaces::msg::Send & msg)
  : msg_(msg)
  {}
  Init_Send_yaw pitch(::auto_aim_interfaces::msg::Send::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_Send_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Send msg_;
};

class Init_Send_v_yaw
{
public:
  explicit Init_Send_v_yaw(::auto_aim_interfaces::msg::Send & msg)
  : msg_(msg)
  {}
  Init_Send_pitch v_yaw(::auto_aim_interfaces::msg::Send::_v_yaw_type arg)
  {
    msg_.v_yaw = std::move(arg);
    return Init_Send_pitch(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Send msg_;
};

class Init_Send_position
{
public:
  explicit Init_Send_position(::auto_aim_interfaces::msg::Send & msg)
  : msg_(msg)
  {}
  Init_Send_v_yaw position(::auto_aim_interfaces::msg::Send::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Send_v_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Send msg_;
};

class Init_Send_is_fire
{
public:
  explicit Init_Send_is_fire(::auto_aim_interfaces::msg::Send & msg)
  : msg_(msg)
  {}
  Init_Send_position is_fire(::auto_aim_interfaces::msg::Send::_is_fire_type arg)
  {
    msg_.is_fire = std::move(arg);
    return Init_Send_position(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Send msg_;
};

class Init_Send_header
{
public:
  Init_Send_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Send_is_fire header(::auto_aim_interfaces::msg::Send::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Send_is_fire(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Send msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Send>()
{
  return auto_aim_interfaces::msg::builder::Init_Send_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SEND__BUILDER_HPP_
