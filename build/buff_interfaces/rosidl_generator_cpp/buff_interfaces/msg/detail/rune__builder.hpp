// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from buff_interfaces:msg/Rune.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__RUNE__BUILDER_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__RUNE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "buff_interfaces/msg/detail/rune__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace buff_interfaces
{

namespace msg
{

namespace builder
{

class Init_Rune_t_offset
{
public:
  explicit Init_Rune_t_offset(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  ::buff_interfaces::msg::Rune t_offset(::buff_interfaces::msg::Rune::_t_offset_type arg)
  {
    msg_.t_offset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_offset_id
{
public:
  explicit Init_Rune_offset_id(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_t_offset offset_id(::buff_interfaces::msg::Rune::_offset_id_type arg)
  {
    msg_.offset_id = std::move(arg);
    return Init_Rune_t_offset(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_c
{
public:
  explicit Init_Rune_c(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_offset_id c(::buff_interfaces::msg::Rune::_c_type arg)
  {
    msg_.c = std::move(arg);
    return Init_Rune_offset_id(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_b
{
public:
  explicit Init_Rune_b(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_c b(::buff_interfaces::msg::Rune::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_Rune_c(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_w
{
public:
  explicit Init_Rune_w(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_b w(::buff_interfaces::msg::Rune::_w_type arg)
  {
    msg_.w = std::move(arg);
    return Init_Rune_b(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_a
{
public:
  explicit Init_Rune_a(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_w a(::buff_interfaces::msg::Rune::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_Rune_w(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_theta
{
public:
  explicit Init_Rune_theta(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_a theta(::buff_interfaces::msg::Rune::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_Rune_a(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_r
{
public:
  explicit Init_Rune_r(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_theta r(::buff_interfaces::msg::Rune::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_Rune_theta(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_velocity
{
public:
  explicit Init_Rune_velocity(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_r velocity(::buff_interfaces::msg::Rune::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Rune_r(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_position
{
public:
  explicit Init_Rune_position(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_velocity position(::buff_interfaces::msg::Rune::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Rune_velocity(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_tracking
{
public:
  explicit Init_Rune_tracking(::buff_interfaces::msg::Rune & msg)
  : msg_(msg)
  {}
  Init_Rune_position tracking(::buff_interfaces::msg::Rune::_tracking_type arg)
  {
    msg_.tracking = std::move(arg);
    return Init_Rune_position(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

class Init_Rune_header
{
public:
  Init_Rune_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Rune_tracking header(::buff_interfaces::msg::Rune::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Rune_tracking(msg_);
  }

private:
  ::buff_interfaces::msg::Rune msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::buff_interfaces::msg::Rune>()
{
  return buff_interfaces::msg::builder::Init_Rune_header();
}

}  // namespace buff_interfaces

#endif  // BUFF_INTERFACES__MSG__DETAIL__RUNE__BUILDER_HPP_
