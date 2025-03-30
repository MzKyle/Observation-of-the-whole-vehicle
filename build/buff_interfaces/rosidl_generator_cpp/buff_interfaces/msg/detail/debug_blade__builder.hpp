// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from buff_interfaces:msg/DebugBlade.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE__BUILDER_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "buff_interfaces/msg/detail/debug_blade__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace buff_interfaces
{

namespace msg
{

namespace builder
{

class Init_DebugBlade_kpt
{
public:
  explicit Init_DebugBlade_kpt(::buff_interfaces::msg::DebugBlade & msg)
  : msg_(msg)
  {}
  ::buff_interfaces::msg::DebugBlade kpt(::buff_interfaces::msg::DebugBlade::_kpt_type arg)
  {
    msg_.kpt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::buff_interfaces::msg::DebugBlade msg_;
};

class Init_DebugBlade_center
{
public:
  explicit Init_DebugBlade_center(::buff_interfaces::msg::DebugBlade & msg)
  : msg_(msg)
  {}
  Init_DebugBlade_kpt center(::buff_interfaces::msg::DebugBlade::_center_type arg)
  {
    msg_.center = std::move(arg);
    return Init_DebugBlade_kpt(msg_);
  }

private:
  ::buff_interfaces::msg::DebugBlade msg_;
};

class Init_DebugBlade_pose
{
public:
  explicit Init_DebugBlade_pose(::buff_interfaces::msg::DebugBlade & msg)
  : msg_(msg)
  {}
  Init_DebugBlade_center pose(::buff_interfaces::msg::DebugBlade::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_DebugBlade_center(msg_);
  }

private:
  ::buff_interfaces::msg::DebugBlade msg_;
};

class Init_DebugBlade_prob
{
public:
  explicit Init_DebugBlade_prob(::buff_interfaces::msg::DebugBlade & msg)
  : msg_(msg)
  {}
  Init_DebugBlade_pose prob(::buff_interfaces::msg::DebugBlade::_prob_type arg)
  {
    msg_.prob = std::move(arg);
    return Init_DebugBlade_pose(msg_);
  }

private:
  ::buff_interfaces::msg::DebugBlade msg_;
};

class Init_DebugBlade_label
{
public:
  explicit Init_DebugBlade_label(::buff_interfaces::msg::DebugBlade & msg)
  : msg_(msg)
  {}
  Init_DebugBlade_prob label(::buff_interfaces::msg::DebugBlade::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_DebugBlade_prob(msg_);
  }

private:
  ::buff_interfaces::msg::DebugBlade msg_;
};

class Init_DebugBlade_height
{
public:
  explicit Init_DebugBlade_height(::buff_interfaces::msg::DebugBlade & msg)
  : msg_(msg)
  {}
  Init_DebugBlade_label height(::buff_interfaces::msg::DebugBlade::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_DebugBlade_label(msg_);
  }

private:
  ::buff_interfaces::msg::DebugBlade msg_;
};

class Init_DebugBlade_width
{
public:
  explicit Init_DebugBlade_width(::buff_interfaces::msg::DebugBlade & msg)
  : msg_(msg)
  {}
  Init_DebugBlade_height width(::buff_interfaces::msg::DebugBlade::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_DebugBlade_height(msg_);
  }

private:
  ::buff_interfaces::msg::DebugBlade msg_;
};

class Init_DebugBlade_y
{
public:
  explicit Init_DebugBlade_y(::buff_interfaces::msg::DebugBlade & msg)
  : msg_(msg)
  {}
  Init_DebugBlade_width y(::buff_interfaces::msg::DebugBlade::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_DebugBlade_width(msg_);
  }

private:
  ::buff_interfaces::msg::DebugBlade msg_;
};

class Init_DebugBlade_x
{
public:
  Init_DebugBlade_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DebugBlade_y x(::buff_interfaces::msg::DebugBlade::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_DebugBlade_y(msg_);
  }

private:
  ::buff_interfaces::msg::DebugBlade msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::buff_interfaces::msg::DebugBlade>()
{
  return buff_interfaces::msg::builder::Init_DebugBlade_x();
}

}  // namespace buff_interfaces

#endif  // BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE__BUILDER_HPP_
