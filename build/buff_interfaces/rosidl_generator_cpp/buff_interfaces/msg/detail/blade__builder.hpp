// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from buff_interfaces:msg/Blade.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__BLADE__BUILDER_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__BLADE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "buff_interfaces/msg/detail/blade__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace buff_interfaces
{

namespace msg
{

namespace builder
{

class Init_Blade_pose
{
public:
  explicit Init_Blade_pose(::buff_interfaces::msg::Blade & msg)
  : msg_(msg)
  {}
  ::buff_interfaces::msg::Blade pose(::buff_interfaces::msg::Blade::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::buff_interfaces::msg::Blade msg_;
};

class Init_Blade_prob
{
public:
  explicit Init_Blade_prob(::buff_interfaces::msg::Blade & msg)
  : msg_(msg)
  {}
  Init_Blade_pose prob(::buff_interfaces::msg::Blade::_prob_type arg)
  {
    msg_.prob = std::move(arg);
    return Init_Blade_pose(msg_);
  }

private:
  ::buff_interfaces::msg::Blade msg_;
};

class Init_Blade_label
{
public:
  Init_Blade_label()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Blade_prob label(::buff_interfaces::msg::Blade::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_Blade_prob(msg_);
  }

private:
  ::buff_interfaces::msg::Blade msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::buff_interfaces::msg::Blade>()
{
  return buff_interfaces::msg::builder::Init_Blade_label();
}

}  // namespace buff_interfaces

#endif  // BUFF_INTERFACES__MSG__DETAIL__BLADE__BUILDER_HPP_
