// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from buff_interfaces:msg/RuneInfo.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__BUILDER_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "buff_interfaces/msg/detail/rune_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace buff_interfaces
{

namespace msg
{

namespace builder
{

class Init_RuneInfo_predicted_speed
{
public:
  explicit Init_RuneInfo_predicted_speed(::buff_interfaces::msg::RuneInfo & msg)
  : msg_(msg)
  {}
  ::buff_interfaces::msg::RuneInfo predicted_speed(::buff_interfaces::msg::RuneInfo::_predicted_speed_type arg)
  {
    msg_.predicted_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::buff_interfaces::msg::RuneInfo msg_;
};

class Init_RuneInfo_speed
{
public:
  explicit Init_RuneInfo_speed(::buff_interfaces::msg::RuneInfo & msg)
  : msg_(msg)
  {}
  Init_RuneInfo_predicted_speed speed(::buff_interfaces::msg::RuneInfo::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_RuneInfo_predicted_speed(msg_);
  }

private:
  ::buff_interfaces::msg::RuneInfo msg_;
};

class Init_RuneInfo_center
{
public:
  explicit Init_RuneInfo_center(::buff_interfaces::msg::RuneInfo & msg)
  : msg_(msg)
  {}
  Init_RuneInfo_speed center(::buff_interfaces::msg::RuneInfo::_center_type arg)
  {
    msg_.center = std::move(arg);
    return Init_RuneInfo_speed(msg_);
  }

private:
  ::buff_interfaces::msg::RuneInfo msg_;
};

class Init_RuneInfo_blade
{
public:
  explicit Init_RuneInfo_blade(::buff_interfaces::msg::RuneInfo & msg)
  : msg_(msg)
  {}
  Init_RuneInfo_center blade(::buff_interfaces::msg::RuneInfo::_blade_type arg)
  {
    msg_.blade = std::move(arg);
    return Init_RuneInfo_center(msg_);
  }

private:
  ::buff_interfaces::msg::RuneInfo msg_;
};

class Init_RuneInfo_header
{
public:
  Init_RuneInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RuneInfo_blade header(::buff_interfaces::msg::RuneInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RuneInfo_blade(msg_);
  }

private:
  ::buff_interfaces::msg::RuneInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::buff_interfaces::msg::RuneInfo>()
{
  return buff_interfaces::msg::builder::Init_RuneInfo_header();
}

}  // namespace buff_interfaces

#endif  // BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__BUILDER_HPP_
