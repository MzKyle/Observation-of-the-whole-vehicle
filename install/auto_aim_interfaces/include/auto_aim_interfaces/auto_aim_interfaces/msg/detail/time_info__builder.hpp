// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/TimeInfo.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TIME_INFO__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TIME_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/time_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_TimeInfo_time
{
public:
  explicit Init_TimeInfo_time(::auto_aim_interfaces::msg::TimeInfo & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::TimeInfo time(::auto_aim_interfaces::msg::TimeInfo::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TimeInfo msg_;
};

class Init_TimeInfo_header
{
public:
  Init_TimeInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimeInfo_time header(::auto_aim_interfaces::msg::TimeInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TimeInfo_time(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TimeInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::TimeInfo>()
{
  return auto_aim_interfaces::msg::builder::Init_TimeInfo_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TIME_INFO__BUILDER_HPP_
