// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from buff_interfaces:msg/TimeInfo.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__TIME_INFO__BUILDER_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__TIME_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "buff_interfaces/msg/detail/time_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace buff_interfaces
{

namespace msg
{

namespace builder
{

class Init_TimeInfo_time
{
public:
  explicit Init_TimeInfo_time(::buff_interfaces::msg::TimeInfo & msg)
  : msg_(msg)
  {}
  ::buff_interfaces::msg::TimeInfo time(::buff_interfaces::msg::TimeInfo::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::buff_interfaces::msg::TimeInfo msg_;
};

class Init_TimeInfo_header
{
public:
  Init_TimeInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TimeInfo_time header(::buff_interfaces::msg::TimeInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TimeInfo_time(msg_);
  }

private:
  ::buff_interfaces::msg::TimeInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::buff_interfaces::msg::TimeInfo>()
{
  return buff_interfaces::msg::builder::Init_TimeInfo_header();
}

}  // namespace buff_interfaces

#endif  // BUFF_INTERFACES__MSG__DETAIL__TIME_INFO__BUILDER_HPP_
