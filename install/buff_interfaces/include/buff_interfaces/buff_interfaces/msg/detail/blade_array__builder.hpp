// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from buff_interfaces:msg/BladeArray.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__BLADE_ARRAY__BUILDER_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__BLADE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "buff_interfaces/msg/detail/blade_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace buff_interfaces
{

namespace msg
{

namespace builder
{

class Init_BladeArray_blades
{
public:
  explicit Init_BladeArray_blades(::buff_interfaces::msg::BladeArray & msg)
  : msg_(msg)
  {}
  ::buff_interfaces::msg::BladeArray blades(::buff_interfaces::msg::BladeArray::_blades_type arg)
  {
    msg_.blades = std::move(arg);
    return std::move(msg_);
  }

private:
  ::buff_interfaces::msg::BladeArray msg_;
};

class Init_BladeArray_header
{
public:
  Init_BladeArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BladeArray_blades header(::buff_interfaces::msg::BladeArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BladeArray_blades(msg_);
  }

private:
  ::buff_interfaces::msg::BladeArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::buff_interfaces::msg::BladeArray>()
{
  return buff_interfaces::msg::builder::Init_BladeArray_header();
}

}  // namespace buff_interfaces

#endif  // BUFF_INTERFACES__MSG__DETAIL__BLADE_ARRAY__BUILDER_HPP_
