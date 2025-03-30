// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from buff_interfaces:msg/DebugBladeArray.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE_ARRAY__BUILDER_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "buff_interfaces/msg/detail/debug_blade_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace buff_interfaces
{

namespace msg
{

namespace builder
{

class Init_DebugBladeArray_blades
{
public:
  Init_DebugBladeArray_blades()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::buff_interfaces::msg::DebugBladeArray blades(::buff_interfaces::msg::DebugBladeArray::_blades_type arg)
  {
    msg_.blades = std::move(arg);
    return std::move(msg_);
  }

private:
  ::buff_interfaces::msg::DebugBladeArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::buff_interfaces::msg::DebugBladeArray>()
{
  return buff_interfaces::msg::builder::Init_DebugBladeArray_blades();
}

}  // namespace buff_interfaces

#endif  // BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE_ARRAY__BUILDER_HPP_
