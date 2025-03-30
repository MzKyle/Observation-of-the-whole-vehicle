// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from buff_interfaces:msg/DebugBladeArray.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE_ARRAY__TRAITS_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "buff_interfaces/msg/detail/debug_blade_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'blades'
#include "buff_interfaces/msg/detail/debug_blade__traits.hpp"

namespace buff_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DebugBladeArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: blades
  {
    if (msg.blades.size() == 0) {
      out << "blades: []";
    } else {
      out << "blades: [";
      size_t pending_items = msg.blades.size();
      for (auto item : msg.blades) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DebugBladeArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: blades
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.blades.size() == 0) {
      out << "blades: []\n";
    } else {
      out << "blades:\n";
      for (auto item : msg.blades) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DebugBladeArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace buff_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use buff_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const buff_interfaces::msg::DebugBladeArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  buff_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use buff_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const buff_interfaces::msg::DebugBladeArray & msg)
{
  return buff_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<buff_interfaces::msg::DebugBladeArray>()
{
  return "buff_interfaces::msg::DebugBladeArray";
}

template<>
inline const char * name<buff_interfaces::msg::DebugBladeArray>()
{
  return "buff_interfaces/msg/DebugBladeArray";
}

template<>
struct has_fixed_size<buff_interfaces::msg::DebugBladeArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<buff_interfaces::msg::DebugBladeArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<buff_interfaces::msg::DebugBladeArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE_ARRAY__TRAITS_HPP_
