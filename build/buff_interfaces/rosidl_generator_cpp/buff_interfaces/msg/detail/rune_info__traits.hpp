// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from buff_interfaces:msg/RuneInfo.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__TRAITS_HPP_
#define BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "buff_interfaces/msg/detail/rune_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'blade'
// Member 'center'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace buff_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const RuneInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: blade
  {
    out << "blade: ";
    to_flow_style_yaml(msg.blade, out);
    out << ", ";
  }

  // member: center
  {
    out << "center: ";
    to_flow_style_yaml(msg.center, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << ", ";
  }

  // member: predicted_speed
  {
    out << "predicted_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.predicted_speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RuneInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: blade
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "blade:\n";
    to_block_style_yaml(msg.blade, out, indentation + 2);
  }

  // member: center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center:\n";
    to_block_style_yaml(msg.center, out, indentation + 2);
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }

  // member: predicted_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "predicted_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.predicted_speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RuneInfo & msg, bool use_flow_style = false)
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
  const buff_interfaces::msg::RuneInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  buff_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use buff_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const buff_interfaces::msg::RuneInfo & msg)
{
  return buff_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<buff_interfaces::msg::RuneInfo>()
{
  return "buff_interfaces::msg::RuneInfo";
}

template<>
inline const char * name<buff_interfaces::msg::RuneInfo>()
{
  return "buff_interfaces/msg/RuneInfo";
}

template<>
struct has_fixed_size<buff_interfaces::msg::RuneInfo>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<buff_interfaces::msg::RuneInfo>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<buff_interfaces::msg::RuneInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__TRAITS_HPP_
