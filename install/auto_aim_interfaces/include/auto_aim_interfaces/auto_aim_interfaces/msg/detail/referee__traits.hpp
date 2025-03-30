// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from auto_aim_interfaces:msg/Referee.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__TRAITS_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "auto_aim_interfaces/msg/detail/referee__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace auto_aim_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Referee & msg,
  std::ostream & out)
{
  out << "{";
  // member: event_data
  {
    out << "event_data: ";
    rosidl_generator_traits::value_to_yaml(msg.event_data, out);
    out << ", ";
  }

  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << ", ";
  }

  // member: rfid
  {
    out << "rfid: ";
    rosidl_generator_traits::value_to_yaml(msg.rfid, out);
    out << ", ";
  }

  // member: base_hp
  {
    out << "base_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.base_hp, out);
    out << ", ";
  }

  // member: sentry_hp
  {
    out << "sentry_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.sentry_hp, out);
    out << ", ";
  }

  // member: outpost_hp
  {
    out << "outpost_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.outpost_hp, out);
    out << ", ";
  }

  // member: projectile_allowance_17mm
  {
    out << "projectile_allowance_17mm: ";
    rosidl_generator_traits::value_to_yaml(msg.projectile_allowance_17mm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Referee & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: event_data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "event_data: ";
    rosidl_generator_traits::value_to_yaml(msg.event_data, out);
    out << "\n";
  }

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << "\n";
  }

  // member: rfid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rfid: ";
    rosidl_generator_traits::value_to_yaml(msg.rfid, out);
    out << "\n";
  }

  // member: base_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.base_hp, out);
    out << "\n";
  }

  // member: sentry_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sentry_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.sentry_hp, out);
    out << "\n";
  }

  // member: outpost_hp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "outpost_hp: ";
    rosidl_generator_traits::value_to_yaml(msg.outpost_hp, out);
    out << "\n";
  }

  // member: projectile_allowance_17mm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "projectile_allowance_17mm: ";
    rosidl_generator_traits::value_to_yaml(msg.projectile_allowance_17mm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Referee & msg, bool use_flow_style = false)
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

}  // namespace auto_aim_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use auto_aim_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const auto_aim_interfaces::msg::Referee & msg,
  std::ostream & out, size_t indentation = 0)
{
  auto_aim_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use auto_aim_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const auto_aim_interfaces::msg::Referee & msg)
{
  return auto_aim_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<auto_aim_interfaces::msg::Referee>()
{
  return "auto_aim_interfaces::msg::Referee";
}

template<>
inline const char * name<auto_aim_interfaces::msg::Referee>()
{
  return "auto_aim_interfaces/msg/Referee";
}

template<>
struct has_fixed_size<auto_aim_interfaces::msg::Referee>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<auto_aim_interfaces::msg::Referee>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<auto_aim_interfaces::msg::Referee>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__TRAITS_HPP_
