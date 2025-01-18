// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/Target.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Target_dz
{
public:
  explicit Init_Target_dz(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::Target dz(::auto_aim_interfaces::msg::Target::_dz_type arg)
  {
    msg_.dz = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_radius_2
{
public:
  explicit Init_Target_radius_2(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_dz radius_2(::auto_aim_interfaces::msg::Target::_radius_2_type arg)
  {
    msg_.radius_2 = std::move(arg);
    return Init_Target_dz(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_radius_1
{
public:
  explicit Init_Target_radius_1(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_radius_2 radius_1(::auto_aim_interfaces::msg::Target::_radius_1_type arg)
  {
    msg_.radius_1 = std::move(arg);
    return Init_Target_radius_2(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_v_yaw
{
public:
  explicit Init_Target_v_yaw(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_radius_1 v_yaw(::auto_aim_interfaces::msg::Target::_v_yaw_type arg)
  {
    msg_.v_yaw = std::move(arg);
    return Init_Target_radius_1(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_yaw
{
public:
  explicit Init_Target_yaw(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_v_yaw yaw(::auto_aim_interfaces::msg::Target::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_Target_v_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_velocity
{
public:
  explicit Init_Target_velocity(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_yaw velocity(::auto_aim_interfaces::msg::Target::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_Target_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_position
{
public:
  explicit Init_Target_position(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_velocity position(::auto_aim_interfaces::msg::Target::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Target_velocity(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_armors_num
{
public:
  explicit Init_Target_armors_num(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_position armors_num(::auto_aim_interfaces::msg::Target::_armors_num_type arg)
  {
    msg_.armors_num = std::move(arg);
    return Init_Target_position(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_id
{
public:
  explicit Init_Target_id(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_armors_num id(::auto_aim_interfaces::msg::Target::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Target_armors_num(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_tracking
{
public:
  explicit Init_Target_tracking(::auto_aim_interfaces::msg::Target & msg)
  : msg_(msg)
  {}
  Init_Target_id tracking(::auto_aim_interfaces::msg::Target::_tracking_type arg)
  {
    msg_.tracking = std::move(arg);
    return Init_Target_id(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

class Init_Target_header
{
public:
  Init_Target_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Target_tracking header(::auto_aim_interfaces::msg::Target::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Target_tracking(msg_);
  }

private:
  ::auto_aim_interfaces::msg::Target msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::Target>()
{
  return auto_aim_interfaces::msg::builder::Init_Target_header();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TARGET__BUILDER_HPP_
