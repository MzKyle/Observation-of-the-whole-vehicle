// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from auto_aim_interfaces:msg/TrackerInfo.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__BUILDER_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "auto_aim_interfaces/msg/detail/tracker_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace auto_aim_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrackerInfo_yaw
{
public:
  explicit Init_TrackerInfo_yaw(::auto_aim_interfaces::msg::TrackerInfo & msg)
  : msg_(msg)
  {}
  ::auto_aim_interfaces::msg::TrackerInfo yaw(::auto_aim_interfaces::msg::TrackerInfo::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerInfo msg_;
};

class Init_TrackerInfo_position
{
public:
  explicit Init_TrackerInfo_position(::auto_aim_interfaces::msg::TrackerInfo & msg)
  : msg_(msg)
  {}
  Init_TrackerInfo_yaw position(::auto_aim_interfaces::msg::TrackerInfo::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_TrackerInfo_yaw(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerInfo msg_;
};

class Init_TrackerInfo_yaw_diff
{
public:
  explicit Init_TrackerInfo_yaw_diff(::auto_aim_interfaces::msg::TrackerInfo & msg)
  : msg_(msg)
  {}
  Init_TrackerInfo_position yaw_diff(::auto_aim_interfaces::msg::TrackerInfo::_yaw_diff_type arg)
  {
    msg_.yaw_diff = std::move(arg);
    return Init_TrackerInfo_position(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerInfo msg_;
};

class Init_TrackerInfo_position_diff
{
public:
  Init_TrackerInfo_position_diff()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackerInfo_yaw_diff position_diff(::auto_aim_interfaces::msg::TrackerInfo::_position_diff_type arg)
  {
    msg_.position_diff = std::move(arg);
    return Init_TrackerInfo_yaw_diff(msg_);
  }

private:
  ::auto_aim_interfaces::msg::TrackerInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::auto_aim_interfaces::msg::TrackerInfo>()
{
  return auto_aim_interfaces::msg::builder::Init_TrackerInfo_position_diff();
}

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__TRACKER_INFO__BUILDER_HPP_
