// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/Send.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__SEND__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__SEND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Send in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__Send
{
  std_msgs__msg__Header header;
  bool is_fire;
  geometry_msgs__msg__Point position;
  double v_yaw;
  double pitch;
  double yaw;
} auto_aim_interfaces__msg__Send;

// Struct for a sequence of auto_aim_interfaces__msg__Send.
typedef struct auto_aim_interfaces__msg__Send__Sequence
{
  auto_aim_interfaces__msg__Send * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__Send__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__SEND__STRUCT_H_
