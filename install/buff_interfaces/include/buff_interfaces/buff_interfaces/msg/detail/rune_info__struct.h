// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from buff_interfaces:msg/RuneInfo.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__STRUCT_H_
#define BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__STRUCT_H_

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
// Member 'blade'
// Member 'center'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/RuneInfo in the package buff_interfaces.
typedef struct buff_interfaces__msg__RuneInfo
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Point blade;
  geometry_msgs__msg__Point center;
  double speed;
  double predicted_speed;
} buff_interfaces__msg__RuneInfo;

// Struct for a sequence of buff_interfaces__msg__RuneInfo.
typedef struct buff_interfaces__msg__RuneInfo__Sequence
{
  buff_interfaces__msg__RuneInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} buff_interfaces__msg__RuneInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BUFF_INTERFACES__MSG__DETAIL__RUNE_INFO__STRUCT_H_
