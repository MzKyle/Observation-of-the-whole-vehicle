// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from buff_interfaces:msg/Blade.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__BLADE__STRUCT_H_
#define BUFF_INTERFACES__MSG__DETAIL__BLADE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/Blade in the package buff_interfaces.
typedef struct buff_interfaces__msg__Blade
{
  int64_t label;
  double prob;
  geometry_msgs__msg__Pose pose;
} buff_interfaces__msg__Blade;

// Struct for a sequence of buff_interfaces__msg__Blade.
typedef struct buff_interfaces__msg__Blade__Sequence
{
  buff_interfaces__msg__Blade * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} buff_interfaces__msg__Blade__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BUFF_INTERFACES__MSG__DETAIL__BLADE__STRUCT_H_
