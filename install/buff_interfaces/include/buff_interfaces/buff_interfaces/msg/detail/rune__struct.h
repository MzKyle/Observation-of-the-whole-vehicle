// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from buff_interfaces:msg/Rune.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__RUNE__STRUCT_H_
#define BUFF_INTERFACES__MSG__DETAIL__RUNE__STRUCT_H_

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
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Rune in the package buff_interfaces.
typedef struct buff_interfaces__msg__Rune
{
  std_msgs__msg__Header header;
  bool tracking;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Vector3 velocity;
  double r;
  double theta;
  double a;
  double w;
  double b;
  double c;
  int32_t offset_id;
  int64_t t_offset;
} buff_interfaces__msg__Rune;

// Struct for a sequence of buff_interfaces__msg__Rune.
typedef struct buff_interfaces__msg__Rune__Sequence
{
  buff_interfaces__msg__Rune * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} buff_interfaces__msg__Rune__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BUFF_INTERFACES__MSG__DETAIL__RUNE__STRUCT_H_
