// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from buff_interfaces:msg/BladeArray.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__BLADE_ARRAY__STRUCT_H_
#define BUFF_INTERFACES__MSG__DETAIL__BLADE_ARRAY__STRUCT_H_

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
// Member 'blades'
#include "buff_interfaces/msg/detail/blade__struct.h"

/// Struct defined in msg/BladeArray in the package buff_interfaces.
typedef struct buff_interfaces__msg__BladeArray
{
  std_msgs__msg__Header header;
  buff_interfaces__msg__Blade__Sequence blades;
} buff_interfaces__msg__BladeArray;

// Struct for a sequence of buff_interfaces__msg__BladeArray.
typedef struct buff_interfaces__msg__BladeArray__Sequence
{
  buff_interfaces__msg__BladeArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} buff_interfaces__msg__BladeArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BUFF_INTERFACES__MSG__DETAIL__BLADE_ARRAY__STRUCT_H_
