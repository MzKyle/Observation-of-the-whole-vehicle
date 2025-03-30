// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from buff_interfaces:msg/DebugBladeArray.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE_ARRAY__STRUCT_H_
#define BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'blades'
#include "buff_interfaces/msg/detail/debug_blade__struct.h"

/// Struct defined in msg/DebugBladeArray in the package buff_interfaces.
typedef struct buff_interfaces__msg__DebugBladeArray
{
  buff_interfaces__msg__DebugBlade__Sequence blades;
} buff_interfaces__msg__DebugBladeArray;

// Struct for a sequence of buff_interfaces__msg__DebugBladeArray.
typedef struct buff_interfaces__msg__DebugBladeArray__Sequence
{
  buff_interfaces__msg__DebugBladeArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} buff_interfaces__msg__DebugBladeArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE_ARRAY__STRUCT_H_
