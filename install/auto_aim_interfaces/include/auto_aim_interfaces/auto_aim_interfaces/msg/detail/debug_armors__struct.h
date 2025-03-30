// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/DebugArmors.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMORS__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMORS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "auto_aim_interfaces/msg/detail/debug_armor__struct.h"

/// Struct defined in msg/DebugArmors in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__DebugArmors
{
  auto_aim_interfaces__msg__DebugArmor__Sequence data;
} auto_aim_interfaces__msg__DebugArmors;

// Struct for a sequence of auto_aim_interfaces__msg__DebugArmors.
typedef struct auto_aim_interfaces__msg__DebugArmors__Sequence
{
  auto_aim_interfaces__msg__DebugArmors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__DebugArmors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMORS__STRUCT_H_
