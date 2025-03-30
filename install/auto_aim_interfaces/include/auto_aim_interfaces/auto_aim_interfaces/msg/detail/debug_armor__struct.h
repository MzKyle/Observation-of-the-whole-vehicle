// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/DebugArmor.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMOR__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DebugArmor in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__DebugArmor
{
  int32_t center_x;
  rosidl_runtime_c__String type;
  float light_ratio;
  float center_distance;
  float angle;
} auto_aim_interfaces__msg__DebugArmor;

// Struct for a sequence of auto_aim_interfaces__msg__DebugArmor.
typedef struct auto_aim_interfaces__msg__DebugArmor__Sequence
{
  auto_aim_interfaces__msg__DebugArmor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__DebugArmor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__DEBUG_ARMOR__STRUCT_H_
