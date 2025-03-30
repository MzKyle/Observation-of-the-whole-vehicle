// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from auto_aim_interfaces:msg/Referee.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__STRUCT_H_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Referee in the package auto_aim_interfaces.
typedef struct auto_aim_interfaces__msg__Referee
{
  uint32_t event_data;
  /// uint16 team
  uint16_t time;
  /// uint8 race
  uint32_t rfid;
  uint16_t base_hp;
  uint16_t sentry_hp;
  /// uint8 ballet_remain
  /// uint8 arm
  uint16_t outpost_hp;
  uint16_t projectile_allowance_17mm;
} auto_aim_interfaces__msg__Referee;

// Struct for a sequence of auto_aim_interfaces__msg__Referee.
typedef struct auto_aim_interfaces__msg__Referee__Sequence
{
  auto_aim_interfaces__msg__Referee * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} auto_aim_interfaces__msg__Referee__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__REFEREE__STRUCT_H_
