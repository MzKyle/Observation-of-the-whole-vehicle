// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from buff_interfaces:msg/DebugBlade.idl
// generated code does not contain a copyright notice

#ifndef BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE__STRUCT_H_
#define BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE__STRUCT_H_

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
// Member 'center'
// Member 'kpt'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/DebugBlade in the package buff_interfaces.
typedef struct buff_interfaces__msg__DebugBlade
{
  /// 矩形框左上角的坐标
  double x;
  double y;
  /// 矩形框的宽度和高度
  double width;
  double height;
  /// 大小符
  int64_t label;
  /// 目标检测的置信度
  double prob;
  /// 位姿信息
  geometry_msgs__msg__Pose pose;
  /// 中心点信息
  geometry_msgs__msg__Point center;
  /// 关键点信息
  geometry_msgs__msg__Point__Sequence kpt;
} buff_interfaces__msg__DebugBlade;

// Struct for a sequence of buff_interfaces__msg__DebugBlade.
typedef struct buff_interfaces__msg__DebugBlade__Sequence
{
  buff_interfaces__msg__DebugBlade * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} buff_interfaces__msg__DebugBlade__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BUFF_INTERFACES__MSG__DETAIL__DEBUG_BLADE__STRUCT_H_
