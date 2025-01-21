// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from auto_aim_interfaces:msg/DebugLight.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "auto_aim_interfaces/msg/detail/debug_light__rosidl_typesupport_introspection_c.h"
#include "auto_aim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "auto_aim_interfaces/msg/detail/debug_light__functions.h"
#include "auto_aim_interfaces/msg/detail/debug_light__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  auto_aim_interfaces__msg__DebugLight__init(message_memory);
}

void auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_fini_function(void * message_memory)
{
  auto_aim_interfaces__msg__DebugLight__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_message_member_array[4] = {
  {
    "center_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__DebugLight, center_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_light",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__DebugLight, is_light),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ratio",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__DebugLight, ratio),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(auto_aim_interfaces__msg__DebugLight, angle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_message_members = {
  "auto_aim_interfaces__msg",  // message namespace
  "DebugLight",  // message name
  4,  // number of fields
  sizeof(auto_aim_interfaces__msg__DebugLight),
  auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_message_member_array,  // message members
  auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_init_function,  // function to initialize message memory (memory has to be allocated)
  auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_message_type_support_handle = {
  0,
  &auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_auto_aim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, auto_aim_interfaces, msg, DebugLight)() {
  if (!auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_message_type_support_handle.typesupport_identifier) {
    auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &auto_aim_interfaces__msg__DebugLight__rosidl_typesupport_introspection_c__DebugLight_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
