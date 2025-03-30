// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from buff_interfaces:msg/DebugBladeArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "buff_interfaces/msg/detail/debug_blade_array__rosidl_typesupport_introspection_c.h"
#include "buff_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "buff_interfaces/msg/detail/debug_blade_array__functions.h"
#include "buff_interfaces/msg/detail/debug_blade_array__struct.h"


// Include directives for member types
// Member `blades`
#include "buff_interfaces/msg/debug_blade.h"
// Member `blades`
#include "buff_interfaces/msg/detail/debug_blade__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  buff_interfaces__msg__DebugBladeArray__init(message_memory);
}

void buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_fini_function(void * message_memory)
{
  buff_interfaces__msg__DebugBladeArray__fini(message_memory);
}

size_t buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__size_function__DebugBladeArray__blades(
  const void * untyped_member)
{
  const buff_interfaces__msg__DebugBlade__Sequence * member =
    (const buff_interfaces__msg__DebugBlade__Sequence *)(untyped_member);
  return member->size;
}

const void * buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__get_const_function__DebugBladeArray__blades(
  const void * untyped_member, size_t index)
{
  const buff_interfaces__msg__DebugBlade__Sequence * member =
    (const buff_interfaces__msg__DebugBlade__Sequence *)(untyped_member);
  return &member->data[index];
}

void * buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__get_function__DebugBladeArray__blades(
  void * untyped_member, size_t index)
{
  buff_interfaces__msg__DebugBlade__Sequence * member =
    (buff_interfaces__msg__DebugBlade__Sequence *)(untyped_member);
  return &member->data[index];
}

void buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__fetch_function__DebugBladeArray__blades(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const buff_interfaces__msg__DebugBlade * item =
    ((const buff_interfaces__msg__DebugBlade *)
    buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__get_const_function__DebugBladeArray__blades(untyped_member, index));
  buff_interfaces__msg__DebugBlade * value =
    (buff_interfaces__msg__DebugBlade *)(untyped_value);
  *value = *item;
}

void buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__assign_function__DebugBladeArray__blades(
  void * untyped_member, size_t index, const void * untyped_value)
{
  buff_interfaces__msg__DebugBlade * item =
    ((buff_interfaces__msg__DebugBlade *)
    buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__get_function__DebugBladeArray__blades(untyped_member, index));
  const buff_interfaces__msg__DebugBlade * value =
    (const buff_interfaces__msg__DebugBlade *)(untyped_value);
  *item = *value;
}

bool buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__resize_function__DebugBladeArray__blades(
  void * untyped_member, size_t size)
{
  buff_interfaces__msg__DebugBlade__Sequence * member =
    (buff_interfaces__msg__DebugBlade__Sequence *)(untyped_member);
  buff_interfaces__msg__DebugBlade__Sequence__fini(member);
  return buff_interfaces__msg__DebugBlade__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_message_member_array[1] = {
  {
    "blades",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__DebugBladeArray, blades),  // bytes offset in struct
    NULL,  // default value
    buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__size_function__DebugBladeArray__blades,  // size() function pointer
    buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__get_const_function__DebugBladeArray__blades,  // get_const(index) function pointer
    buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__get_function__DebugBladeArray__blades,  // get(index) function pointer
    buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__fetch_function__DebugBladeArray__blades,  // fetch(index, &value) function pointer
    buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__assign_function__DebugBladeArray__blades,  // assign(index, value) function pointer
    buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__resize_function__DebugBladeArray__blades  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_message_members = {
  "buff_interfaces__msg",  // message namespace
  "DebugBladeArray",  // message name
  1,  // number of fields
  sizeof(buff_interfaces__msg__DebugBladeArray),
  buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_message_member_array,  // message members
  buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_init_function,  // function to initialize message memory (memory has to be allocated)
  buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_message_type_support_handle = {
  0,
  &buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_buff_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, buff_interfaces, msg, DebugBladeArray)() {
  buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, buff_interfaces, msg, DebugBlade)();
  if (!buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_message_type_support_handle.typesupport_identifier) {
    buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &buff_interfaces__msg__DebugBladeArray__rosidl_typesupport_introspection_c__DebugBladeArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
