// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from buff_interfaces:msg/DebugBlade.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "buff_interfaces/msg/detail/debug_blade__rosidl_typesupport_introspection_c.h"
#include "buff_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "buff_interfaces/msg/detail/debug_blade__functions.h"
#include "buff_interfaces/msg/detail/debug_blade__struct.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `center`
// Member `kpt`
#include "geometry_msgs/msg/point.h"
// Member `center`
// Member `kpt`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  buff_interfaces__msg__DebugBlade__init(message_memory);
}

void buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_fini_function(void * message_memory)
{
  buff_interfaces__msg__DebugBlade__fini(message_memory);
}

size_t buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__size_function__DebugBlade__kpt(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__get_const_function__DebugBlade__kpt(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__get_function__DebugBlade__kpt(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__fetch_function__DebugBlade__kpt(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__get_const_function__DebugBlade__kpt(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__assign_function__DebugBlade__kpt(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__get_function__DebugBlade__kpt(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__resize_function__DebugBlade__kpt(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_member_array[9] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__DebugBlade, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__DebugBlade, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__DebugBlade, width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__DebugBlade, height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "label",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__DebugBlade, label),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "prob",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__DebugBlade, prob),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__DebugBlade, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "center",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__DebugBlade, center),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kpt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(buff_interfaces__msg__DebugBlade, kpt),  // bytes offset in struct
    NULL,  // default value
    buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__size_function__DebugBlade__kpt,  // size() function pointer
    buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__get_const_function__DebugBlade__kpt,  // get_const(index) function pointer
    buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__get_function__DebugBlade__kpt,  // get(index) function pointer
    buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__fetch_function__DebugBlade__kpt,  // fetch(index, &value) function pointer
    buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__assign_function__DebugBlade__kpt,  // assign(index, value) function pointer
    buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__resize_function__DebugBlade__kpt  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_members = {
  "buff_interfaces__msg",  // message namespace
  "DebugBlade",  // message name
  9,  // number of fields
  sizeof(buff_interfaces__msg__DebugBlade),
  buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_member_array,  // message members
  buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_init_function,  // function to initialize message memory (memory has to be allocated)
  buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_type_support_handle = {
  0,
  &buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_buff_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, buff_interfaces, msg, DebugBlade)() {
  buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_type_support_handle.typesupport_identifier) {
    buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &buff_interfaces__msg__DebugBlade__rosidl_typesupport_introspection_c__DebugBlade_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
