// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from buff_interfaces:msg/Rune.idl
// generated code does not contain a copyright notice
#include "buff_interfaces/msg/detail/rune__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
buff_interfaces__msg__Rune__init(buff_interfaces__msg__Rune * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    buff_interfaces__msg__Rune__fini(msg);
    return false;
  }
  // tracking
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    buff_interfaces__msg__Rune__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    buff_interfaces__msg__Rune__fini(msg);
    return false;
  }
  // r
  // theta
  // a
  // w
  // b
  // c
  // offset_id
  // t_offset
  return true;
}

void
buff_interfaces__msg__Rune__fini(buff_interfaces__msg__Rune * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // tracking
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // r
  // theta
  // a
  // w
  // b
  // c
  // offset_id
  // t_offset
}

bool
buff_interfaces__msg__Rune__are_equal(const buff_interfaces__msg__Rune * lhs, const buff_interfaces__msg__Rune * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // tracking
  if (lhs->tracking != rhs->tracking) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // r
  if (lhs->r != rhs->r) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  // a
  if (lhs->a != rhs->a) {
    return false;
  }
  // w
  if (lhs->w != rhs->w) {
    return false;
  }
  // b
  if (lhs->b != rhs->b) {
    return false;
  }
  // c
  if (lhs->c != rhs->c) {
    return false;
  }
  // offset_id
  if (lhs->offset_id != rhs->offset_id) {
    return false;
  }
  // t_offset
  if (lhs->t_offset != rhs->t_offset) {
    return false;
  }
  return true;
}

bool
buff_interfaces__msg__Rune__copy(
  const buff_interfaces__msg__Rune * input,
  buff_interfaces__msg__Rune * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // tracking
  output->tracking = input->tracking;
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // r
  output->r = input->r;
  // theta
  output->theta = input->theta;
  // a
  output->a = input->a;
  // w
  output->w = input->w;
  // b
  output->b = input->b;
  // c
  output->c = input->c;
  // offset_id
  output->offset_id = input->offset_id;
  // t_offset
  output->t_offset = input->t_offset;
  return true;
}

buff_interfaces__msg__Rune *
buff_interfaces__msg__Rune__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  buff_interfaces__msg__Rune * msg = (buff_interfaces__msg__Rune *)allocator.allocate(sizeof(buff_interfaces__msg__Rune), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(buff_interfaces__msg__Rune));
  bool success = buff_interfaces__msg__Rune__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
buff_interfaces__msg__Rune__destroy(buff_interfaces__msg__Rune * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    buff_interfaces__msg__Rune__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
buff_interfaces__msg__Rune__Sequence__init(buff_interfaces__msg__Rune__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  buff_interfaces__msg__Rune * data = NULL;

  if (size) {
    data = (buff_interfaces__msg__Rune *)allocator.zero_allocate(size, sizeof(buff_interfaces__msg__Rune), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = buff_interfaces__msg__Rune__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        buff_interfaces__msg__Rune__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
buff_interfaces__msg__Rune__Sequence__fini(buff_interfaces__msg__Rune__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      buff_interfaces__msg__Rune__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

buff_interfaces__msg__Rune__Sequence *
buff_interfaces__msg__Rune__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  buff_interfaces__msg__Rune__Sequence * array = (buff_interfaces__msg__Rune__Sequence *)allocator.allocate(sizeof(buff_interfaces__msg__Rune__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = buff_interfaces__msg__Rune__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
buff_interfaces__msg__Rune__Sequence__destroy(buff_interfaces__msg__Rune__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    buff_interfaces__msg__Rune__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
buff_interfaces__msg__Rune__Sequence__are_equal(const buff_interfaces__msg__Rune__Sequence * lhs, const buff_interfaces__msg__Rune__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!buff_interfaces__msg__Rune__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
buff_interfaces__msg__Rune__Sequence__copy(
  const buff_interfaces__msg__Rune__Sequence * input,
  buff_interfaces__msg__Rune__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(buff_interfaces__msg__Rune);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    buff_interfaces__msg__Rune * data =
      (buff_interfaces__msg__Rune *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!buff_interfaces__msg__Rune__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          buff_interfaces__msg__Rune__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!buff_interfaces__msg__Rune__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
