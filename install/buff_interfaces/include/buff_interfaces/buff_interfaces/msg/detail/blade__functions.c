// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from buff_interfaces:msg/Blade.idl
// generated code does not contain a copyright notice
#include "buff_interfaces/msg/detail/blade__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
buff_interfaces__msg__Blade__init(buff_interfaces__msg__Blade * msg)
{
  if (!msg) {
    return false;
  }
  // label
  // prob
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    buff_interfaces__msg__Blade__fini(msg);
    return false;
  }
  return true;
}

void
buff_interfaces__msg__Blade__fini(buff_interfaces__msg__Blade * msg)
{
  if (!msg) {
    return;
  }
  // label
  // prob
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
}

bool
buff_interfaces__msg__Blade__are_equal(const buff_interfaces__msg__Blade * lhs, const buff_interfaces__msg__Blade * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // label
  if (lhs->label != rhs->label) {
    return false;
  }
  // prob
  if (lhs->prob != rhs->prob) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
buff_interfaces__msg__Blade__copy(
  const buff_interfaces__msg__Blade * input,
  buff_interfaces__msg__Blade * output)
{
  if (!input || !output) {
    return false;
  }
  // label
  output->label = input->label;
  // prob
  output->prob = input->prob;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

buff_interfaces__msg__Blade *
buff_interfaces__msg__Blade__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  buff_interfaces__msg__Blade * msg = (buff_interfaces__msg__Blade *)allocator.allocate(sizeof(buff_interfaces__msg__Blade), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(buff_interfaces__msg__Blade));
  bool success = buff_interfaces__msg__Blade__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
buff_interfaces__msg__Blade__destroy(buff_interfaces__msg__Blade * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    buff_interfaces__msg__Blade__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
buff_interfaces__msg__Blade__Sequence__init(buff_interfaces__msg__Blade__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  buff_interfaces__msg__Blade * data = NULL;

  if (size) {
    data = (buff_interfaces__msg__Blade *)allocator.zero_allocate(size, sizeof(buff_interfaces__msg__Blade), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = buff_interfaces__msg__Blade__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        buff_interfaces__msg__Blade__fini(&data[i - 1]);
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
buff_interfaces__msg__Blade__Sequence__fini(buff_interfaces__msg__Blade__Sequence * array)
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
      buff_interfaces__msg__Blade__fini(&array->data[i]);
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

buff_interfaces__msg__Blade__Sequence *
buff_interfaces__msg__Blade__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  buff_interfaces__msg__Blade__Sequence * array = (buff_interfaces__msg__Blade__Sequence *)allocator.allocate(sizeof(buff_interfaces__msg__Blade__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = buff_interfaces__msg__Blade__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
buff_interfaces__msg__Blade__Sequence__destroy(buff_interfaces__msg__Blade__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    buff_interfaces__msg__Blade__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
buff_interfaces__msg__Blade__Sequence__are_equal(const buff_interfaces__msg__Blade__Sequence * lhs, const buff_interfaces__msg__Blade__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!buff_interfaces__msg__Blade__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
buff_interfaces__msg__Blade__Sequence__copy(
  const buff_interfaces__msg__Blade__Sequence * input,
  buff_interfaces__msg__Blade__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(buff_interfaces__msg__Blade);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    buff_interfaces__msg__Blade * data =
      (buff_interfaces__msg__Blade *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!buff_interfaces__msg__Blade__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          buff_interfaces__msg__Blade__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!buff_interfaces__msg__Blade__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
