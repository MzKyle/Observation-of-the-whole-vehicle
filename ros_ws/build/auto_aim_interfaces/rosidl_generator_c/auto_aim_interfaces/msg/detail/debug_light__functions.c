// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/DebugLight.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/debug_light__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
auto_aim_interfaces__msg__DebugLight__init(auto_aim_interfaces__msg__DebugLight * msg)
{
  if (!msg) {
    return false;
  }
  // center_x
  // is_light
  // ratio
  // angle
  return true;
}

void
auto_aim_interfaces__msg__DebugLight__fini(auto_aim_interfaces__msg__DebugLight * msg)
{
  if (!msg) {
    return;
  }
  // center_x
  // is_light
  // ratio
  // angle
}

bool
auto_aim_interfaces__msg__DebugLight__are_equal(const auto_aim_interfaces__msg__DebugLight * lhs, const auto_aim_interfaces__msg__DebugLight * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // center_x
  if (lhs->center_x != rhs->center_x) {
    return false;
  }
  // is_light
  if (lhs->is_light != rhs->is_light) {
    return false;
  }
  // ratio
  if (lhs->ratio != rhs->ratio) {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__DebugLight__copy(
  const auto_aim_interfaces__msg__DebugLight * input,
  auto_aim_interfaces__msg__DebugLight * output)
{
  if (!input || !output) {
    return false;
  }
  // center_x
  output->center_x = input->center_x;
  // is_light
  output->is_light = input->is_light;
  // ratio
  output->ratio = input->ratio;
  // angle
  output->angle = input->angle;
  return true;
}

auto_aim_interfaces__msg__DebugLight *
auto_aim_interfaces__msg__DebugLight__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__DebugLight * msg = (auto_aim_interfaces__msg__DebugLight *)allocator.allocate(sizeof(auto_aim_interfaces__msg__DebugLight), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__DebugLight));
  bool success = auto_aim_interfaces__msg__DebugLight__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__DebugLight__destroy(auto_aim_interfaces__msg__DebugLight * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__DebugLight__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__DebugLight__Sequence__init(auto_aim_interfaces__msg__DebugLight__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__DebugLight * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__DebugLight *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__DebugLight), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__DebugLight__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__DebugLight__fini(&data[i - 1]);
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
auto_aim_interfaces__msg__DebugLight__Sequence__fini(auto_aim_interfaces__msg__DebugLight__Sequence * array)
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
      auto_aim_interfaces__msg__DebugLight__fini(&array->data[i]);
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

auto_aim_interfaces__msg__DebugLight__Sequence *
auto_aim_interfaces__msg__DebugLight__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__DebugLight__Sequence * array = (auto_aim_interfaces__msg__DebugLight__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__DebugLight__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__DebugLight__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__DebugLight__Sequence__destroy(auto_aim_interfaces__msg__DebugLight__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__DebugLight__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__DebugLight__Sequence__are_equal(const auto_aim_interfaces__msg__DebugLight__Sequence * lhs, const auto_aim_interfaces__msg__DebugLight__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__DebugLight__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__DebugLight__Sequence__copy(
  const auto_aim_interfaces__msg__DebugLight__Sequence * input,
  auto_aim_interfaces__msg__DebugLight__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__DebugLight);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__DebugLight * data =
      (auto_aim_interfaces__msg__DebugLight *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__DebugLight__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__DebugLight__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__DebugLight__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
