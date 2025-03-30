// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from auto_aim_interfaces:msg/Referee.idl
// generated code does not contain a copyright notice
#include "auto_aim_interfaces/msg/detail/referee__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
auto_aim_interfaces__msg__Referee__init(auto_aim_interfaces__msg__Referee * msg)
{
  if (!msg) {
    return false;
  }
  // event_data
  // time
  // rfid
  // base_hp
  // sentry_hp
  // outpost_hp
  // projectile_allowance_17mm
  return true;
}

void
auto_aim_interfaces__msg__Referee__fini(auto_aim_interfaces__msg__Referee * msg)
{
  if (!msg) {
    return;
  }
  // event_data
  // time
  // rfid
  // base_hp
  // sentry_hp
  // outpost_hp
  // projectile_allowance_17mm
}

bool
auto_aim_interfaces__msg__Referee__are_equal(const auto_aim_interfaces__msg__Referee * lhs, const auto_aim_interfaces__msg__Referee * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // event_data
  if (lhs->event_data != rhs->event_data) {
    return false;
  }
  // time
  if (lhs->time != rhs->time) {
    return false;
  }
  // rfid
  if (lhs->rfid != rhs->rfid) {
    return false;
  }
  // base_hp
  if (lhs->base_hp != rhs->base_hp) {
    return false;
  }
  // sentry_hp
  if (lhs->sentry_hp != rhs->sentry_hp) {
    return false;
  }
  // outpost_hp
  if (lhs->outpost_hp != rhs->outpost_hp) {
    return false;
  }
  // projectile_allowance_17mm
  if (lhs->projectile_allowance_17mm != rhs->projectile_allowance_17mm) {
    return false;
  }
  return true;
}

bool
auto_aim_interfaces__msg__Referee__copy(
  const auto_aim_interfaces__msg__Referee * input,
  auto_aim_interfaces__msg__Referee * output)
{
  if (!input || !output) {
    return false;
  }
  // event_data
  output->event_data = input->event_data;
  // time
  output->time = input->time;
  // rfid
  output->rfid = input->rfid;
  // base_hp
  output->base_hp = input->base_hp;
  // sentry_hp
  output->sentry_hp = input->sentry_hp;
  // outpost_hp
  output->outpost_hp = input->outpost_hp;
  // projectile_allowance_17mm
  output->projectile_allowance_17mm = input->projectile_allowance_17mm;
  return true;
}

auto_aim_interfaces__msg__Referee *
auto_aim_interfaces__msg__Referee__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Referee * msg = (auto_aim_interfaces__msg__Referee *)allocator.allocate(sizeof(auto_aim_interfaces__msg__Referee), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(auto_aim_interfaces__msg__Referee));
  bool success = auto_aim_interfaces__msg__Referee__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
auto_aim_interfaces__msg__Referee__destroy(auto_aim_interfaces__msg__Referee * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    auto_aim_interfaces__msg__Referee__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
auto_aim_interfaces__msg__Referee__Sequence__init(auto_aim_interfaces__msg__Referee__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Referee * data = NULL;

  if (size) {
    data = (auto_aim_interfaces__msg__Referee *)allocator.zero_allocate(size, sizeof(auto_aim_interfaces__msg__Referee), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = auto_aim_interfaces__msg__Referee__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        auto_aim_interfaces__msg__Referee__fini(&data[i - 1]);
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
auto_aim_interfaces__msg__Referee__Sequence__fini(auto_aim_interfaces__msg__Referee__Sequence * array)
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
      auto_aim_interfaces__msg__Referee__fini(&array->data[i]);
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

auto_aim_interfaces__msg__Referee__Sequence *
auto_aim_interfaces__msg__Referee__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  auto_aim_interfaces__msg__Referee__Sequence * array = (auto_aim_interfaces__msg__Referee__Sequence *)allocator.allocate(sizeof(auto_aim_interfaces__msg__Referee__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = auto_aim_interfaces__msg__Referee__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
auto_aim_interfaces__msg__Referee__Sequence__destroy(auto_aim_interfaces__msg__Referee__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    auto_aim_interfaces__msg__Referee__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
auto_aim_interfaces__msg__Referee__Sequence__are_equal(const auto_aim_interfaces__msg__Referee__Sequence * lhs, const auto_aim_interfaces__msg__Referee__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!auto_aim_interfaces__msg__Referee__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
auto_aim_interfaces__msg__Referee__Sequence__copy(
  const auto_aim_interfaces__msg__Referee__Sequence * input,
  auto_aim_interfaces__msg__Referee__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(auto_aim_interfaces__msg__Referee);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    auto_aim_interfaces__msg__Referee * data =
      (auto_aim_interfaces__msg__Referee *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!auto_aim_interfaces__msg__Referee__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          auto_aim_interfaces__msg__Referee__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!auto_aim_interfaces__msg__Referee__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
