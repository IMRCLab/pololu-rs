// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crazyflie_interfaces:msg/ConnectionStatisticsArray.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/msg/detail/connection_statistics_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `stats`
#include "crazyflie_interfaces/msg/detail/connection_statistics__functions.h"

bool
crazyflie_interfaces__msg__ConnectionStatisticsArray__init(crazyflie_interfaces__msg__ConnectionStatisticsArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    crazyflie_interfaces__msg__ConnectionStatisticsArray__fini(msg);
    return false;
  }
  // stats
  if (!crazyflie_interfaces__msg__ConnectionStatistics__Sequence__init(&msg->stats, 0)) {
    crazyflie_interfaces__msg__ConnectionStatisticsArray__fini(msg);
    return false;
  }
  return true;
}

void
crazyflie_interfaces__msg__ConnectionStatisticsArray__fini(crazyflie_interfaces__msg__ConnectionStatisticsArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // stats
  crazyflie_interfaces__msg__ConnectionStatistics__Sequence__fini(&msg->stats);
}

bool
crazyflie_interfaces__msg__ConnectionStatisticsArray__are_equal(const crazyflie_interfaces__msg__ConnectionStatisticsArray * lhs, const crazyflie_interfaces__msg__ConnectionStatisticsArray * rhs)
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
  // stats
  if (!crazyflie_interfaces__msg__ConnectionStatistics__Sequence__are_equal(
      &(lhs->stats), &(rhs->stats)))
  {
    return false;
  }
  return true;
}

bool
crazyflie_interfaces__msg__ConnectionStatisticsArray__copy(
  const crazyflie_interfaces__msg__ConnectionStatisticsArray * input,
  crazyflie_interfaces__msg__ConnectionStatisticsArray * output)
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
  // stats
  if (!crazyflie_interfaces__msg__ConnectionStatistics__Sequence__copy(
      &(input->stats), &(output->stats)))
  {
    return false;
  }
  return true;
}

crazyflie_interfaces__msg__ConnectionStatisticsArray *
crazyflie_interfaces__msg__ConnectionStatisticsArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__ConnectionStatisticsArray * msg = (crazyflie_interfaces__msg__ConnectionStatisticsArray *)allocator.allocate(sizeof(crazyflie_interfaces__msg__ConnectionStatisticsArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crazyflie_interfaces__msg__ConnectionStatisticsArray));
  bool success = crazyflie_interfaces__msg__ConnectionStatisticsArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crazyflie_interfaces__msg__ConnectionStatisticsArray__destroy(crazyflie_interfaces__msg__ConnectionStatisticsArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crazyflie_interfaces__msg__ConnectionStatisticsArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence__init(crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__ConnectionStatisticsArray * data = NULL;

  if (size) {
    data = (crazyflie_interfaces__msg__ConnectionStatisticsArray *)allocator.zero_allocate(size, sizeof(crazyflie_interfaces__msg__ConnectionStatisticsArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crazyflie_interfaces__msg__ConnectionStatisticsArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crazyflie_interfaces__msg__ConnectionStatisticsArray__fini(&data[i - 1]);
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
crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence__fini(crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence * array)
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
      crazyflie_interfaces__msg__ConnectionStatisticsArray__fini(&array->data[i]);
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

crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence *
crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence * array = (crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence *)allocator.allocate(sizeof(crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence__destroy(crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence__are_equal(const crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence * lhs, const crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crazyflie_interfaces__msg__ConnectionStatisticsArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence__copy(
  const crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence * input,
  crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crazyflie_interfaces__msg__ConnectionStatisticsArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crazyflie_interfaces__msg__ConnectionStatisticsArray * data =
      (crazyflie_interfaces__msg__ConnectionStatisticsArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crazyflie_interfaces__msg__ConnectionStatisticsArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crazyflie_interfaces__msg__ConnectionStatisticsArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crazyflie_interfaces__msg__ConnectionStatisticsArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
