// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crazyflie_interfaces:msg/LogDataGeneric.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/msg/detail/log_data_generic__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `values`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
crazyflie_interfaces__msg__LogDataGeneric__init(crazyflie_interfaces__msg__LogDataGeneric * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    crazyflie_interfaces__msg__LogDataGeneric__fini(msg);
    return false;
  }
  // timestamp
  // values
  if (!rosidl_runtime_c__float__Sequence__init(&msg->values, 0)) {
    crazyflie_interfaces__msg__LogDataGeneric__fini(msg);
    return false;
  }
  return true;
}

void
crazyflie_interfaces__msg__LogDataGeneric__fini(crazyflie_interfaces__msg__LogDataGeneric * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // timestamp
  // values
  rosidl_runtime_c__float__Sequence__fini(&msg->values);
}

bool
crazyflie_interfaces__msg__LogDataGeneric__are_equal(const crazyflie_interfaces__msg__LogDataGeneric * lhs, const crazyflie_interfaces__msg__LogDataGeneric * rhs)
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
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // values
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->values), &(rhs->values)))
  {
    return false;
  }
  return true;
}

bool
crazyflie_interfaces__msg__LogDataGeneric__copy(
  const crazyflie_interfaces__msg__LogDataGeneric * input,
  crazyflie_interfaces__msg__LogDataGeneric * output)
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
  // timestamp
  output->timestamp = input->timestamp;
  // values
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->values), &(output->values)))
  {
    return false;
  }
  return true;
}

crazyflie_interfaces__msg__LogDataGeneric *
crazyflie_interfaces__msg__LogDataGeneric__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__LogDataGeneric * msg = (crazyflie_interfaces__msg__LogDataGeneric *)allocator.allocate(sizeof(crazyflie_interfaces__msg__LogDataGeneric), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crazyflie_interfaces__msg__LogDataGeneric));
  bool success = crazyflie_interfaces__msg__LogDataGeneric__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crazyflie_interfaces__msg__LogDataGeneric__destroy(crazyflie_interfaces__msg__LogDataGeneric * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crazyflie_interfaces__msg__LogDataGeneric__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crazyflie_interfaces__msg__LogDataGeneric__Sequence__init(crazyflie_interfaces__msg__LogDataGeneric__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__LogDataGeneric * data = NULL;

  if (size) {
    data = (crazyflie_interfaces__msg__LogDataGeneric *)allocator.zero_allocate(size, sizeof(crazyflie_interfaces__msg__LogDataGeneric), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crazyflie_interfaces__msg__LogDataGeneric__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crazyflie_interfaces__msg__LogDataGeneric__fini(&data[i - 1]);
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
crazyflie_interfaces__msg__LogDataGeneric__Sequence__fini(crazyflie_interfaces__msg__LogDataGeneric__Sequence * array)
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
      crazyflie_interfaces__msg__LogDataGeneric__fini(&array->data[i]);
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

crazyflie_interfaces__msg__LogDataGeneric__Sequence *
crazyflie_interfaces__msg__LogDataGeneric__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__LogDataGeneric__Sequence * array = (crazyflie_interfaces__msg__LogDataGeneric__Sequence *)allocator.allocate(sizeof(crazyflie_interfaces__msg__LogDataGeneric__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crazyflie_interfaces__msg__LogDataGeneric__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crazyflie_interfaces__msg__LogDataGeneric__Sequence__destroy(crazyflie_interfaces__msg__LogDataGeneric__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crazyflie_interfaces__msg__LogDataGeneric__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crazyflie_interfaces__msg__LogDataGeneric__Sequence__are_equal(const crazyflie_interfaces__msg__LogDataGeneric__Sequence * lhs, const crazyflie_interfaces__msg__LogDataGeneric__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crazyflie_interfaces__msg__LogDataGeneric__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crazyflie_interfaces__msg__LogDataGeneric__Sequence__copy(
  const crazyflie_interfaces__msg__LogDataGeneric__Sequence * input,
  crazyflie_interfaces__msg__LogDataGeneric__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crazyflie_interfaces__msg__LogDataGeneric);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crazyflie_interfaces__msg__LogDataGeneric * data =
      (crazyflie_interfaces__msg__LogDataGeneric *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crazyflie_interfaces__msg__LogDataGeneric__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crazyflie_interfaces__msg__LogDataGeneric__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crazyflie_interfaces__msg__LogDataGeneric__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
