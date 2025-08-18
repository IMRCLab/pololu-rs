// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crazyflie_interfaces:msg/ConnectionStatistics.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/msg/detail/connection_statistics__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `uri`
#include "rosidl_runtime_c/string_functions.h"

bool
crazyflie_interfaces__msg__ConnectionStatistics__init(crazyflie_interfaces__msg__ConnectionStatistics * msg)
{
  if (!msg) {
    return false;
  }
  // uri
  if (!rosidl_runtime_c__String__init(&msg->uri)) {
    crazyflie_interfaces__msg__ConnectionStatistics__fini(msg);
    return false;
  }
  // sent_count
  // sent_ping_count
  // receive_count
  // enqueued_count
  // ack_count
  return true;
}

void
crazyflie_interfaces__msg__ConnectionStatistics__fini(crazyflie_interfaces__msg__ConnectionStatistics * msg)
{
  if (!msg) {
    return;
  }
  // uri
  rosidl_runtime_c__String__fini(&msg->uri);
  // sent_count
  // sent_ping_count
  // receive_count
  // enqueued_count
  // ack_count
}

bool
crazyflie_interfaces__msg__ConnectionStatistics__are_equal(const crazyflie_interfaces__msg__ConnectionStatistics * lhs, const crazyflie_interfaces__msg__ConnectionStatistics * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // uri
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->uri), &(rhs->uri)))
  {
    return false;
  }
  // sent_count
  if (lhs->sent_count != rhs->sent_count) {
    return false;
  }
  // sent_ping_count
  if (lhs->sent_ping_count != rhs->sent_ping_count) {
    return false;
  }
  // receive_count
  if (lhs->receive_count != rhs->receive_count) {
    return false;
  }
  // enqueued_count
  if (lhs->enqueued_count != rhs->enqueued_count) {
    return false;
  }
  // ack_count
  if (lhs->ack_count != rhs->ack_count) {
    return false;
  }
  return true;
}

bool
crazyflie_interfaces__msg__ConnectionStatistics__copy(
  const crazyflie_interfaces__msg__ConnectionStatistics * input,
  crazyflie_interfaces__msg__ConnectionStatistics * output)
{
  if (!input || !output) {
    return false;
  }
  // uri
  if (!rosidl_runtime_c__String__copy(
      &(input->uri), &(output->uri)))
  {
    return false;
  }
  // sent_count
  output->sent_count = input->sent_count;
  // sent_ping_count
  output->sent_ping_count = input->sent_ping_count;
  // receive_count
  output->receive_count = input->receive_count;
  // enqueued_count
  output->enqueued_count = input->enqueued_count;
  // ack_count
  output->ack_count = input->ack_count;
  return true;
}

crazyflie_interfaces__msg__ConnectionStatistics *
crazyflie_interfaces__msg__ConnectionStatistics__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__ConnectionStatistics * msg = (crazyflie_interfaces__msg__ConnectionStatistics *)allocator.allocate(sizeof(crazyflie_interfaces__msg__ConnectionStatistics), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crazyflie_interfaces__msg__ConnectionStatistics));
  bool success = crazyflie_interfaces__msg__ConnectionStatistics__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crazyflie_interfaces__msg__ConnectionStatistics__destroy(crazyflie_interfaces__msg__ConnectionStatistics * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crazyflie_interfaces__msg__ConnectionStatistics__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crazyflie_interfaces__msg__ConnectionStatistics__Sequence__init(crazyflie_interfaces__msg__ConnectionStatistics__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__ConnectionStatistics * data = NULL;

  if (size) {
    data = (crazyflie_interfaces__msg__ConnectionStatistics *)allocator.zero_allocate(size, sizeof(crazyflie_interfaces__msg__ConnectionStatistics), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crazyflie_interfaces__msg__ConnectionStatistics__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crazyflie_interfaces__msg__ConnectionStatistics__fini(&data[i - 1]);
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
crazyflie_interfaces__msg__ConnectionStatistics__Sequence__fini(crazyflie_interfaces__msg__ConnectionStatistics__Sequence * array)
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
      crazyflie_interfaces__msg__ConnectionStatistics__fini(&array->data[i]);
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

crazyflie_interfaces__msg__ConnectionStatistics__Sequence *
crazyflie_interfaces__msg__ConnectionStatistics__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__ConnectionStatistics__Sequence * array = (crazyflie_interfaces__msg__ConnectionStatistics__Sequence *)allocator.allocate(sizeof(crazyflie_interfaces__msg__ConnectionStatistics__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crazyflie_interfaces__msg__ConnectionStatistics__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crazyflie_interfaces__msg__ConnectionStatistics__Sequence__destroy(crazyflie_interfaces__msg__ConnectionStatistics__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crazyflie_interfaces__msg__ConnectionStatistics__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crazyflie_interfaces__msg__ConnectionStatistics__Sequence__are_equal(const crazyflie_interfaces__msg__ConnectionStatistics__Sequence * lhs, const crazyflie_interfaces__msg__ConnectionStatistics__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crazyflie_interfaces__msg__ConnectionStatistics__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crazyflie_interfaces__msg__ConnectionStatistics__Sequence__copy(
  const crazyflie_interfaces__msg__ConnectionStatistics__Sequence * input,
  crazyflie_interfaces__msg__ConnectionStatistics__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crazyflie_interfaces__msg__ConnectionStatistics);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crazyflie_interfaces__msg__ConnectionStatistics * data =
      (crazyflie_interfaces__msg__ConnectionStatistics *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crazyflie_interfaces__msg__ConnectionStatistics__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crazyflie_interfaces__msg__ConnectionStatistics__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crazyflie_interfaces__msg__ConnectionStatistics__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
