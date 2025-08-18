// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crazyflie_interfaces:msg/Status.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/msg/detail/status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
crazyflie_interfaces__msg__Status__init(crazyflie_interfaces__msg__Status * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    crazyflie_interfaces__msg__Status__fini(msg);
    return false;
  }
  // supervisor_info
  // battery_voltage
  // pm_state
  // rssi
  // num_rx_broadcast
  // num_tx_broadcast
  // num_rx_unicast
  // num_tx_unicast
  // latency_unicast
  return true;
}

void
crazyflie_interfaces__msg__Status__fini(crazyflie_interfaces__msg__Status * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // supervisor_info
  // battery_voltage
  // pm_state
  // rssi
  // num_rx_broadcast
  // num_tx_broadcast
  // num_rx_unicast
  // num_tx_unicast
  // latency_unicast
}

bool
crazyflie_interfaces__msg__Status__are_equal(const crazyflie_interfaces__msg__Status * lhs, const crazyflie_interfaces__msg__Status * rhs)
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
  // supervisor_info
  if (lhs->supervisor_info != rhs->supervisor_info) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // pm_state
  if (lhs->pm_state != rhs->pm_state) {
    return false;
  }
  // rssi
  if (lhs->rssi != rhs->rssi) {
    return false;
  }
  // num_rx_broadcast
  if (lhs->num_rx_broadcast != rhs->num_rx_broadcast) {
    return false;
  }
  // num_tx_broadcast
  if (lhs->num_tx_broadcast != rhs->num_tx_broadcast) {
    return false;
  }
  // num_rx_unicast
  if (lhs->num_rx_unicast != rhs->num_rx_unicast) {
    return false;
  }
  // num_tx_unicast
  if (lhs->num_tx_unicast != rhs->num_tx_unicast) {
    return false;
  }
  // latency_unicast
  if (lhs->latency_unicast != rhs->latency_unicast) {
    return false;
  }
  return true;
}

bool
crazyflie_interfaces__msg__Status__copy(
  const crazyflie_interfaces__msg__Status * input,
  crazyflie_interfaces__msg__Status * output)
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
  // supervisor_info
  output->supervisor_info = input->supervisor_info;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // pm_state
  output->pm_state = input->pm_state;
  // rssi
  output->rssi = input->rssi;
  // num_rx_broadcast
  output->num_rx_broadcast = input->num_rx_broadcast;
  // num_tx_broadcast
  output->num_tx_broadcast = input->num_tx_broadcast;
  // num_rx_unicast
  output->num_rx_unicast = input->num_rx_unicast;
  // num_tx_unicast
  output->num_tx_unicast = input->num_tx_unicast;
  // latency_unicast
  output->latency_unicast = input->latency_unicast;
  return true;
}

crazyflie_interfaces__msg__Status *
crazyflie_interfaces__msg__Status__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__Status * msg = (crazyflie_interfaces__msg__Status *)allocator.allocate(sizeof(crazyflie_interfaces__msg__Status), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crazyflie_interfaces__msg__Status));
  bool success = crazyflie_interfaces__msg__Status__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crazyflie_interfaces__msg__Status__destroy(crazyflie_interfaces__msg__Status * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crazyflie_interfaces__msg__Status__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crazyflie_interfaces__msg__Status__Sequence__init(crazyflie_interfaces__msg__Status__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__Status * data = NULL;

  if (size) {
    data = (crazyflie_interfaces__msg__Status *)allocator.zero_allocate(size, sizeof(crazyflie_interfaces__msg__Status), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crazyflie_interfaces__msg__Status__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crazyflie_interfaces__msg__Status__fini(&data[i - 1]);
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
crazyflie_interfaces__msg__Status__Sequence__fini(crazyflie_interfaces__msg__Status__Sequence * array)
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
      crazyflie_interfaces__msg__Status__fini(&array->data[i]);
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

crazyflie_interfaces__msg__Status__Sequence *
crazyflie_interfaces__msg__Status__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__Status__Sequence * array = (crazyflie_interfaces__msg__Status__Sequence *)allocator.allocate(sizeof(crazyflie_interfaces__msg__Status__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crazyflie_interfaces__msg__Status__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crazyflie_interfaces__msg__Status__Sequence__destroy(crazyflie_interfaces__msg__Status__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crazyflie_interfaces__msg__Status__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crazyflie_interfaces__msg__Status__Sequence__are_equal(const crazyflie_interfaces__msg__Status__Sequence * lhs, const crazyflie_interfaces__msg__Status__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crazyflie_interfaces__msg__Status__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crazyflie_interfaces__msg__Status__Sequence__copy(
  const crazyflie_interfaces__msg__Status__Sequence * input,
  crazyflie_interfaces__msg__Status__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crazyflie_interfaces__msg__Status);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crazyflie_interfaces__msg__Status * data =
      (crazyflie_interfaces__msg__Status *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crazyflie_interfaces__msg__Status__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crazyflie_interfaces__msg__Status__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crazyflie_interfaces__msg__Status__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
