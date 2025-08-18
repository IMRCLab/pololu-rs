// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crazyflie_interfaces:msg/TrajectoryPolynomialPiece.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/msg/detail/trajectory_polynomial_piece__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `poly_x`
// Member `poly_y`
// Member `poly_z`
// Member `poly_yaw`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `duration`
#include "builtin_interfaces/msg/detail/duration__functions.h"

bool
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__init(crazyflie_interfaces__msg__TrajectoryPolynomialPiece * msg)
{
  if (!msg) {
    return false;
  }
  // poly_x
  if (!rosidl_runtime_c__float__Sequence__init(&msg->poly_x, 0)) {
    crazyflie_interfaces__msg__TrajectoryPolynomialPiece__fini(msg);
    return false;
  }
  // poly_y
  if (!rosidl_runtime_c__float__Sequence__init(&msg->poly_y, 0)) {
    crazyflie_interfaces__msg__TrajectoryPolynomialPiece__fini(msg);
    return false;
  }
  // poly_z
  if (!rosidl_runtime_c__float__Sequence__init(&msg->poly_z, 0)) {
    crazyflie_interfaces__msg__TrajectoryPolynomialPiece__fini(msg);
    return false;
  }
  // poly_yaw
  if (!rosidl_runtime_c__float__Sequence__init(&msg->poly_yaw, 0)) {
    crazyflie_interfaces__msg__TrajectoryPolynomialPiece__fini(msg);
    return false;
  }
  // duration
  if (!builtin_interfaces__msg__Duration__init(&msg->duration)) {
    crazyflie_interfaces__msg__TrajectoryPolynomialPiece__fini(msg);
    return false;
  }
  return true;
}

void
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__fini(crazyflie_interfaces__msg__TrajectoryPolynomialPiece * msg)
{
  if (!msg) {
    return;
  }
  // poly_x
  rosidl_runtime_c__float__Sequence__fini(&msg->poly_x);
  // poly_y
  rosidl_runtime_c__float__Sequence__fini(&msg->poly_y);
  // poly_z
  rosidl_runtime_c__float__Sequence__fini(&msg->poly_z);
  // poly_yaw
  rosidl_runtime_c__float__Sequence__fini(&msg->poly_yaw);
  // duration
  builtin_interfaces__msg__Duration__fini(&msg->duration);
}

bool
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__are_equal(const crazyflie_interfaces__msg__TrajectoryPolynomialPiece * lhs, const crazyflie_interfaces__msg__TrajectoryPolynomialPiece * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // poly_x
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->poly_x), &(rhs->poly_x)))
  {
    return false;
  }
  // poly_y
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->poly_y), &(rhs->poly_y)))
  {
    return false;
  }
  // poly_z
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->poly_z), &(rhs->poly_z)))
  {
    return false;
  }
  // poly_yaw
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->poly_yaw), &(rhs->poly_yaw)))
  {
    return false;
  }
  // duration
  if (!builtin_interfaces__msg__Duration__are_equal(
      &(lhs->duration), &(rhs->duration)))
  {
    return false;
  }
  return true;
}

bool
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__copy(
  const crazyflie_interfaces__msg__TrajectoryPolynomialPiece * input,
  crazyflie_interfaces__msg__TrajectoryPolynomialPiece * output)
{
  if (!input || !output) {
    return false;
  }
  // poly_x
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->poly_x), &(output->poly_x)))
  {
    return false;
  }
  // poly_y
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->poly_y), &(output->poly_y)))
  {
    return false;
  }
  // poly_z
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->poly_z), &(output->poly_z)))
  {
    return false;
  }
  // poly_yaw
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->poly_yaw), &(output->poly_yaw)))
  {
    return false;
  }
  // duration
  if (!builtin_interfaces__msg__Duration__copy(
      &(input->duration), &(output->duration)))
  {
    return false;
  }
  return true;
}

crazyflie_interfaces__msg__TrajectoryPolynomialPiece *
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__TrajectoryPolynomialPiece * msg = (crazyflie_interfaces__msg__TrajectoryPolynomialPiece *)allocator.allocate(sizeof(crazyflie_interfaces__msg__TrajectoryPolynomialPiece), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crazyflie_interfaces__msg__TrajectoryPolynomialPiece));
  bool success = crazyflie_interfaces__msg__TrajectoryPolynomialPiece__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__destroy(crazyflie_interfaces__msg__TrajectoryPolynomialPiece * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crazyflie_interfaces__msg__TrajectoryPolynomialPiece__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence__init(crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__TrajectoryPolynomialPiece * data = NULL;

  if (size) {
    data = (crazyflie_interfaces__msg__TrajectoryPolynomialPiece *)allocator.zero_allocate(size, sizeof(crazyflie_interfaces__msg__TrajectoryPolynomialPiece), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crazyflie_interfaces__msg__TrajectoryPolynomialPiece__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crazyflie_interfaces__msg__TrajectoryPolynomialPiece__fini(&data[i - 1]);
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
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence__fini(crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence * array)
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
      crazyflie_interfaces__msg__TrajectoryPolynomialPiece__fini(&array->data[i]);
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

crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence *
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence * array = (crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence *)allocator.allocate(sizeof(crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence__destroy(crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence__are_equal(const crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence * lhs, const crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crazyflie_interfaces__msg__TrajectoryPolynomialPiece__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence__copy(
  const crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence * input,
  crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crazyflie_interfaces__msg__TrajectoryPolynomialPiece);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crazyflie_interfaces__msg__TrajectoryPolynomialPiece * data =
      (crazyflie_interfaces__msg__TrajectoryPolynomialPiece *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crazyflie_interfaces__msg__TrajectoryPolynomialPiece__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crazyflie_interfaces__msg__TrajectoryPolynomialPiece__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crazyflie_interfaces__msg__TrajectoryPolynomialPiece__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
