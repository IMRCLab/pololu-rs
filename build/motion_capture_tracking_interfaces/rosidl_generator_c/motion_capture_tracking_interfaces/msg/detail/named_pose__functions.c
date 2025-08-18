// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motion_capture_tracking_interfaces:msg/NamedPose.idl
// generated code does not contain a copyright notice
#include "motion_capture_tracking_interfaces/msg/detail/named_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
motion_capture_tracking_interfaces__msg__NamedPose__init(motion_capture_tracking_interfaces__msg__NamedPose * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    motion_capture_tracking_interfaces__msg__NamedPose__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    motion_capture_tracking_interfaces__msg__NamedPose__fini(msg);
    return false;
  }
  return true;
}

void
motion_capture_tracking_interfaces__msg__NamedPose__fini(motion_capture_tracking_interfaces__msg__NamedPose * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
}

bool
motion_capture_tracking_interfaces__msg__NamedPose__are_equal(const motion_capture_tracking_interfaces__msg__NamedPose * lhs, const motion_capture_tracking_interfaces__msg__NamedPose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
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
motion_capture_tracking_interfaces__msg__NamedPose__copy(
  const motion_capture_tracking_interfaces__msg__NamedPose * input,
  motion_capture_tracking_interfaces__msg__NamedPose * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

motion_capture_tracking_interfaces__msg__NamedPose *
motion_capture_tracking_interfaces__msg__NamedPose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_capture_tracking_interfaces__msg__NamedPose * msg = (motion_capture_tracking_interfaces__msg__NamedPose *)allocator.allocate(sizeof(motion_capture_tracking_interfaces__msg__NamedPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motion_capture_tracking_interfaces__msg__NamedPose));
  bool success = motion_capture_tracking_interfaces__msg__NamedPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motion_capture_tracking_interfaces__msg__NamedPose__destroy(motion_capture_tracking_interfaces__msg__NamedPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motion_capture_tracking_interfaces__msg__NamedPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motion_capture_tracking_interfaces__msg__NamedPose__Sequence__init(motion_capture_tracking_interfaces__msg__NamedPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_capture_tracking_interfaces__msg__NamedPose * data = NULL;

  if (size) {
    data = (motion_capture_tracking_interfaces__msg__NamedPose *)allocator.zero_allocate(size, sizeof(motion_capture_tracking_interfaces__msg__NamedPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motion_capture_tracking_interfaces__msg__NamedPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motion_capture_tracking_interfaces__msg__NamedPose__fini(&data[i - 1]);
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
motion_capture_tracking_interfaces__msg__NamedPose__Sequence__fini(motion_capture_tracking_interfaces__msg__NamedPose__Sequence * array)
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
      motion_capture_tracking_interfaces__msg__NamedPose__fini(&array->data[i]);
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

motion_capture_tracking_interfaces__msg__NamedPose__Sequence *
motion_capture_tracking_interfaces__msg__NamedPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_capture_tracking_interfaces__msg__NamedPose__Sequence * array = (motion_capture_tracking_interfaces__msg__NamedPose__Sequence *)allocator.allocate(sizeof(motion_capture_tracking_interfaces__msg__NamedPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motion_capture_tracking_interfaces__msg__NamedPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motion_capture_tracking_interfaces__msg__NamedPose__Sequence__destroy(motion_capture_tracking_interfaces__msg__NamedPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motion_capture_tracking_interfaces__msg__NamedPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motion_capture_tracking_interfaces__msg__NamedPose__Sequence__are_equal(const motion_capture_tracking_interfaces__msg__NamedPose__Sequence * lhs, const motion_capture_tracking_interfaces__msg__NamedPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motion_capture_tracking_interfaces__msg__NamedPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motion_capture_tracking_interfaces__msg__NamedPose__Sequence__copy(
  const motion_capture_tracking_interfaces__msg__NamedPose__Sequence * input,
  motion_capture_tracking_interfaces__msg__NamedPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motion_capture_tracking_interfaces__msg__NamedPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motion_capture_tracking_interfaces__msg__NamedPose * data =
      (motion_capture_tracking_interfaces__msg__NamedPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motion_capture_tracking_interfaces__msg__NamedPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motion_capture_tracking_interfaces__msg__NamedPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motion_capture_tracking_interfaces__msg__NamedPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
