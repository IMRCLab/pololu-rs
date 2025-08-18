// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motion_capture_tracking_interfaces:msg/NamedPoseArray.idl
// generated code does not contain a copyright notice
#include "motion_capture_tracking_interfaces/msg/detail/named_pose_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `poses`
#include "motion_capture_tracking_interfaces/msg/detail/named_pose__functions.h"

bool
motion_capture_tracking_interfaces__msg__NamedPoseArray__init(motion_capture_tracking_interfaces__msg__NamedPoseArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    motion_capture_tracking_interfaces__msg__NamedPoseArray__fini(msg);
    return false;
  }
  // poses
  if (!motion_capture_tracking_interfaces__msg__NamedPose__Sequence__init(&msg->poses, 0)) {
    motion_capture_tracking_interfaces__msg__NamedPoseArray__fini(msg);
    return false;
  }
  return true;
}

void
motion_capture_tracking_interfaces__msg__NamedPoseArray__fini(motion_capture_tracking_interfaces__msg__NamedPoseArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // poses
  motion_capture_tracking_interfaces__msg__NamedPose__Sequence__fini(&msg->poses);
}

bool
motion_capture_tracking_interfaces__msg__NamedPoseArray__are_equal(const motion_capture_tracking_interfaces__msg__NamedPoseArray * lhs, const motion_capture_tracking_interfaces__msg__NamedPoseArray * rhs)
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
  // poses
  if (!motion_capture_tracking_interfaces__msg__NamedPose__Sequence__are_equal(
      &(lhs->poses), &(rhs->poses)))
  {
    return false;
  }
  return true;
}

bool
motion_capture_tracking_interfaces__msg__NamedPoseArray__copy(
  const motion_capture_tracking_interfaces__msg__NamedPoseArray * input,
  motion_capture_tracking_interfaces__msg__NamedPoseArray * output)
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
  // poses
  if (!motion_capture_tracking_interfaces__msg__NamedPose__Sequence__copy(
      &(input->poses), &(output->poses)))
  {
    return false;
  }
  return true;
}

motion_capture_tracking_interfaces__msg__NamedPoseArray *
motion_capture_tracking_interfaces__msg__NamedPoseArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_capture_tracking_interfaces__msg__NamedPoseArray * msg = (motion_capture_tracking_interfaces__msg__NamedPoseArray *)allocator.allocate(sizeof(motion_capture_tracking_interfaces__msg__NamedPoseArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motion_capture_tracking_interfaces__msg__NamedPoseArray));
  bool success = motion_capture_tracking_interfaces__msg__NamedPoseArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motion_capture_tracking_interfaces__msg__NamedPoseArray__destroy(motion_capture_tracking_interfaces__msg__NamedPoseArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motion_capture_tracking_interfaces__msg__NamedPoseArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence__init(motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_capture_tracking_interfaces__msg__NamedPoseArray * data = NULL;

  if (size) {
    data = (motion_capture_tracking_interfaces__msg__NamedPoseArray *)allocator.zero_allocate(size, sizeof(motion_capture_tracking_interfaces__msg__NamedPoseArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motion_capture_tracking_interfaces__msg__NamedPoseArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motion_capture_tracking_interfaces__msg__NamedPoseArray__fini(&data[i - 1]);
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
motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence__fini(motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence * array)
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
      motion_capture_tracking_interfaces__msg__NamedPoseArray__fini(&array->data[i]);
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

motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence *
motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence * array = (motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence *)allocator.allocate(sizeof(motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence__destroy(motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence__are_equal(const motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence * lhs, const motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motion_capture_tracking_interfaces__msg__NamedPoseArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence__copy(
  const motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence * input,
  motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motion_capture_tracking_interfaces__msg__NamedPoseArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motion_capture_tracking_interfaces__msg__NamedPoseArray * data =
      (motion_capture_tracking_interfaces__msg__NamedPoseArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motion_capture_tracking_interfaces__msg__NamedPoseArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motion_capture_tracking_interfaces__msg__NamedPoseArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motion_capture_tracking_interfaces__msg__NamedPoseArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
