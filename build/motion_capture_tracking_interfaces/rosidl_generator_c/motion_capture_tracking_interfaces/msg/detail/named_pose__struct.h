// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motion_capture_tracking_interfaces:msg/NamedPose.idl
// generated code does not contain a copyright notice

#ifndef MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE__STRUCT_H_
#define MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/NamedPose in the package motion_capture_tracking_interfaces.
typedef struct motion_capture_tracking_interfaces__msg__NamedPose
{
  rosidl_runtime_c__String name;
  geometry_msgs__msg__Pose pose;
} motion_capture_tracking_interfaces__msg__NamedPose;

// Struct for a sequence of motion_capture_tracking_interfaces__msg__NamedPose.
typedef struct motion_capture_tracking_interfaces__msg__NamedPose__Sequence
{
  motion_capture_tracking_interfaces__msg__NamedPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motion_capture_tracking_interfaces__msg__NamedPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE__STRUCT_H_
