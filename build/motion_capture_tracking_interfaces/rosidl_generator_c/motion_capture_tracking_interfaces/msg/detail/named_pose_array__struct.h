// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motion_capture_tracking_interfaces:msg/NamedPoseArray.idl
// generated code does not contain a copyright notice

#ifndef MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE_ARRAY__STRUCT_H_
#define MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'poses'
#include "motion_capture_tracking_interfaces/msg/detail/named_pose__struct.h"

/// Struct defined in msg/NamedPoseArray in the package motion_capture_tracking_interfaces.
typedef struct motion_capture_tracking_interfaces__msg__NamedPoseArray
{
  std_msgs__msg__Header header;
  motion_capture_tracking_interfaces__msg__NamedPose__Sequence poses;
} motion_capture_tracking_interfaces__msg__NamedPoseArray;

// Struct for a sequence of motion_capture_tracking_interfaces__msg__NamedPoseArray.
typedef struct motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence
{
  motion_capture_tracking_interfaces__msg__NamedPoseArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motion_capture_tracking_interfaces__msg__NamedPoseArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE_ARRAY__STRUCT_H_
