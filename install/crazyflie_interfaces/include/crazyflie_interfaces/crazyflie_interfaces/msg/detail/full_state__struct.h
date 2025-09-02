// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:msg/FullState.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__FULL_STATE__STRUCT_H_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__FULL_STATE__STRUCT_H_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'acc'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/FullState in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__msg__FullState
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Twist twist;
  geometry_msgs__msg__Vector3 acc;
} crazyflie_interfaces__msg__FullState;

// Struct for a sequence of crazyflie_interfaces__msg__FullState.
typedef struct crazyflie_interfaces__msg__FullState__Sequence
{
  crazyflie_interfaces__msg__FullState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__msg__FullState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__FULL_STATE__STRUCT_H_
