// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:msg/Hover.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__HOVER__STRUCT_H_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__HOVER__STRUCT_H_

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

/// Struct defined in msg/Hover in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__msg__Hover
{
  std_msgs__msg__Header header;
  float vx;
  float vy;
  float yaw_rate;
  float z_distance;
} crazyflie_interfaces__msg__Hover;

// Struct for a sequence of crazyflie_interfaces__msg__Hover.
typedef struct crazyflie_interfaces__msg__Hover__Sequence
{
  crazyflie_interfaces__msg__Hover * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__msg__Hover__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__HOVER__STRUCT_H_
