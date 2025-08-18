// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__POSITION__STRUCT_H_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__POSITION__STRUCT_H_

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

/// Struct defined in msg/Position in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__msg__Position
{
  std_msgs__msg__Header header;
  float x;
  float y;
  float z;
  float yaw;
} crazyflie_interfaces__msg__Position;

// Struct for a sequence of crazyflie_interfaces__msg__Position.
typedef struct crazyflie_interfaces__msg__Position__Sequence
{
  crazyflie_interfaces__msg__Position * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__msg__Position__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__POSITION__STRUCT_H_
