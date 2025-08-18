// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:msg/LogDataGeneric.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_DATA_GENERIC__STRUCT_H_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_DATA_GENERIC__STRUCT_H_

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
// Member 'values'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/LogDataGeneric in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__msg__LogDataGeneric
{
  /// Header including the ROS 2 timestamp when the log data was received
  std_msgs__msg__Header header;
  /// on-board timestamp from the STM32 (in ms)
  uint32_t timestamp;
  /// converted values, in the order as specified for the log block
  rosidl_runtime_c__float__Sequence values;
} crazyflie_interfaces__msg__LogDataGeneric;

// Struct for a sequence of crazyflie_interfaces__msg__LogDataGeneric.
typedef struct crazyflie_interfaces__msg__LogDataGeneric__Sequence
{
  crazyflie_interfaces__msg__LogDataGeneric * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__msg__LogDataGeneric__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_DATA_GENERIC__STRUCT_H_
