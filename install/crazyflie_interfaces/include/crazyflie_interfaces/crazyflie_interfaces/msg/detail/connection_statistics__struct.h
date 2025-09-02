// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:msg/ConnectionStatistics.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__STRUCT_H_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'uri'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ConnectionStatistics in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__msg__ConnectionStatistics
{
  rosidl_runtime_c__String uri;
  uint64_t sent_count;
  uint64_t sent_ping_count;
  uint64_t receive_count;
  uint64_t enqueued_count;
  uint64_t ack_count;
} crazyflie_interfaces__msg__ConnectionStatistics;

// Struct for a sequence of crazyflie_interfaces__msg__ConnectionStatistics.
typedef struct crazyflie_interfaces__msg__ConnectionStatistics__Sequence
{
  crazyflie_interfaces__msg__ConnectionStatistics * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__msg__ConnectionStatistics__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__STRUCT_H_
