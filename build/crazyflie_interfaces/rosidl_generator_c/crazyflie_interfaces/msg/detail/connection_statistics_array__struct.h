// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:msg/ConnectionStatisticsArray.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS_ARRAY__STRUCT_H_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS_ARRAY__STRUCT_H_

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
// Member 'stats'
#include "crazyflie_interfaces/msg/detail/connection_statistics__struct.h"

/// Struct defined in msg/ConnectionStatisticsArray in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__msg__ConnectionStatisticsArray
{
  std_msgs__msg__Header header;
  crazyflie_interfaces__msg__ConnectionStatistics__Sequence stats;
} crazyflie_interfaces__msg__ConnectionStatisticsArray;

// Struct for a sequence of crazyflie_interfaces__msg__ConnectionStatisticsArray.
typedef struct crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence
{
  crazyflie_interfaces__msg__ConnectionStatisticsArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__msg__ConnectionStatisticsArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS_ARRAY__STRUCT_H_
