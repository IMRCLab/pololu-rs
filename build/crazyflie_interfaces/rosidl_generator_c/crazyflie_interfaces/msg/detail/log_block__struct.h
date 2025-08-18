// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:msg/LogBlock.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_BLOCK__STRUCT_H_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_BLOCK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'topic_name'
// Member 'variables'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LogBlock in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__msg__LogBlock
{
  rosidl_runtime_c__String topic_name;
  int16_t frequency;
  rosidl_runtime_c__String__Sequence variables;
} crazyflie_interfaces__msg__LogBlock;

// Struct for a sequence of crazyflie_interfaces__msg__LogBlock.
typedef struct crazyflie_interfaces__msg__LogBlock__Sequence
{
  crazyflie_interfaces__msg__LogBlock * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__msg__LogBlock__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__LOG_BLOCK__STRUCT_H_
