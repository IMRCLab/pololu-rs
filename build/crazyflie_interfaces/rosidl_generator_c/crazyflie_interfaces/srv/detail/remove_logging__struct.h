// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:srv/RemoveLogging.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__REMOVE_LOGGING__STRUCT_H_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__REMOVE_LOGGING__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RemoveLogging in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__srv__RemoveLogging_Request
{
  rosidl_runtime_c__String topic_name;
} crazyflie_interfaces__srv__RemoveLogging_Request;

// Struct for a sequence of crazyflie_interfaces__srv__RemoveLogging_Request.
typedef struct crazyflie_interfaces__srv__RemoveLogging_Request__Sequence
{
  crazyflie_interfaces__srv__RemoveLogging_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__srv__RemoveLogging_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/RemoveLogging in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__srv__RemoveLogging_Response
{
  bool success;
} crazyflie_interfaces__srv__RemoveLogging_Response;

// Struct for a sequence of crazyflie_interfaces__srv__RemoveLogging_Response.
typedef struct crazyflie_interfaces__srv__RemoveLogging_Response__Sequence
{
  crazyflie_interfaces__srv__RemoveLogging_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__srv__RemoveLogging_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__REMOVE_LOGGING__STRUCT_H_
