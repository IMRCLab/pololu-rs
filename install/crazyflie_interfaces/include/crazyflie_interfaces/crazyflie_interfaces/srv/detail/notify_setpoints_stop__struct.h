// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:srv/NotifySetpointsStop.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__NOTIFY_SETPOINTS_STOP__STRUCT_H_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__NOTIFY_SETPOINTS_STOP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/NotifySetpointsStop in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__srv__NotifySetpointsStop_Request
{
  uint8_t group_mask;
  uint32_t remain_valid_millisecs;
} crazyflie_interfaces__srv__NotifySetpointsStop_Request;

// Struct for a sequence of crazyflie_interfaces__srv__NotifySetpointsStop_Request.
typedef struct crazyflie_interfaces__srv__NotifySetpointsStop_Request__Sequence
{
  crazyflie_interfaces__srv__NotifySetpointsStop_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__srv__NotifySetpointsStop_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/NotifySetpointsStop in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__srv__NotifySetpointsStop_Response
{
  uint8_t structure_needs_at_least_one_member;
} crazyflie_interfaces__srv__NotifySetpointsStop_Response;

// Struct for a sequence of crazyflie_interfaces__srv__NotifySetpointsStop_Response.
typedef struct crazyflie_interfaces__srv__NotifySetpointsStop_Response__Sequence
{
  crazyflie_interfaces__srv__NotifySetpointsStop_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__srv__NotifySetpointsStop_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__NOTIFY_SETPOINTS_STOP__STRUCT_H_
