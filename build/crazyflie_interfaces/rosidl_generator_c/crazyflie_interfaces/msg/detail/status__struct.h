// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__STRUCT_H_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SUPERVISOR_INFO_CAN_BE_ARMED'.
/**
  * Constants
 */
enum
{
  crazyflie_interfaces__msg__Status__SUPERVISOR_INFO_CAN_BE_ARMED = 1
};

/// Constant 'SUPERVISOR_INFO_IS_ARMED'.
enum
{
  crazyflie_interfaces__msg__Status__SUPERVISOR_INFO_IS_ARMED = 2
};

/// Constant 'SUPERVISOR_INFO_AUTO_ARM'.
enum
{
  crazyflie_interfaces__msg__Status__SUPERVISOR_INFO_AUTO_ARM = 4
};

/// Constant 'SUPERVISOR_INFO_CAN_FLY'.
enum
{
  crazyflie_interfaces__msg__Status__SUPERVISOR_INFO_CAN_FLY = 8
};

/// Constant 'SUPERVISOR_INFO_IS_FLYING'.
enum
{
  crazyflie_interfaces__msg__Status__SUPERVISOR_INFO_IS_FLYING = 16
};

/// Constant 'SUPERVISOR_INFO_IS_TUMBLED'.
enum
{
  crazyflie_interfaces__msg__Status__SUPERVISOR_INFO_IS_TUMBLED = 32
};

/// Constant 'SUPERVISOR_INFO_IS_LOCKED'.
enum
{
  crazyflie_interfaces__msg__Status__SUPERVISOR_INFO_IS_LOCKED = 64
};

/// Constant 'PM_STATE_BATTERY'.
enum
{
  crazyflie_interfaces__msg__Status__PM_STATE_BATTERY = 0
};

/// Constant 'PM_STATE_CHARGING'.
enum
{
  crazyflie_interfaces__msg__Status__PM_STATE_CHARGING = 1
};

/// Constant 'PM_STATE_CHARGED'.
enum
{
  crazyflie_interfaces__msg__Status__PM_STATE_CHARGED = 2
};

/// Constant 'PM_STATE_LOW_POWER'.
enum
{
  crazyflie_interfaces__msg__Status__PM_STATE_LOW_POWER = 3
};

/// Constant 'PM_STATE_SHUTDOWN'.
enum
{
  crazyflie_interfaces__msg__Status__PM_STATE_SHUTDOWN = 4
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Status in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__msg__Status
{
  std_msgs__msg__Header header;
  /// general status
  /// Bitfield, see SUPERVISOR_INFO for individual bits
  uint16_t supervisor_info;
  /// battery related
  /// internal battery voltage
  float battery_voltage;
  /// See PM_STATE_* for potential values
  uint8_t pm_state;
  /// radio related
  /// latest radio signal strength indicator
  uint8_t rssi;
  /// number of received broadcast packets during the last period
  uint32_t num_rx_broadcast;
  /// number of broadcast packets sent during the last period
  uint32_t num_tx_broadcast;
  /// number of received unicast packets during the last period
  uint32_t num_rx_unicast;
  /// number of unicast packets sent during the last period
  uint32_t num_tx_unicast;
  /// latency (in ms) for unicast packets
  uint16_t latency_unicast;
} crazyflie_interfaces__msg__Status;

// Struct for a sequence of crazyflie_interfaces__msg__Status.
typedef struct crazyflie_interfaces__msg__Status__Sequence
{
  crazyflie_interfaces__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__STRUCT_H_
