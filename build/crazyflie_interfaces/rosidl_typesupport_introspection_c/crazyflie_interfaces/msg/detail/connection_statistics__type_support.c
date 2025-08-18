// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crazyflie_interfaces:msg/ConnectionStatistics.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crazyflie_interfaces/msg/detail/connection_statistics__rosidl_typesupport_introspection_c.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crazyflie_interfaces/msg/detail/connection_statistics__functions.h"
#include "crazyflie_interfaces/msg/detail/connection_statistics__struct.h"


// Include directives for member types
// Member `uri`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__msg__ConnectionStatistics__init(message_memory);
}

void crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_fini_function(void * message_memory)
{
  crazyflie_interfaces__msg__ConnectionStatistics__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_message_member_array[6] = {
  {
    "uri",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__ConnectionStatistics, uri),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sent_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__ConnectionStatistics, sent_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sent_ping_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__ConnectionStatistics, sent_ping_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "receive_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__ConnectionStatistics, receive_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enqueued_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__ConnectionStatistics, enqueued_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ack_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__ConnectionStatistics, ack_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_message_members = {
  "crazyflie_interfaces__msg",  // message namespace
  "ConnectionStatistics",  // message name
  6,  // number of fields
  sizeof(crazyflie_interfaces__msg__ConnectionStatistics),
  crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_message_member_array,  // message members
  crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_message_type_support_handle = {
  0,
  &crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, msg, ConnectionStatistics)() {
  if (!crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__msg__ConnectionStatistics__rosidl_typesupport_introspection_c__ConnectionStatistics_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
