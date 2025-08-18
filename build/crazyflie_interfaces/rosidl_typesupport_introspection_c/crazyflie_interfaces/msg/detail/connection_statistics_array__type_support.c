// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crazyflie_interfaces:msg/ConnectionStatisticsArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crazyflie_interfaces/msg/detail/connection_statistics_array__rosidl_typesupport_introspection_c.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crazyflie_interfaces/msg/detail/connection_statistics_array__functions.h"
#include "crazyflie_interfaces/msg/detail/connection_statistics_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `stats`
#include "crazyflie_interfaces/msg/connection_statistics.h"
// Member `stats`
#include "crazyflie_interfaces/msg/detail/connection_statistics__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__msg__ConnectionStatisticsArray__init(message_memory);
}

void crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_fini_function(void * message_memory)
{
  crazyflie_interfaces__msg__ConnectionStatisticsArray__fini(message_memory);
}

size_t crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__size_function__ConnectionStatisticsArray__stats(
  const void * untyped_member)
{
  const crazyflie_interfaces__msg__ConnectionStatistics__Sequence * member =
    (const crazyflie_interfaces__msg__ConnectionStatistics__Sequence *)(untyped_member);
  return member->size;
}

const void * crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__get_const_function__ConnectionStatisticsArray__stats(
  const void * untyped_member, size_t index)
{
  const crazyflie_interfaces__msg__ConnectionStatistics__Sequence * member =
    (const crazyflie_interfaces__msg__ConnectionStatistics__Sequence *)(untyped_member);
  return &member->data[index];
}

void * crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__get_function__ConnectionStatisticsArray__stats(
  void * untyped_member, size_t index)
{
  crazyflie_interfaces__msg__ConnectionStatistics__Sequence * member =
    (crazyflie_interfaces__msg__ConnectionStatistics__Sequence *)(untyped_member);
  return &member->data[index];
}

void crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__fetch_function__ConnectionStatisticsArray__stats(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const crazyflie_interfaces__msg__ConnectionStatistics * item =
    ((const crazyflie_interfaces__msg__ConnectionStatistics *)
    crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__get_const_function__ConnectionStatisticsArray__stats(untyped_member, index));
  crazyflie_interfaces__msg__ConnectionStatistics * value =
    (crazyflie_interfaces__msg__ConnectionStatistics *)(untyped_value);
  *value = *item;
}

void crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__assign_function__ConnectionStatisticsArray__stats(
  void * untyped_member, size_t index, const void * untyped_value)
{
  crazyflie_interfaces__msg__ConnectionStatistics * item =
    ((crazyflie_interfaces__msg__ConnectionStatistics *)
    crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__get_function__ConnectionStatisticsArray__stats(untyped_member, index));
  const crazyflie_interfaces__msg__ConnectionStatistics * value =
    (const crazyflie_interfaces__msg__ConnectionStatistics *)(untyped_value);
  *item = *value;
}

bool crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__resize_function__ConnectionStatisticsArray__stats(
  void * untyped_member, size_t size)
{
  crazyflie_interfaces__msg__ConnectionStatistics__Sequence * member =
    (crazyflie_interfaces__msg__ConnectionStatistics__Sequence *)(untyped_member);
  crazyflie_interfaces__msg__ConnectionStatistics__Sequence__fini(member);
  return crazyflie_interfaces__msg__ConnectionStatistics__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__ConnectionStatisticsArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stats",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__ConnectionStatisticsArray, stats),  // bytes offset in struct
    NULL,  // default value
    crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__size_function__ConnectionStatisticsArray__stats,  // size() function pointer
    crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__get_const_function__ConnectionStatisticsArray__stats,  // get_const(index) function pointer
    crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__get_function__ConnectionStatisticsArray__stats,  // get(index) function pointer
    crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__fetch_function__ConnectionStatisticsArray__stats,  // fetch(index, &value) function pointer
    crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__assign_function__ConnectionStatisticsArray__stats,  // assign(index, value) function pointer
    crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__resize_function__ConnectionStatisticsArray__stats  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_message_members = {
  "crazyflie_interfaces__msg",  // message namespace
  "ConnectionStatisticsArray",  // message name
  2,  // number of fields
  sizeof(crazyflie_interfaces__msg__ConnectionStatisticsArray),
  crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_message_member_array,  // message members
  crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_message_type_support_handle = {
  0,
  &crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, msg, ConnectionStatisticsArray)() {
  crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, msg, ConnectionStatistics)();
  if (!crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__msg__ConnectionStatisticsArray__rosidl_typesupport_introspection_c__ConnectionStatisticsArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
