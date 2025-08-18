// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crazyflie_interfaces:msg/FullState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crazyflie_interfaces/msg/detail/full_state__rosidl_typesupport_introspection_c.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crazyflie_interfaces/msg/detail/full_state__functions.h"
#include "crazyflie_interfaces/msg/detail/full_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"
// Member `twist`
#include "geometry_msgs/msg/twist.h"
// Member `twist`
#include "geometry_msgs/msg/detail/twist__rosidl_typesupport_introspection_c.h"
// Member `acc`
#include "geometry_msgs/msg/vector3.h"
// Member `acc`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__msg__FullState__init(message_memory);
}

void crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_fini_function(void * message_memory)
{
  crazyflie_interfaces__msg__FullState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__FullState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__FullState, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "twist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__FullState, twist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__FullState, acc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_members = {
  "crazyflie_interfaces__msg",  // message namespace
  "FullState",  // message name
  4,  // number of fields
  sizeof(crazyflie_interfaces__msg__FullState),
  crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_member_array,  // message members
  crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_type_support_handle = {
  0,
  &crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, msg, FullState)() {
  crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Twist)();
  crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__msg__FullState__rosidl_typesupport_introspection_c__FullState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
