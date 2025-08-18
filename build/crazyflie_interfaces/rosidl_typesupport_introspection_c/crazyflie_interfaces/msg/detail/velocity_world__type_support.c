// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crazyflie_interfaces:msg/VelocityWorld.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crazyflie_interfaces/msg/detail/velocity_world__rosidl_typesupport_introspection_c.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crazyflie_interfaces/msg/detail/velocity_world__functions.h"
#include "crazyflie_interfaces/msg/detail/velocity_world__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `vel`
#include "geometry_msgs/msg/vector3.h"
// Member `vel`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__msg__VelocityWorld__init(message_memory);
}

void crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_fini_function(void * message_memory)
{
  crazyflie_interfaces__msg__VelocityWorld__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__VelocityWorld, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__VelocityWorld, vel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__VelocityWorld, yaw_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_message_members = {
  "crazyflie_interfaces__msg",  // message namespace
  "VelocityWorld",  // message name
  3,  // number of fields
  sizeof(crazyflie_interfaces__msg__VelocityWorld),
  crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_message_member_array,  // message members
  crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_message_type_support_handle = {
  0,
  &crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, msg, VelocityWorld)() {
  crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__msg__VelocityWorld__rosidl_typesupport_introspection_c__VelocityWorld_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
