// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motion_capture_tracking_interfaces:msg/NamedPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motion_capture_tracking_interfaces/msg/detail/named_pose__rosidl_typesupport_introspection_c.h"
#include "motion_capture_tracking_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motion_capture_tracking_interfaces/msg/detail/named_pose__functions.h"
#include "motion_capture_tracking_interfaces/msg/detail/named_pose__struct.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motion_capture_tracking_interfaces__msg__NamedPose__init(message_memory);
}

void motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_fini_function(void * message_memory)
{
  motion_capture_tracking_interfaces__msg__NamedPose__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_message_member_array[2] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motion_capture_tracking_interfaces__msg__NamedPose, name),  // bytes offset in struct
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
    offsetof(motion_capture_tracking_interfaces__msg__NamedPose, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_message_members = {
  "motion_capture_tracking_interfaces__msg",  // message namespace
  "NamedPose",  // message name
  2,  // number of fields
  sizeof(motion_capture_tracking_interfaces__msg__NamedPose),
  motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_message_member_array,  // message members
  motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_init_function,  // function to initialize message memory (memory has to be allocated)
  motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_message_type_support_handle = {
  0,
  &motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motion_capture_tracking_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motion_capture_tracking_interfaces, msg, NamedPose)() {
  motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_message_type_support_handle.typesupport_identifier) {
    motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motion_capture_tracking_interfaces__msg__NamedPose__rosidl_typesupport_introspection_c__NamedPose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
