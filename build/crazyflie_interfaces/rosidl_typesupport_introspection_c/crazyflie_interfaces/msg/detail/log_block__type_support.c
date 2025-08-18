// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crazyflie_interfaces:msg/LogBlock.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crazyflie_interfaces/msg/detail/log_block__rosidl_typesupport_introspection_c.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crazyflie_interfaces/msg/detail/log_block__functions.h"
#include "crazyflie_interfaces/msg/detail/log_block__struct.h"


// Include directives for member types
// Member `topic_name`
// Member `variables`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__msg__LogBlock__init(message_memory);
}

void crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_fini_function(void * message_memory)
{
  crazyflie_interfaces__msg__LogBlock__fini(message_memory);
}

size_t crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__size_function__LogBlock__variables(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__get_const_function__LogBlock__variables(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__get_function__LogBlock__variables(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__fetch_function__LogBlock__variables(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__get_const_function__LogBlock__variables(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__assign_function__LogBlock__variables(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__get_function__LogBlock__variables(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__resize_function__LogBlock__variables(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_message_member_array[3] = {
  {
    "topic_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__LogBlock, topic_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frequency",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__LogBlock, frequency),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "variables",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__msg__LogBlock, variables),  // bytes offset in struct
    NULL,  // default value
    crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__size_function__LogBlock__variables,  // size() function pointer
    crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__get_const_function__LogBlock__variables,  // get_const(index) function pointer
    crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__get_function__LogBlock__variables,  // get(index) function pointer
    crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__fetch_function__LogBlock__variables,  // fetch(index, &value) function pointer
    crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__assign_function__LogBlock__variables,  // assign(index, value) function pointer
    crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__resize_function__LogBlock__variables  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_message_members = {
  "crazyflie_interfaces__msg",  // message namespace
  "LogBlock",  // message name
  3,  // number of fields
  sizeof(crazyflie_interfaces__msg__LogBlock),
  crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_message_member_array,  // message members
  crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_message_type_support_handle = {
  0,
  &crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, msg, LogBlock)() {
  if (!crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__msg__LogBlock__rosidl_typesupport_introspection_c__LogBlock_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
