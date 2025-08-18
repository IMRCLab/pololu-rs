// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from crazyflie_interfaces:msg/VelocityWorld.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "crazyflie_interfaces/msg/detail/velocity_world__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace crazyflie_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void VelocityWorld_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) crazyflie_interfaces::msg::VelocityWorld(_init);
}

void VelocityWorld_fini_function(void * message_memory)
{
  auto typed_message = static_cast<crazyflie_interfaces::msg::VelocityWorld *>(message_memory);
  typed_message->~VelocityWorld();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember VelocityWorld_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::VelocityWorld, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::VelocityWorld, vel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw_rate",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::VelocityWorld, yaw_rate),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers VelocityWorld_message_members = {
  "crazyflie_interfaces::msg",  // message namespace
  "VelocityWorld",  // message name
  3,  // number of fields
  sizeof(crazyflie_interfaces::msg::VelocityWorld),
  VelocityWorld_message_member_array,  // message members
  VelocityWorld_init_function,  // function to initialize message memory (memory has to be allocated)
  VelocityWorld_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t VelocityWorld_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &VelocityWorld_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace crazyflie_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<crazyflie_interfaces::msg::VelocityWorld>()
{
  return &::crazyflie_interfaces::msg::rosidl_typesupport_introspection_cpp::VelocityWorld_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crazyflie_interfaces, msg, VelocityWorld)() {
  return &::crazyflie_interfaces::msg::rosidl_typesupport_introspection_cpp::VelocityWorld_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
