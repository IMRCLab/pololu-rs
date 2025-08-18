// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from motion_capture_tracking_interfaces:msg/NamedPose.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "motion_capture_tracking_interfaces/msg/detail/named_pose__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace motion_capture_tracking_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void NamedPose_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) motion_capture_tracking_interfaces::msg::NamedPose(_init);
}

void NamedPose_fini_function(void * message_memory)
{
  auto typed_message = static_cast<motion_capture_tracking_interfaces::msg::NamedPose *>(message_memory);
  typed_message->~NamedPose();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember NamedPose_message_member_array[2] = {
  {
    "name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motion_capture_tracking_interfaces::msg::NamedPose, name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motion_capture_tracking_interfaces::msg::NamedPose, pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers NamedPose_message_members = {
  "motion_capture_tracking_interfaces::msg",  // message namespace
  "NamedPose",  // message name
  2,  // number of fields
  sizeof(motion_capture_tracking_interfaces::msg::NamedPose),
  NamedPose_message_member_array,  // message members
  NamedPose_init_function,  // function to initialize message memory (memory has to be allocated)
  NamedPose_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t NamedPose_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &NamedPose_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace motion_capture_tracking_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<motion_capture_tracking_interfaces::msg::NamedPose>()
{
  return &::motion_capture_tracking_interfaces::msg::rosidl_typesupport_introspection_cpp::NamedPose_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, motion_capture_tracking_interfaces, msg, NamedPose)() {
  return &::motion_capture_tracking_interfaces::msg::rosidl_typesupport_introspection_cpp::NamedPose_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
