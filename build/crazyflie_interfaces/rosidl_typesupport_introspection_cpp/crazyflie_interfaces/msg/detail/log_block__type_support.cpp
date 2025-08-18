// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from crazyflie_interfaces:msg/LogBlock.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "crazyflie_interfaces/msg/detail/log_block__struct.hpp"
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

void LogBlock_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) crazyflie_interfaces::msg::LogBlock(_init);
}

void LogBlock_fini_function(void * message_memory)
{
  auto typed_message = static_cast<crazyflie_interfaces::msg::LogBlock *>(message_memory);
  typed_message->~LogBlock();
}

size_t size_function__LogBlock__variables(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__LogBlock__variables(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__LogBlock__variables(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void fetch_function__LogBlock__variables(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__LogBlock__variables(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__LogBlock__variables(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__LogBlock__variables(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

void resize_function__LogBlock__variables(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LogBlock_message_member_array[3] = {
  {
    "topic_name",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::LogBlock, topic_name),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "frequency",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::LogBlock, frequency),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "variables",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::LogBlock, variables),  // bytes offset in struct
    nullptr,  // default value
    size_function__LogBlock__variables,  // size() function pointer
    get_const_function__LogBlock__variables,  // get_const(index) function pointer
    get_function__LogBlock__variables,  // get(index) function pointer
    fetch_function__LogBlock__variables,  // fetch(index, &value) function pointer
    assign_function__LogBlock__variables,  // assign(index, value) function pointer
    resize_function__LogBlock__variables  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LogBlock_message_members = {
  "crazyflie_interfaces::msg",  // message namespace
  "LogBlock",  // message name
  3,  // number of fields
  sizeof(crazyflie_interfaces::msg::LogBlock),
  LogBlock_message_member_array,  // message members
  LogBlock_init_function,  // function to initialize message memory (memory has to be allocated)
  LogBlock_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LogBlock_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LogBlock_message_members,
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
get_message_type_support_handle<crazyflie_interfaces::msg::LogBlock>()
{
  return &::crazyflie_interfaces::msg::rosidl_typesupport_introspection_cpp::LogBlock_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crazyflie_interfaces, msg, LogBlock)() {
  return &::crazyflie_interfaces::msg::rosidl_typesupport_introspection_cpp::LogBlock_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
