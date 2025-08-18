// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from crazyflie_interfaces:msg/TrajectoryPolynomialPiece.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "crazyflie_interfaces/msg/detail/trajectory_polynomial_piece__struct.hpp"
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

void TrajectoryPolynomialPiece_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) crazyflie_interfaces::msg::TrajectoryPolynomialPiece(_init);
}

void TrajectoryPolynomialPiece_fini_function(void * message_memory)
{
  auto typed_message = static_cast<crazyflie_interfaces::msg::TrajectoryPolynomialPiece *>(message_memory);
  typed_message->~TrajectoryPolynomialPiece();
}

size_t size_function__TrajectoryPolynomialPiece__poly_x(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TrajectoryPolynomialPiece__poly_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryPolynomialPiece__poly_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryPolynomialPiece__poly_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__TrajectoryPolynomialPiece__poly_x(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryPolynomialPiece__poly_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__TrajectoryPolynomialPiece__poly_x(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__TrajectoryPolynomialPiece__poly_x(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TrajectoryPolynomialPiece__poly_y(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TrajectoryPolynomialPiece__poly_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryPolynomialPiece__poly_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryPolynomialPiece__poly_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__TrajectoryPolynomialPiece__poly_y(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryPolynomialPiece__poly_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__TrajectoryPolynomialPiece__poly_y(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__TrajectoryPolynomialPiece__poly_y(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TrajectoryPolynomialPiece__poly_z(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TrajectoryPolynomialPiece__poly_z(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryPolynomialPiece__poly_z(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryPolynomialPiece__poly_z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__TrajectoryPolynomialPiece__poly_z(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryPolynomialPiece__poly_z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__TrajectoryPolynomialPiece__poly_z(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__TrajectoryPolynomialPiece__poly_z(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__TrajectoryPolynomialPiece__poly_yaw(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TrajectoryPolynomialPiece__poly_yaw(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__TrajectoryPolynomialPiece__poly_yaw(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrajectoryPolynomialPiece__poly_yaw(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__TrajectoryPolynomialPiece__poly_yaw(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__TrajectoryPolynomialPiece__poly_yaw(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__TrajectoryPolynomialPiece__poly_yaw(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__TrajectoryPolynomialPiece__poly_yaw(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TrajectoryPolynomialPiece_message_member_array[5] = {
  {
    "poly_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::TrajectoryPolynomialPiece, poly_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryPolynomialPiece__poly_x,  // size() function pointer
    get_const_function__TrajectoryPolynomialPiece__poly_x,  // get_const(index) function pointer
    get_function__TrajectoryPolynomialPiece__poly_x,  // get(index) function pointer
    fetch_function__TrajectoryPolynomialPiece__poly_x,  // fetch(index, &value) function pointer
    assign_function__TrajectoryPolynomialPiece__poly_x,  // assign(index, value) function pointer
    resize_function__TrajectoryPolynomialPiece__poly_x  // resize(index) function pointer
  },
  {
    "poly_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::TrajectoryPolynomialPiece, poly_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryPolynomialPiece__poly_y,  // size() function pointer
    get_const_function__TrajectoryPolynomialPiece__poly_y,  // get_const(index) function pointer
    get_function__TrajectoryPolynomialPiece__poly_y,  // get(index) function pointer
    fetch_function__TrajectoryPolynomialPiece__poly_y,  // fetch(index, &value) function pointer
    assign_function__TrajectoryPolynomialPiece__poly_y,  // assign(index, value) function pointer
    resize_function__TrajectoryPolynomialPiece__poly_y  // resize(index) function pointer
  },
  {
    "poly_z",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::TrajectoryPolynomialPiece, poly_z),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryPolynomialPiece__poly_z,  // size() function pointer
    get_const_function__TrajectoryPolynomialPiece__poly_z,  // get_const(index) function pointer
    get_function__TrajectoryPolynomialPiece__poly_z,  // get(index) function pointer
    fetch_function__TrajectoryPolynomialPiece__poly_z,  // fetch(index, &value) function pointer
    assign_function__TrajectoryPolynomialPiece__poly_z,  // assign(index, value) function pointer
    resize_function__TrajectoryPolynomialPiece__poly_z  // resize(index) function pointer
  },
  {
    "poly_yaw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::TrajectoryPolynomialPiece, poly_yaw),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrajectoryPolynomialPiece__poly_yaw,  // size() function pointer
    get_const_function__TrajectoryPolynomialPiece__poly_yaw,  // get_const(index) function pointer
    get_function__TrajectoryPolynomialPiece__poly_yaw,  // get(index) function pointer
    fetch_function__TrajectoryPolynomialPiece__poly_yaw,  // fetch(index, &value) function pointer
    assign_function__TrajectoryPolynomialPiece__poly_yaw,  // assign(index, value) function pointer
    resize_function__TrajectoryPolynomialPiece__poly_yaw  // resize(index) function pointer
  },
  {
    "duration",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Duration>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces::msg::TrajectoryPolynomialPiece, duration),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TrajectoryPolynomialPiece_message_members = {
  "crazyflie_interfaces::msg",  // message namespace
  "TrajectoryPolynomialPiece",  // message name
  5,  // number of fields
  sizeof(crazyflie_interfaces::msg::TrajectoryPolynomialPiece),
  TrajectoryPolynomialPiece_message_member_array,  // message members
  TrajectoryPolynomialPiece_init_function,  // function to initialize message memory (memory has to be allocated)
  TrajectoryPolynomialPiece_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TrajectoryPolynomialPiece_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TrajectoryPolynomialPiece_message_members,
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
get_message_type_support_handle<crazyflie_interfaces::msg::TrajectoryPolynomialPiece>()
{
  return &::crazyflie_interfaces::msg::rosidl_typesupport_introspection_cpp::TrajectoryPolynomialPiece_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crazyflie_interfaces, msg, TrajectoryPolynomialPiece)() {
  return &::crazyflie_interfaces::msg::rosidl_typesupport_introspection_cpp::TrajectoryPolynomialPiece_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
