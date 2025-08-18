// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from motion_capture_tracking_interfaces:msg/NamedPoseArray.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "motion_capture_tracking_interfaces/msg/detail/named_pose_array__struct.h"
#include "motion_capture_tracking_interfaces/msg/detail/named_pose_array__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace motion_capture_tracking_interfaces
{

namespace msg
{

namespace rosidl_typesupport_c
{

typedef struct _NamedPoseArray_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _NamedPoseArray_type_support_ids_t;

static const _NamedPoseArray_type_support_ids_t _NamedPoseArray_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _NamedPoseArray_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _NamedPoseArray_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _NamedPoseArray_type_support_symbol_names_t _NamedPoseArray_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, motion_capture_tracking_interfaces, msg, NamedPoseArray)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motion_capture_tracking_interfaces, msg, NamedPoseArray)),
  }
};

typedef struct _NamedPoseArray_type_support_data_t
{
  void * data[2];
} _NamedPoseArray_type_support_data_t;

static _NamedPoseArray_type_support_data_t _NamedPoseArray_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _NamedPoseArray_message_typesupport_map = {
  2,
  "motion_capture_tracking_interfaces",
  &_NamedPoseArray_message_typesupport_ids.typesupport_identifier[0],
  &_NamedPoseArray_message_typesupport_symbol_names.symbol_name[0],
  &_NamedPoseArray_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t NamedPoseArray_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_NamedPoseArray_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace msg

}  // namespace motion_capture_tracking_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, motion_capture_tracking_interfaces, msg, NamedPoseArray)() {
  return &::motion_capture_tracking_interfaces::msg::rosidl_typesupport_c::NamedPoseArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
