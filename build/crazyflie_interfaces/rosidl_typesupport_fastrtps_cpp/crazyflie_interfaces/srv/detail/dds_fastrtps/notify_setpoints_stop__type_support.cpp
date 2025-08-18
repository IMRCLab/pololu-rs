// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from crazyflie_interfaces:srv/NotifySetpointsStop.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/srv/detail/notify_setpoints_stop__rosidl_typesupport_fastrtps_cpp.hpp"
#include "crazyflie_interfaces/srv/detail/notify_setpoints_stop__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace crazyflie_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
cdr_serialize(
  const crazyflie_interfaces::srv::NotifySetpointsStop_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: group_mask
  cdr << ros_message.group_mask;
  // Member: remain_valid_millisecs
  cdr << ros_message.remain_valid_millisecs;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  crazyflie_interfaces::srv::NotifySetpointsStop_Request & ros_message)
{
  // Member: group_mask
  cdr >> ros_message.group_mask;

  // Member: remain_valid_millisecs
  cdr >> ros_message.remain_valid_millisecs;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
get_serialized_size(
  const crazyflie_interfaces::srv::NotifySetpointsStop_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: group_mask
  {
    size_t item_size = sizeof(ros_message.group_mask);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: remain_valid_millisecs
  {
    size_t item_size = sizeof(ros_message.remain_valid_millisecs);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
max_serialized_size_NotifySetpointsStop_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: group_mask
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: remain_valid_millisecs
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = crazyflie_interfaces::srv::NotifySetpointsStop_Request;
    is_plain =
      (
      offsetof(DataType, remain_valid_millisecs) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _NotifySetpointsStop_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const crazyflie_interfaces::srv::NotifySetpointsStop_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _NotifySetpointsStop_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<crazyflie_interfaces::srv::NotifySetpointsStop_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _NotifySetpointsStop_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const crazyflie_interfaces::srv::NotifySetpointsStop_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _NotifySetpointsStop_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_NotifySetpointsStop_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _NotifySetpointsStop_Request__callbacks = {
  "crazyflie_interfaces::srv",
  "NotifySetpointsStop_Request",
  _NotifySetpointsStop_Request__cdr_serialize,
  _NotifySetpointsStop_Request__cdr_deserialize,
  _NotifySetpointsStop_Request__get_serialized_size,
  _NotifySetpointsStop_Request__max_serialized_size
};

static rosidl_message_type_support_t _NotifySetpointsStop_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_NotifySetpointsStop_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<crazyflie_interfaces::srv::NotifySetpointsStop_Request>()
{
  return &crazyflie_interfaces::srv::typesupport_fastrtps_cpp::_NotifySetpointsStop_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, crazyflie_interfaces, srv, NotifySetpointsStop_Request)() {
  return &crazyflie_interfaces::srv::typesupport_fastrtps_cpp::_NotifySetpointsStop_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace crazyflie_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
cdr_serialize(
  const crazyflie_interfaces::srv::NotifySetpointsStop_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: structure_needs_at_least_one_member
  cdr << ros_message.structure_needs_at_least_one_member;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  crazyflie_interfaces::srv::NotifySetpointsStop_Response & ros_message)
{
  // Member: structure_needs_at_least_one_member
  cdr >> ros_message.structure_needs_at_least_one_member;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
get_serialized_size(
  const crazyflie_interfaces::srv::NotifySetpointsStop_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message.structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
max_serialized_size_NotifySetpointsStop_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: structure_needs_at_least_one_member
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = crazyflie_interfaces::srv::NotifySetpointsStop_Response;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _NotifySetpointsStop_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const crazyflie_interfaces::srv::NotifySetpointsStop_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _NotifySetpointsStop_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<crazyflie_interfaces::srv::NotifySetpointsStop_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _NotifySetpointsStop_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const crazyflie_interfaces::srv::NotifySetpointsStop_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _NotifySetpointsStop_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_NotifySetpointsStop_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _NotifySetpointsStop_Response__callbacks = {
  "crazyflie_interfaces::srv",
  "NotifySetpointsStop_Response",
  _NotifySetpointsStop_Response__cdr_serialize,
  _NotifySetpointsStop_Response__cdr_deserialize,
  _NotifySetpointsStop_Response__get_serialized_size,
  _NotifySetpointsStop_Response__max_serialized_size
};

static rosidl_message_type_support_t _NotifySetpointsStop_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_NotifySetpointsStop_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<crazyflie_interfaces::srv::NotifySetpointsStop_Response>()
{
  return &crazyflie_interfaces::srv::typesupport_fastrtps_cpp::_NotifySetpointsStop_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, crazyflie_interfaces, srv, NotifySetpointsStop_Response)() {
  return &crazyflie_interfaces::srv::typesupport_fastrtps_cpp::_NotifySetpointsStop_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace crazyflie_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _NotifySetpointsStop__callbacks = {
  "crazyflie_interfaces::srv",
  "NotifySetpointsStop",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, crazyflie_interfaces, srv, NotifySetpointsStop_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, crazyflie_interfaces, srv, NotifySetpointsStop_Response)(),
};

static rosidl_service_type_support_t _NotifySetpointsStop__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_NotifySetpointsStop__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_crazyflie_interfaces
const rosidl_service_type_support_t *
get_service_type_support_handle<crazyflie_interfaces::srv::NotifySetpointsStop>()
{
  return &crazyflie_interfaces::srv::typesupport_fastrtps_cpp::_NotifySetpointsStop__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, crazyflie_interfaces, srv, NotifySetpointsStop)() {
  return &crazyflie_interfaces::srv::typesupport_fastrtps_cpp::_NotifySetpointsStop__handle;
}

#ifdef __cplusplus
}
#endif
