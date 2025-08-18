// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from crazyflie_interfaces:srv/NotifySetpointsStop.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/srv/detail/notify_setpoints_stop__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "crazyflie_interfaces/srv/detail/notify_setpoints_stop__struct.h"
#include "crazyflie_interfaces/srv/detail/notify_setpoints_stop__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _NotifySetpointsStop_Request__ros_msg_type = crazyflie_interfaces__srv__NotifySetpointsStop_Request;

static bool _NotifySetpointsStop_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _NotifySetpointsStop_Request__ros_msg_type * ros_message = static_cast<const _NotifySetpointsStop_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: group_mask
  {
    cdr << ros_message->group_mask;
  }

  // Field name: remain_valid_millisecs
  {
    cdr << ros_message->remain_valid_millisecs;
  }

  return true;
}

static bool _NotifySetpointsStop_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _NotifySetpointsStop_Request__ros_msg_type * ros_message = static_cast<_NotifySetpointsStop_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: group_mask
  {
    cdr >> ros_message->group_mask;
  }

  // Field name: remain_valid_millisecs
  {
    cdr >> ros_message->remain_valid_millisecs;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_interfaces
size_t get_serialized_size_crazyflie_interfaces__srv__NotifySetpointsStop_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _NotifySetpointsStop_Request__ros_msg_type * ros_message = static_cast<const _NotifySetpointsStop_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name group_mask
  {
    size_t item_size = sizeof(ros_message->group_mask);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name remain_valid_millisecs
  {
    size_t item_size = sizeof(ros_message->remain_valid_millisecs);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _NotifySetpointsStop_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_crazyflie_interfaces__srv__NotifySetpointsStop_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_interfaces
size_t max_serialized_size_crazyflie_interfaces__srv__NotifySetpointsStop_Request(
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

  // member: group_mask
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: remain_valid_millisecs
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
    using DataType = crazyflie_interfaces__srv__NotifySetpointsStop_Request;
    is_plain =
      (
      offsetof(DataType, remain_valid_millisecs) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _NotifySetpointsStop_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_crazyflie_interfaces__srv__NotifySetpointsStop_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_NotifySetpointsStop_Request = {
  "crazyflie_interfaces::srv",
  "NotifySetpointsStop_Request",
  _NotifySetpointsStop_Request__cdr_serialize,
  _NotifySetpointsStop_Request__cdr_deserialize,
  _NotifySetpointsStop_Request__get_serialized_size,
  _NotifySetpointsStop_Request__max_serialized_size
};

static rosidl_message_type_support_t _NotifySetpointsStop_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_NotifySetpointsStop_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, srv, NotifySetpointsStop_Request)() {
  return &_NotifySetpointsStop_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "crazyflie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/notify_setpoints_stop__struct.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/notify_setpoints_stop__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _NotifySetpointsStop_Response__ros_msg_type = crazyflie_interfaces__srv__NotifySetpointsStop_Response;

static bool _NotifySetpointsStop_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _NotifySetpointsStop_Response__ros_msg_type * ros_message = static_cast<const _NotifySetpointsStop_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _NotifySetpointsStop_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _NotifySetpointsStop_Response__ros_msg_type * ros_message = static_cast<_NotifySetpointsStop_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_interfaces
size_t get_serialized_size_crazyflie_interfaces__srv__NotifySetpointsStop_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _NotifySetpointsStop_Response__ros_msg_type * ros_message = static_cast<const _NotifySetpointsStop_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _NotifySetpointsStop_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_crazyflie_interfaces__srv__NotifySetpointsStop_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_interfaces
size_t max_serialized_size_crazyflie_interfaces__srv__NotifySetpointsStop_Response(
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

  // member: structure_needs_at_least_one_member
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
    using DataType = crazyflie_interfaces__srv__NotifySetpointsStop_Response;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _NotifySetpointsStop_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_crazyflie_interfaces__srv__NotifySetpointsStop_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_NotifySetpointsStop_Response = {
  "crazyflie_interfaces::srv",
  "NotifySetpointsStop_Response",
  _NotifySetpointsStop_Response__cdr_serialize,
  _NotifySetpointsStop_Response__cdr_deserialize,
  _NotifySetpointsStop_Response__get_serialized_size,
  _NotifySetpointsStop_Response__max_serialized_size
};

static rosidl_message_type_support_t _NotifySetpointsStop_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_NotifySetpointsStop_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, srv, NotifySetpointsStop_Response)() {
  return &_NotifySetpointsStop_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "crazyflie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t NotifySetpointsStop__callbacks = {
  "crazyflie_interfaces::srv",
  "NotifySetpointsStop",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, srv, NotifySetpointsStop_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, srv, NotifySetpointsStop_Response)(),
};

static rosidl_service_type_support_t NotifySetpointsStop__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &NotifySetpointsStop__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, srv, NotifySetpointsStop)() {
  return &NotifySetpointsStop__handle;
}

#if defined(__cplusplus)
}
#endif
