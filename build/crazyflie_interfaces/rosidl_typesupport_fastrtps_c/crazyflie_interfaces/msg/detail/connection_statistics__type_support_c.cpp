// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from crazyflie_interfaces:msg/ConnectionStatistics.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/msg/detail/connection_statistics__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "crazyflie_interfaces/msg/detail/connection_statistics__struct.h"
#include "crazyflie_interfaces/msg/detail/connection_statistics__functions.h"
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

#include "rosidl_runtime_c/string.h"  // uri
#include "rosidl_runtime_c/string_functions.h"  // uri

// forward declare type support functions


using _ConnectionStatistics__ros_msg_type = crazyflie_interfaces__msg__ConnectionStatistics;

static bool _ConnectionStatistics__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ConnectionStatistics__ros_msg_type * ros_message = static_cast<const _ConnectionStatistics__ros_msg_type *>(untyped_ros_message);
  // Field name: uri
  {
    const rosidl_runtime_c__String * str = &ros_message->uri;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: sent_count
  {
    cdr << ros_message->sent_count;
  }

  // Field name: sent_ping_count
  {
    cdr << ros_message->sent_ping_count;
  }

  // Field name: receive_count
  {
    cdr << ros_message->receive_count;
  }

  // Field name: enqueued_count
  {
    cdr << ros_message->enqueued_count;
  }

  // Field name: ack_count
  {
    cdr << ros_message->ack_count;
  }

  return true;
}

static bool _ConnectionStatistics__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ConnectionStatistics__ros_msg_type * ros_message = static_cast<_ConnectionStatistics__ros_msg_type *>(untyped_ros_message);
  // Field name: uri
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->uri.data) {
      rosidl_runtime_c__String__init(&ros_message->uri);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->uri,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'uri'\n");
      return false;
    }
  }

  // Field name: sent_count
  {
    cdr >> ros_message->sent_count;
  }

  // Field name: sent_ping_count
  {
    cdr >> ros_message->sent_ping_count;
  }

  // Field name: receive_count
  {
    cdr >> ros_message->receive_count;
  }

  // Field name: enqueued_count
  {
    cdr >> ros_message->enqueued_count;
  }

  // Field name: ack_count
  {
    cdr >> ros_message->ack_count;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_interfaces
size_t get_serialized_size_crazyflie_interfaces__msg__ConnectionStatistics(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ConnectionStatistics__ros_msg_type * ros_message = static_cast<const _ConnectionStatistics__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name uri
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->uri.size + 1);
  // field.name sent_count
  {
    size_t item_size = sizeof(ros_message->sent_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sent_ping_count
  {
    size_t item_size = sizeof(ros_message->sent_ping_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name receive_count
  {
    size_t item_size = sizeof(ros_message->receive_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name enqueued_count
  {
    size_t item_size = sizeof(ros_message->enqueued_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ack_count
  {
    size_t item_size = sizeof(ros_message->ack_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ConnectionStatistics__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_crazyflie_interfaces__msg__ConnectionStatistics(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_interfaces
size_t max_serialized_size_crazyflie_interfaces__msg__ConnectionStatistics(
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

  // member: uri
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: sent_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: sent_ping_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: receive_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: enqueued_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ack_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = crazyflie_interfaces__msg__ConnectionStatistics;
    is_plain =
      (
      offsetof(DataType, ack_count) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ConnectionStatistics__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_crazyflie_interfaces__msg__ConnectionStatistics(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ConnectionStatistics = {
  "crazyflie_interfaces::msg",
  "ConnectionStatistics",
  _ConnectionStatistics__cdr_serialize,
  _ConnectionStatistics__cdr_deserialize,
  _ConnectionStatistics__get_serialized_size,
  _ConnectionStatistics__max_serialized_size
};

static rosidl_message_type_support_t _ConnectionStatistics__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ConnectionStatistics,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, msg, ConnectionStatistics)() {
  return &_ConnectionStatistics__type_support;
}

#if defined(__cplusplus)
}
#endif
