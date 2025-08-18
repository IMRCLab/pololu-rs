// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from crazyflie_interfaces:msg/ConnectionStatistics.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/msg/detail/connection_statistics__rosidl_typesupport_fastrtps_cpp.hpp"
#include "crazyflie_interfaces/msg/detail/connection_statistics__struct.hpp"

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

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
cdr_serialize(
  const crazyflie_interfaces::msg::ConnectionStatistics & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: uri
  cdr << ros_message.uri;
  // Member: sent_count
  cdr << ros_message.sent_count;
  // Member: sent_ping_count
  cdr << ros_message.sent_ping_count;
  // Member: receive_count
  cdr << ros_message.receive_count;
  // Member: enqueued_count
  cdr << ros_message.enqueued_count;
  // Member: ack_count
  cdr << ros_message.ack_count;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  crazyflie_interfaces::msg::ConnectionStatistics & ros_message)
{
  // Member: uri
  cdr >> ros_message.uri;

  // Member: sent_count
  cdr >> ros_message.sent_count;

  // Member: sent_ping_count
  cdr >> ros_message.sent_ping_count;

  // Member: receive_count
  cdr >> ros_message.receive_count;

  // Member: enqueued_count
  cdr >> ros_message.enqueued_count;

  // Member: ack_count
  cdr >> ros_message.ack_count;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
get_serialized_size(
  const crazyflie_interfaces::msg::ConnectionStatistics & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: uri
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.uri.size() + 1);
  // Member: sent_count
  {
    size_t item_size = sizeof(ros_message.sent_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sent_ping_count
  {
    size_t item_size = sizeof(ros_message.sent_ping_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: receive_count
  {
    size_t item_size = sizeof(ros_message.receive_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: enqueued_count
  {
    size_t item_size = sizeof(ros_message.enqueued_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ack_count
  {
    size_t item_size = sizeof(ros_message.ack_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_interfaces
max_serialized_size_ConnectionStatistics(
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


  // Member: uri
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

  // Member: sent_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: sent_ping_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: receive_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: enqueued_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ack_count
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
    using DataType = crazyflie_interfaces::msg::ConnectionStatistics;
    is_plain =
      (
      offsetof(DataType, ack_count) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ConnectionStatistics__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const crazyflie_interfaces::msg::ConnectionStatistics *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ConnectionStatistics__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<crazyflie_interfaces::msg::ConnectionStatistics *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ConnectionStatistics__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const crazyflie_interfaces::msg::ConnectionStatistics *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ConnectionStatistics__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ConnectionStatistics(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ConnectionStatistics__callbacks = {
  "crazyflie_interfaces::msg",
  "ConnectionStatistics",
  _ConnectionStatistics__cdr_serialize,
  _ConnectionStatistics__cdr_deserialize,
  _ConnectionStatistics__get_serialized_size,
  _ConnectionStatistics__max_serialized_size
};

static rosidl_message_type_support_t _ConnectionStatistics__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ConnectionStatistics__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace crazyflie_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<crazyflie_interfaces::msg::ConnectionStatistics>()
{
  return &crazyflie_interfaces::msg::typesupport_fastrtps_cpp::_ConnectionStatistics__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, crazyflie_interfaces, msg, ConnectionStatistics)() {
  return &crazyflie_interfaces::msg::typesupport_fastrtps_cpp::_ConnectionStatistics__handle;
}

#ifdef __cplusplus
}
#endif
