// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from crazyflie_interfaces:srv/Takeoff.idl
// generated code does not contain a copyright notice
#include "crazyflie_interfaces/srv/detail/takeoff__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "crazyflie_interfaces/srv/detail/takeoff__struct.h"
#include "crazyflie_interfaces/srv/detail/takeoff__functions.h"
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

#include "builtin_interfaces/msg/detail/duration__functions.h"  // duration

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_interfaces
size_t get_serialized_size_builtin_interfaces__msg__Duration(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_interfaces
size_t max_serialized_size_builtin_interfaces__msg__Duration(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Duration)();


using _Takeoff_Request__ros_msg_type = crazyflie_interfaces__srv__Takeoff_Request;

static bool _Takeoff_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Takeoff_Request__ros_msg_type * ros_message = static_cast<const _Takeoff_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: group_mask
  {
    cdr << ros_message->group_mask;
  }

  // Field name: height
  {
    cdr << ros_message->height;
  }

  // Field name: duration
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Duration
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->duration, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _Takeoff_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Takeoff_Request__ros_msg_type * ros_message = static_cast<_Takeoff_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: group_mask
  {
    cdr >> ros_message->group_mask;
  }

  // Field name: height
  {
    cdr >> ros_message->height;
  }

  // Field name: duration
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Duration
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->duration))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_interfaces
size_t get_serialized_size_crazyflie_interfaces__srv__Takeoff_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Takeoff_Request__ros_msg_type * ros_message = static_cast<const _Takeoff_Request__ros_msg_type *>(untyped_ros_message);
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
  // field.name height
  {
    size_t item_size = sizeof(ros_message->height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name duration

  current_alignment += get_serialized_size_builtin_interfaces__msg__Duration(
    &(ros_message->duration), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _Takeoff_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_crazyflie_interfaces__srv__Takeoff_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_interfaces
size_t max_serialized_size_crazyflie_interfaces__srv__Takeoff_Request(
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
  // member: height
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: duration
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_builtin_interfaces__msg__Duration(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = crazyflie_interfaces__srv__Takeoff_Request;
    is_plain =
      (
      offsetof(DataType, duration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Takeoff_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_crazyflie_interfaces__srv__Takeoff_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Takeoff_Request = {
  "crazyflie_interfaces::srv",
  "Takeoff_Request",
  _Takeoff_Request__cdr_serialize,
  _Takeoff_Request__cdr_deserialize,
  _Takeoff_Request__get_serialized_size,
  _Takeoff_Request__max_serialized_size
};

static rosidl_message_type_support_t _Takeoff_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Takeoff_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, srv, Takeoff_Request)() {
  return &_Takeoff_Request__type_support;
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
// #include "crazyflie_interfaces/srv/detail/takeoff__struct.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/takeoff__functions.h"
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


using _Takeoff_Response__ros_msg_type = crazyflie_interfaces__srv__Takeoff_Response;

static bool _Takeoff_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Takeoff_Response__ros_msg_type * ros_message = static_cast<const _Takeoff_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _Takeoff_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Takeoff_Response__ros_msg_type * ros_message = static_cast<_Takeoff_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_interfaces
size_t get_serialized_size_crazyflie_interfaces__srv__Takeoff_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Takeoff_Response__ros_msg_type * ros_message = static_cast<const _Takeoff_Response__ros_msg_type *>(untyped_ros_message);
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

static uint32_t _Takeoff_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_crazyflie_interfaces__srv__Takeoff_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_interfaces
size_t max_serialized_size_crazyflie_interfaces__srv__Takeoff_Response(
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
    using DataType = crazyflie_interfaces__srv__Takeoff_Response;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _Takeoff_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_crazyflie_interfaces__srv__Takeoff_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Takeoff_Response = {
  "crazyflie_interfaces::srv",
  "Takeoff_Response",
  _Takeoff_Response__cdr_serialize,
  _Takeoff_Response__cdr_deserialize,
  _Takeoff_Response__get_serialized_size,
  _Takeoff_Response__max_serialized_size
};

static rosidl_message_type_support_t _Takeoff_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Takeoff_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, srv, Takeoff_Response)() {
  return &_Takeoff_Response__type_support;
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
#include "crazyflie_interfaces/srv/takeoff.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t Takeoff__callbacks = {
  "crazyflie_interfaces::srv",
  "Takeoff",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, srv, Takeoff_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, srv, Takeoff_Response)(),
};

static rosidl_service_type_support_t Takeoff__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &Takeoff__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_interfaces, srv, Takeoff)() {
  return &Takeoff__handle;
}

#if defined(__cplusplus)
}
#endif
