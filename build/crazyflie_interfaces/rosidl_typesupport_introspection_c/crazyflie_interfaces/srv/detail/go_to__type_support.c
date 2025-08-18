// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crazyflie_interfaces:srv/GoTo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crazyflie_interfaces/srv/detail/go_to__rosidl_typesupport_introspection_c.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crazyflie_interfaces/srv/detail/go_to__functions.h"
#include "crazyflie_interfaces/srv/detail/go_to__struct.h"


// Include directives for member types
// Member `goal`
#include "geometry_msgs/msg/point.h"
// Member `goal`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `duration`
#include "builtin_interfaces/msg/duration.h"
// Member `duration`
#include "builtin_interfaces/msg/detail/duration__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__srv__GoTo_Request__init(message_memory);
}

void crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_fini_function(void * message_memory)
{
  crazyflie_interfaces__srv__GoTo_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_message_member_array[5] = {
  {
    "group_mask",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__GoTo_Request, group_mask),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "relative",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__GoTo_Request, relative),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__GoTo_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__GoTo_Request, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "duration",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__GoTo_Request, duration),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_message_members = {
  "crazyflie_interfaces__srv",  // message namespace
  "GoTo_Request",  // message name
  5,  // number of fields
  sizeof(crazyflie_interfaces__srv__GoTo_Request),
  crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_message_member_array,  // message members
  crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_message_type_support_handle = {
  0,
  &crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, GoTo_Request)() {
  crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Duration)();
  if (!crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__srv__GoTo_Request__rosidl_typesupport_introspection_c__GoTo_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "crazyflie_interfaces/srv/detail/go_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/go_to__functions.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/go_to__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__srv__GoTo_Response__init(message_memory);
}

void crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_fini_function(void * message_memory)
{
  crazyflie_interfaces__srv__GoTo_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__GoTo_Response, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_message_members = {
  "crazyflie_interfaces__srv",  // message namespace
  "GoTo_Response",  // message name
  1,  // number of fields
  sizeof(crazyflie_interfaces__srv__GoTo_Response),
  crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_message_member_array,  // message members
  crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_message_type_support_handle = {
  0,
  &crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, GoTo_Response)() {
  if (!crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__srv__GoTo_Response__rosidl_typesupport_introspection_c__GoTo_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/go_to__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers crazyflie_interfaces__srv__detail__go_to__rosidl_typesupport_introspection_c__GoTo_service_members = {
  "crazyflie_interfaces__srv",  // service namespace
  "GoTo",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // crazyflie_interfaces__srv__detail__go_to__rosidl_typesupport_introspection_c__GoTo_Request_message_type_support_handle,
  NULL  // response message
  // crazyflie_interfaces__srv__detail__go_to__rosidl_typesupport_introspection_c__GoTo_Response_message_type_support_handle
};

static rosidl_service_type_support_t crazyflie_interfaces__srv__detail__go_to__rosidl_typesupport_introspection_c__GoTo_service_type_support_handle = {
  0,
  &crazyflie_interfaces__srv__detail__go_to__rosidl_typesupport_introspection_c__GoTo_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, GoTo_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, GoTo_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, GoTo)() {
  if (!crazyflie_interfaces__srv__detail__go_to__rosidl_typesupport_introspection_c__GoTo_service_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__srv__detail__go_to__rosidl_typesupport_introspection_c__GoTo_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)crazyflie_interfaces__srv__detail__go_to__rosidl_typesupport_introspection_c__GoTo_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, GoTo_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, GoTo_Response)()->data;
  }

  return &crazyflie_interfaces__srv__detail__go_to__rosidl_typesupport_introspection_c__GoTo_service_type_support_handle;
}
