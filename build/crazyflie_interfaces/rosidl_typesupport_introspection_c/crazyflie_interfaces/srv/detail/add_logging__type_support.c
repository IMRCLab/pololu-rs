// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crazyflie_interfaces:srv/AddLogging.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crazyflie_interfaces/srv/detail/add_logging__rosidl_typesupport_introspection_c.h"
#include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crazyflie_interfaces/srv/detail/add_logging__functions.h"
#include "crazyflie_interfaces/srv/detail/add_logging__struct.h"


// Include directives for member types
// Member `topic_name`
// Member `vars`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__srv__AddLogging_Request__init(message_memory);
}

void crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_fini_function(void * message_memory)
{
  crazyflie_interfaces__srv__AddLogging_Request__fini(message_memory);
}

size_t crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__size_function__AddLogging_Request__vars(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__get_const_function__AddLogging_Request__vars(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__get_function__AddLogging_Request__vars(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__fetch_function__AddLogging_Request__vars(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__get_const_function__AddLogging_Request__vars(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__assign_function__AddLogging_Request__vars(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__get_function__AddLogging_Request__vars(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__resize_function__AddLogging_Request__vars(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_message_member_array[3] = {
  {
    "topic_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__AddLogging_Request, topic_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frequency",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__AddLogging_Request, frequency),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vars",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__AddLogging_Request, vars),  // bytes offset in struct
    NULL,  // default value
    crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__size_function__AddLogging_Request__vars,  // size() function pointer
    crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__get_const_function__AddLogging_Request__vars,  // get_const(index) function pointer
    crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__get_function__AddLogging_Request__vars,  // get(index) function pointer
    crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__fetch_function__AddLogging_Request__vars,  // fetch(index, &value) function pointer
    crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__assign_function__AddLogging_Request__vars,  // assign(index, value) function pointer
    crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__resize_function__AddLogging_Request__vars  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_message_members = {
  "crazyflie_interfaces__srv",  // message namespace
  "AddLogging_Request",  // message name
  3,  // number of fields
  sizeof(crazyflie_interfaces__srv__AddLogging_Request),
  crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_message_member_array,  // message members
  crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_message_type_support_handle = {
  0,
  &crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, AddLogging_Request)() {
  if (!crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__srv__AddLogging_Request__rosidl_typesupport_introspection_c__AddLogging_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "crazyflie_interfaces/srv/detail/add_logging__rosidl_typesupport_introspection_c.h"
// already included above
// #include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/add_logging__functions.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/add_logging__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_interfaces__srv__AddLogging_Response__init(message_memory);
}

void crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_fini_function(void * message_memory)
{
  crazyflie_interfaces__srv__AddLogging_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_interfaces__srv__AddLogging_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_message_members = {
  "crazyflie_interfaces__srv",  // message namespace
  "AddLogging_Response",  // message name
  1,  // number of fields
  sizeof(crazyflie_interfaces__srv__AddLogging_Response),
  crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_message_member_array,  // message members
  crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_message_type_support_handle = {
  0,
  &crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, AddLogging_Response)() {
  if (!crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_message_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_interfaces__srv__AddLogging_Response__rosidl_typesupport_introspection_c__AddLogging_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "crazyflie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/add_logging__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers crazyflie_interfaces__srv__detail__add_logging__rosidl_typesupport_introspection_c__AddLogging_service_members = {
  "crazyflie_interfaces__srv",  // service namespace
  "AddLogging",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // crazyflie_interfaces__srv__detail__add_logging__rosidl_typesupport_introspection_c__AddLogging_Request_message_type_support_handle,
  NULL  // response message
  // crazyflie_interfaces__srv__detail__add_logging__rosidl_typesupport_introspection_c__AddLogging_Response_message_type_support_handle
};

static rosidl_service_type_support_t crazyflie_interfaces__srv__detail__add_logging__rosidl_typesupport_introspection_c__AddLogging_service_type_support_handle = {
  0,
  &crazyflie_interfaces__srv__detail__add_logging__rosidl_typesupport_introspection_c__AddLogging_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, AddLogging_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, AddLogging_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, AddLogging)() {
  if (!crazyflie_interfaces__srv__detail__add_logging__rosidl_typesupport_introspection_c__AddLogging_service_type_support_handle.typesupport_identifier) {
    crazyflie_interfaces__srv__detail__add_logging__rosidl_typesupport_introspection_c__AddLogging_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)crazyflie_interfaces__srv__detail__add_logging__rosidl_typesupport_introspection_c__AddLogging_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, AddLogging_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_interfaces, srv, AddLogging_Response)()->data;
  }

  return &crazyflie_interfaces__srv__detail__add_logging__rosidl_typesupport_introspection_c__AddLogging_service_type_support_handle;
}
