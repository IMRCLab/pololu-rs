// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:srv/RemoveLogging.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__REMOVE_LOGGING__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__REMOVE_LOGGING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/srv/detail/remove_logging__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_RemoveLogging_Request_topic_name
{
public:
  Init_RemoveLogging_Request_topic_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::crazyflie_interfaces::srv::RemoveLogging_Request topic_name(::crazyflie_interfaces::srv::RemoveLogging_Request::_topic_name_type arg)
  {
    msg_.topic_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::RemoveLogging_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::RemoveLogging_Request>()
{
  return crazyflie_interfaces::srv::builder::Init_RemoveLogging_Request_topic_name();
}

}  // namespace crazyflie_interfaces


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_RemoveLogging_Response_success
{
public:
  Init_RemoveLogging_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::crazyflie_interfaces::srv::RemoveLogging_Response success(::crazyflie_interfaces::srv::RemoveLogging_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::RemoveLogging_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::RemoveLogging_Response>()
{
  return crazyflie_interfaces::srv::builder::Init_RemoveLogging_Response_success();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__REMOVE_LOGGING__BUILDER_HPP_
