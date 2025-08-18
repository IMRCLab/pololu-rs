// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:srv/GoTo.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__GO_TO__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__GO_TO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/srv/detail/go_to__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_GoTo_Request_duration
{
public:
  explicit Init_GoTo_Request_duration(::crazyflie_interfaces::srv::GoTo_Request & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::srv::GoTo_Request duration(::crazyflie_interfaces::srv::GoTo_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::GoTo_Request msg_;
};

class Init_GoTo_Request_yaw
{
public:
  explicit Init_GoTo_Request_yaw(::crazyflie_interfaces::srv::GoTo_Request & msg)
  : msg_(msg)
  {}
  Init_GoTo_Request_duration yaw(::crazyflie_interfaces::srv::GoTo_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_GoTo_Request_duration(msg_);
  }

private:
  ::crazyflie_interfaces::srv::GoTo_Request msg_;
};

class Init_GoTo_Request_goal
{
public:
  explicit Init_GoTo_Request_goal(::crazyflie_interfaces::srv::GoTo_Request & msg)
  : msg_(msg)
  {}
  Init_GoTo_Request_yaw goal(::crazyflie_interfaces::srv::GoTo_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return Init_GoTo_Request_yaw(msg_);
  }

private:
  ::crazyflie_interfaces::srv::GoTo_Request msg_;
};

class Init_GoTo_Request_relative
{
public:
  explicit Init_GoTo_Request_relative(::crazyflie_interfaces::srv::GoTo_Request & msg)
  : msg_(msg)
  {}
  Init_GoTo_Request_goal relative(::crazyflie_interfaces::srv::GoTo_Request::_relative_type arg)
  {
    msg_.relative = std::move(arg);
    return Init_GoTo_Request_goal(msg_);
  }

private:
  ::crazyflie_interfaces::srv::GoTo_Request msg_;
};

class Init_GoTo_Request_group_mask
{
public:
  Init_GoTo_Request_group_mask()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GoTo_Request_relative group_mask(::crazyflie_interfaces::srv::GoTo_Request::_group_mask_type arg)
  {
    msg_.group_mask = std::move(arg);
    return Init_GoTo_Request_relative(msg_);
  }

private:
  ::crazyflie_interfaces::srv::GoTo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::GoTo_Request>()
{
  return crazyflie_interfaces::srv::builder::Init_GoTo_Request_group_mask();
}

}  // namespace crazyflie_interfaces


namespace crazyflie_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::GoTo_Response>()
{
  return ::crazyflie_interfaces::srv::GoTo_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__GO_TO__BUILDER_HPP_
