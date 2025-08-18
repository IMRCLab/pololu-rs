// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:srv/Land.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__LAND__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__LAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/srv/detail/land__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_Land_Request_duration
{
public:
  explicit Init_Land_Request_duration(::crazyflie_interfaces::srv::Land_Request & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::srv::Land_Request duration(::crazyflie_interfaces::srv::Land_Request::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::Land_Request msg_;
};

class Init_Land_Request_height
{
public:
  explicit Init_Land_Request_height(::crazyflie_interfaces::srv::Land_Request & msg)
  : msg_(msg)
  {}
  Init_Land_Request_duration height(::crazyflie_interfaces::srv::Land_Request::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_Land_Request_duration(msg_);
  }

private:
  ::crazyflie_interfaces::srv::Land_Request msg_;
};

class Init_Land_Request_group_mask
{
public:
  Init_Land_Request_group_mask()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Land_Request_height group_mask(::crazyflie_interfaces::srv::Land_Request::_group_mask_type arg)
  {
    msg_.group_mask = std::move(arg);
    return Init_Land_Request_height(msg_);
  }

private:
  ::crazyflie_interfaces::srv::Land_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::Land_Request>()
{
  return crazyflie_interfaces::srv::builder::Init_Land_Request_group_mask();
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
auto build<::crazyflie_interfaces::srv::Land_Response>()
{
  return ::crazyflie_interfaces::srv::Land_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__LAND__BUILDER_HPP_
