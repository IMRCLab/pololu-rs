// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/Hover.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__HOVER__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__HOVER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/hover__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_Hover_z_distance
{
public:
  explicit Init_Hover_z_distance(::crazyflie_interfaces::msg::Hover & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::Hover z_distance(::crazyflie_interfaces::msg::Hover::_z_distance_type arg)
  {
    msg_.z_distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Hover msg_;
};

class Init_Hover_yaw_rate
{
public:
  explicit Init_Hover_yaw_rate(::crazyflie_interfaces::msg::Hover & msg)
  : msg_(msg)
  {}
  Init_Hover_z_distance yaw_rate(::crazyflie_interfaces::msg::Hover::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return Init_Hover_z_distance(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Hover msg_;
};

class Init_Hover_vy
{
public:
  explicit Init_Hover_vy(::crazyflie_interfaces::msg::Hover & msg)
  : msg_(msg)
  {}
  Init_Hover_yaw_rate vy(::crazyflie_interfaces::msg::Hover::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_Hover_yaw_rate(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Hover msg_;
};

class Init_Hover_vx
{
public:
  explicit Init_Hover_vx(::crazyflie_interfaces::msg::Hover & msg)
  : msg_(msg)
  {}
  Init_Hover_vy vx(::crazyflie_interfaces::msg::Hover::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_Hover_vy(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Hover msg_;
};

class Init_Hover_header
{
public:
  Init_Hover_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Hover_vx header(::crazyflie_interfaces::msg::Hover::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Hover_vx(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Hover msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::Hover>()
{
  return crazyflie_interfaces::msg::builder::Init_Hover_header();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__HOVER__BUILDER_HPP_
