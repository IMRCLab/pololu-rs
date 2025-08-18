// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/VelocityWorld.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__VELOCITY_WORLD__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__VELOCITY_WORLD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/velocity_world__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_VelocityWorld_yaw_rate
{
public:
  explicit Init_VelocityWorld_yaw_rate(::crazyflie_interfaces::msg::VelocityWorld & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::VelocityWorld yaw_rate(::crazyflie_interfaces::msg::VelocityWorld::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::VelocityWorld msg_;
};

class Init_VelocityWorld_vel
{
public:
  explicit Init_VelocityWorld_vel(::crazyflie_interfaces::msg::VelocityWorld & msg)
  : msg_(msg)
  {}
  Init_VelocityWorld_yaw_rate vel(::crazyflie_interfaces::msg::VelocityWorld::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_VelocityWorld_yaw_rate(msg_);
  }

private:
  ::crazyflie_interfaces::msg::VelocityWorld msg_;
};

class Init_VelocityWorld_header
{
public:
  Init_VelocityWorld_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VelocityWorld_vel header(::crazyflie_interfaces::msg::VelocityWorld::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VelocityWorld_vel(msg_);
  }

private:
  ::crazyflie_interfaces::msg::VelocityWorld msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::VelocityWorld>()
{
  return crazyflie_interfaces::msg::builder::Init_VelocityWorld_header();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__VELOCITY_WORLD__BUILDER_HPP_
