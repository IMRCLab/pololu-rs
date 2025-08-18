// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/Position.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__POSITION__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_Position_yaw
{
public:
  explicit Init_Position_yaw(::crazyflie_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::Position yaw(::crazyflie_interfaces::msg::Position::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Position msg_;
};

class Init_Position_z
{
public:
  explicit Init_Position_z(::crazyflie_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_yaw z(::crazyflie_interfaces::msg::Position::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Position_yaw(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Position msg_;
};

class Init_Position_y
{
public:
  explicit Init_Position_y(::crazyflie_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_z y(::crazyflie_interfaces::msg::Position::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Position_z(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Position msg_;
};

class Init_Position_x
{
public:
  explicit Init_Position_x(::crazyflie_interfaces::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_y x(::crazyflie_interfaces::msg::Position::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Position_y(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Position msg_;
};

class Init_Position_header
{
public:
  Init_Position_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_x header(::crazyflie_interfaces::msg::Position::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Position_x(msg_);
  }

private:
  ::crazyflie_interfaces::msg::Position msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::Position>()
{
  return crazyflie_interfaces::msg::builder::Init_Position_header();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__POSITION__BUILDER_HPP_
