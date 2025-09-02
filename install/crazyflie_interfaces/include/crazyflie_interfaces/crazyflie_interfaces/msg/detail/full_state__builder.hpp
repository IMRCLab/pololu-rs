// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/FullState.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__FULL_STATE__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__FULL_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/full_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_FullState_acc
{
public:
  explicit Init_FullState_acc(::crazyflie_interfaces::msg::FullState & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::FullState acc(::crazyflie_interfaces::msg::FullState::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::FullState msg_;
};

class Init_FullState_twist
{
public:
  explicit Init_FullState_twist(::crazyflie_interfaces::msg::FullState & msg)
  : msg_(msg)
  {}
  Init_FullState_acc twist(::crazyflie_interfaces::msg::FullState::_twist_type arg)
  {
    msg_.twist = std::move(arg);
    return Init_FullState_acc(msg_);
  }

private:
  ::crazyflie_interfaces::msg::FullState msg_;
};

class Init_FullState_pose
{
public:
  explicit Init_FullState_pose(::crazyflie_interfaces::msg::FullState & msg)
  : msg_(msg)
  {}
  Init_FullState_twist pose(::crazyflie_interfaces::msg::FullState::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_FullState_twist(msg_);
  }

private:
  ::crazyflie_interfaces::msg::FullState msg_;
};

class Init_FullState_header
{
public:
  Init_FullState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FullState_pose header(::crazyflie_interfaces::msg::FullState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FullState_pose(msg_);
  }

private:
  ::crazyflie_interfaces::msg::FullState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::FullState>()
{
  return crazyflie_interfaces::msg::builder::Init_FullState_header();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__FULL_STATE__BUILDER_HPP_
