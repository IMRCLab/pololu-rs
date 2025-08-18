// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:srv/Arm.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__ARM__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__ARM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/srv/detail/arm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_Arm_Request_arm
{
public:
  Init_Arm_Request_arm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::crazyflie_interfaces::srv::Arm_Request arm(::crazyflie_interfaces::srv::Arm_Request::_arm_type arg)
  {
    msg_.arm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::Arm_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::Arm_Request>()
{
  return crazyflie_interfaces::srv::builder::Init_Arm_Request_arm();
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
auto build<::crazyflie_interfaces::srv::Arm_Response>()
{
  return ::crazyflie_interfaces::srv::Arm_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__ARM__BUILDER_HPP_
