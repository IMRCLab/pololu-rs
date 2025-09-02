// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:srv/StartTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__START_TRAJECTORY__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__START_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/srv/detail/start_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_StartTrajectory_Request_relative
{
public:
  explicit Init_StartTrajectory_Request_relative(::crazyflie_interfaces::srv::StartTrajectory_Request & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::srv::StartTrajectory_Request relative(::crazyflie_interfaces::srv::StartTrajectory_Request::_relative_type arg)
  {
    msg_.relative = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::StartTrajectory_Request msg_;
};

class Init_StartTrajectory_Request_reversed
{
public:
  explicit Init_StartTrajectory_Request_reversed(::crazyflie_interfaces::srv::StartTrajectory_Request & msg)
  : msg_(msg)
  {}
  Init_StartTrajectory_Request_relative reversed(::crazyflie_interfaces::srv::StartTrajectory_Request::_reversed_type arg)
  {
    msg_.reversed = std::move(arg);
    return Init_StartTrajectory_Request_relative(msg_);
  }

private:
  ::crazyflie_interfaces::srv::StartTrajectory_Request msg_;
};

class Init_StartTrajectory_Request_timescale
{
public:
  explicit Init_StartTrajectory_Request_timescale(::crazyflie_interfaces::srv::StartTrajectory_Request & msg)
  : msg_(msg)
  {}
  Init_StartTrajectory_Request_reversed timescale(::crazyflie_interfaces::srv::StartTrajectory_Request::_timescale_type arg)
  {
    msg_.timescale = std::move(arg);
    return Init_StartTrajectory_Request_reversed(msg_);
  }

private:
  ::crazyflie_interfaces::srv::StartTrajectory_Request msg_;
};

class Init_StartTrajectory_Request_trajectory_id
{
public:
  explicit Init_StartTrajectory_Request_trajectory_id(::crazyflie_interfaces::srv::StartTrajectory_Request & msg)
  : msg_(msg)
  {}
  Init_StartTrajectory_Request_timescale trajectory_id(::crazyflie_interfaces::srv::StartTrajectory_Request::_trajectory_id_type arg)
  {
    msg_.trajectory_id = std::move(arg);
    return Init_StartTrajectory_Request_timescale(msg_);
  }

private:
  ::crazyflie_interfaces::srv::StartTrajectory_Request msg_;
};

class Init_StartTrajectory_Request_group_mask
{
public:
  Init_StartTrajectory_Request_group_mask()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StartTrajectory_Request_trajectory_id group_mask(::crazyflie_interfaces::srv::StartTrajectory_Request::_group_mask_type arg)
  {
    msg_.group_mask = std::move(arg);
    return Init_StartTrajectory_Request_trajectory_id(msg_);
  }

private:
  ::crazyflie_interfaces::srv::StartTrajectory_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::StartTrajectory_Request>()
{
  return crazyflie_interfaces::srv::builder::Init_StartTrajectory_Request_group_mask();
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
auto build<::crazyflie_interfaces::srv::StartTrajectory_Response>()
{
  return ::crazyflie_interfaces::srv::StartTrajectory_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__START_TRAJECTORY__BUILDER_HPP_
