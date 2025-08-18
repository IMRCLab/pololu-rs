// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:srv/UploadTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__UPLOAD_TRAJECTORY__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__UPLOAD_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/srv/detail/upload_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace srv
{

namespace builder
{

class Init_UploadTrajectory_Request_pieces
{
public:
  explicit Init_UploadTrajectory_Request_pieces(::crazyflie_interfaces::srv::UploadTrajectory_Request & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::srv::UploadTrajectory_Request pieces(::crazyflie_interfaces::srv::UploadTrajectory_Request::_pieces_type arg)
  {
    msg_.pieces = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::srv::UploadTrajectory_Request msg_;
};

class Init_UploadTrajectory_Request_piece_offset
{
public:
  explicit Init_UploadTrajectory_Request_piece_offset(::crazyflie_interfaces::srv::UploadTrajectory_Request & msg)
  : msg_(msg)
  {}
  Init_UploadTrajectory_Request_pieces piece_offset(::crazyflie_interfaces::srv::UploadTrajectory_Request::_piece_offset_type arg)
  {
    msg_.piece_offset = std::move(arg);
    return Init_UploadTrajectory_Request_pieces(msg_);
  }

private:
  ::crazyflie_interfaces::srv::UploadTrajectory_Request msg_;
};

class Init_UploadTrajectory_Request_trajectory_id
{
public:
  Init_UploadTrajectory_Request_trajectory_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UploadTrajectory_Request_piece_offset trajectory_id(::crazyflie_interfaces::srv::UploadTrajectory_Request::_trajectory_id_type arg)
  {
    msg_.trajectory_id = std::move(arg);
    return Init_UploadTrajectory_Request_piece_offset(msg_);
  }

private:
  ::crazyflie_interfaces::srv::UploadTrajectory_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::srv::UploadTrajectory_Request>()
{
  return crazyflie_interfaces::srv::builder::Init_UploadTrajectory_Request_trajectory_id();
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
auto build<::crazyflie_interfaces::srv::UploadTrajectory_Response>()
{
  return ::crazyflie_interfaces::srv::UploadTrajectory_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__UPLOAD_TRAJECTORY__BUILDER_HPP_
