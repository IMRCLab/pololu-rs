// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motion_capture_tracking_interfaces:msg/NamedPose.idl
// generated code does not contain a copyright notice

#ifndef MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE__BUILDER_HPP_
#define MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motion_capture_tracking_interfaces/msg/detail/named_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motion_capture_tracking_interfaces
{

namespace msg
{

namespace builder
{

class Init_NamedPose_pose
{
public:
  explicit Init_NamedPose_pose(::motion_capture_tracking_interfaces::msg::NamedPose & msg)
  : msg_(msg)
  {}
  ::motion_capture_tracking_interfaces::msg::NamedPose pose(::motion_capture_tracking_interfaces::msg::NamedPose::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motion_capture_tracking_interfaces::msg::NamedPose msg_;
};

class Init_NamedPose_name
{
public:
  Init_NamedPose_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NamedPose_pose name(::motion_capture_tracking_interfaces::msg::NamedPose::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_NamedPose_pose(msg_);
  }

private:
  ::motion_capture_tracking_interfaces::msg::NamedPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motion_capture_tracking_interfaces::msg::NamedPose>()
{
  return motion_capture_tracking_interfaces::msg::builder::Init_NamedPose_name();
}

}  // namespace motion_capture_tracking_interfaces

#endif  // MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE__BUILDER_HPP_
