// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motion_capture_tracking_interfaces:msg/NamedPoseArray.idl
// generated code does not contain a copyright notice

#ifndef MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE_ARRAY__BUILDER_HPP_
#define MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motion_capture_tracking_interfaces/msg/detail/named_pose_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motion_capture_tracking_interfaces
{

namespace msg
{

namespace builder
{

class Init_NamedPoseArray_poses
{
public:
  explicit Init_NamedPoseArray_poses(::motion_capture_tracking_interfaces::msg::NamedPoseArray & msg)
  : msg_(msg)
  {}
  ::motion_capture_tracking_interfaces::msg::NamedPoseArray poses(::motion_capture_tracking_interfaces::msg::NamedPoseArray::_poses_type arg)
  {
    msg_.poses = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motion_capture_tracking_interfaces::msg::NamedPoseArray msg_;
};

class Init_NamedPoseArray_header
{
public:
  Init_NamedPoseArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NamedPoseArray_poses header(::motion_capture_tracking_interfaces::msg::NamedPoseArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NamedPoseArray_poses(msg_);
  }

private:
  ::motion_capture_tracking_interfaces::msg::NamedPoseArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motion_capture_tracking_interfaces::msg::NamedPoseArray>()
{
  return motion_capture_tracking_interfaces::msg::builder::Init_NamedPoseArray_header();
}

}  // namespace motion_capture_tracking_interfaces

#endif  // MOTION_CAPTURE_TRACKING_INTERFACES__MSG__DETAIL__NAMED_POSE_ARRAY__BUILDER_HPP_
