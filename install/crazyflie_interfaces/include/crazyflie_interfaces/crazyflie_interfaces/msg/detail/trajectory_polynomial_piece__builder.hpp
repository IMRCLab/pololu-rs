// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_interfaces:msg/TrajectoryPolynomialPiece.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__BUILDER_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_interfaces/msg/detail/trajectory_polynomial_piece__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrajectoryPolynomialPiece_duration
{
public:
  explicit Init_TrajectoryPolynomialPiece_duration(::crazyflie_interfaces::msg::TrajectoryPolynomialPiece & msg)
  : msg_(msg)
  {}
  ::crazyflie_interfaces::msg::TrajectoryPolynomialPiece duration(::crazyflie_interfaces::msg::TrajectoryPolynomialPiece::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_interfaces::msg::TrajectoryPolynomialPiece msg_;
};

class Init_TrajectoryPolynomialPiece_poly_yaw
{
public:
  explicit Init_TrajectoryPolynomialPiece_poly_yaw(::crazyflie_interfaces::msg::TrajectoryPolynomialPiece & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPolynomialPiece_duration poly_yaw(::crazyflie_interfaces::msg::TrajectoryPolynomialPiece::_poly_yaw_type arg)
  {
    msg_.poly_yaw = std::move(arg);
    return Init_TrajectoryPolynomialPiece_duration(msg_);
  }

private:
  ::crazyflie_interfaces::msg::TrajectoryPolynomialPiece msg_;
};

class Init_TrajectoryPolynomialPiece_poly_z
{
public:
  explicit Init_TrajectoryPolynomialPiece_poly_z(::crazyflie_interfaces::msg::TrajectoryPolynomialPiece & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPolynomialPiece_poly_yaw poly_z(::crazyflie_interfaces::msg::TrajectoryPolynomialPiece::_poly_z_type arg)
  {
    msg_.poly_z = std::move(arg);
    return Init_TrajectoryPolynomialPiece_poly_yaw(msg_);
  }

private:
  ::crazyflie_interfaces::msg::TrajectoryPolynomialPiece msg_;
};

class Init_TrajectoryPolynomialPiece_poly_y
{
public:
  explicit Init_TrajectoryPolynomialPiece_poly_y(::crazyflie_interfaces::msg::TrajectoryPolynomialPiece & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPolynomialPiece_poly_z poly_y(::crazyflie_interfaces::msg::TrajectoryPolynomialPiece::_poly_y_type arg)
  {
    msg_.poly_y = std::move(arg);
    return Init_TrajectoryPolynomialPiece_poly_z(msg_);
  }

private:
  ::crazyflie_interfaces::msg::TrajectoryPolynomialPiece msg_;
};

class Init_TrajectoryPolynomialPiece_poly_x
{
public:
  Init_TrajectoryPolynomialPiece_poly_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajectoryPolynomialPiece_poly_y poly_x(::crazyflie_interfaces::msg::TrajectoryPolynomialPiece::_poly_x_type arg)
  {
    msg_.poly_x = std::move(arg);
    return Init_TrajectoryPolynomialPiece_poly_y(msg_);
  }

private:
  ::crazyflie_interfaces::msg::TrajectoryPolynomialPiece msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_interfaces::msg::TrajectoryPolynomialPiece>()
{
  return crazyflie_interfaces::msg::builder::Init_TrajectoryPolynomialPiece_poly_x();
}

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__BUILDER_HPP_
