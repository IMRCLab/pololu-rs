// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_interfaces:msg/TrajectoryPolynomialPiece.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__STRUCT_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'duration'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__msg__TrajectoryPolynomialPiece __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__msg__TrajectoryPolynomialPiece __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajectoryPolynomialPiece_
{
  using Type = TrajectoryPolynomialPiece_<ContainerAllocator>;

  explicit TrajectoryPolynomialPiece_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : duration(_init)
  {
    (void)_init;
  }

  explicit TrajectoryPolynomialPiece_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : duration(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _poly_x_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _poly_x_type poly_x;
  using _poly_y_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _poly_y_type poly_y;
  using _poly_z_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _poly_z_type poly_z;
  using _poly_yaw_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _poly_yaw_type poly_yaw;
  using _duration_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _duration_type duration;

  // setters for named parameter idiom
  Type & set__poly_x(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->poly_x = _arg;
    return *this;
  }
  Type & set__poly_y(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->poly_y = _arg;
    return *this;
  }
  Type & set__poly_z(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->poly_z = _arg;
    return *this;
  }
  Type & set__poly_yaw(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->poly_yaw = _arg;
    return *this;
  }
  Type & set__duration(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->duration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__msg__TrajectoryPolynomialPiece
    std::shared_ptr<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__msg__TrajectoryPolynomialPiece
    std::shared_ptr<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajectoryPolynomialPiece_ & other) const
  {
    if (this->poly_x != other.poly_x) {
      return false;
    }
    if (this->poly_y != other.poly_y) {
      return false;
    }
    if (this->poly_z != other.poly_z) {
      return false;
    }
    if (this->poly_yaw != other.poly_yaw) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajectoryPolynomialPiece_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajectoryPolynomialPiece_

// alias to use template instance with default allocator
using TrajectoryPolynomialPiece =
  crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__STRUCT_HPP_
