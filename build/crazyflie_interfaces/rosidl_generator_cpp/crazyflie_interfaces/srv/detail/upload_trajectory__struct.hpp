// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_interfaces:srv/UploadTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__UPLOAD_TRAJECTORY__STRUCT_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__UPLOAD_TRAJECTORY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pieces'
#include "crazyflie_interfaces/msg/detail/trajectory_polynomial_piece__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__srv__UploadTrajectory_Request __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__srv__UploadTrajectory_Request __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UploadTrajectory_Request_
{
  using Type = UploadTrajectory_Request_<ContainerAllocator>;

  explicit UploadTrajectory_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trajectory_id = 0;
      this->piece_offset = 0ul;
    }
  }

  explicit UploadTrajectory_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trajectory_id = 0;
      this->piece_offset = 0ul;
    }
  }

  // field types and members
  using _trajectory_id_type =
    uint8_t;
  _trajectory_id_type trajectory_id;
  using _piece_offset_type =
    uint32_t;
  _piece_offset_type piece_offset;
  using _pieces_type =
    std::vector<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator>>>;
  _pieces_type pieces;

  // setters for named parameter idiom
  Type & set__trajectory_id(
    const uint8_t & _arg)
  {
    this->trajectory_id = _arg;
    return *this;
  }
  Type & set__piece_offset(
    const uint32_t & _arg)
  {
    this->piece_offset = _arg;
    return *this;
  }
  Type & set__pieces(
    const std::vector<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<crazyflie_interfaces::msg::TrajectoryPolynomialPiece_<ContainerAllocator>>> & _arg)
  {
    this->pieces = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__srv__UploadTrajectory_Request
    std::shared_ptr<crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__srv__UploadTrajectory_Request
    std::shared_ptr<crazyflie_interfaces::srv::UploadTrajectory_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UploadTrajectory_Request_ & other) const
  {
    if (this->trajectory_id != other.trajectory_id) {
      return false;
    }
    if (this->piece_offset != other.piece_offset) {
      return false;
    }
    if (this->pieces != other.pieces) {
      return false;
    }
    return true;
  }
  bool operator!=(const UploadTrajectory_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UploadTrajectory_Request_

// alias to use template instance with default allocator
using UploadTrajectory_Request =
  crazyflie_interfaces::srv::UploadTrajectory_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crazyflie_interfaces


#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__srv__UploadTrajectory_Response __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__srv__UploadTrajectory_Response __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UploadTrajectory_Response_
{
  using Type = UploadTrajectory_Response_<ContainerAllocator>;

  explicit UploadTrajectory_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit UploadTrajectory_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__srv__UploadTrajectory_Response
    std::shared_ptr<crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__srv__UploadTrajectory_Response
    std::shared_ptr<crazyflie_interfaces::srv::UploadTrajectory_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UploadTrajectory_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const UploadTrajectory_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UploadTrajectory_Response_

// alias to use template instance with default allocator
using UploadTrajectory_Response =
  crazyflie_interfaces::srv::UploadTrajectory_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace crazyflie_interfaces
{

namespace srv
{

struct UploadTrajectory
{
  using Request = crazyflie_interfaces::srv::UploadTrajectory_Request;
  using Response = crazyflie_interfaces::srv::UploadTrajectory_Response;
};

}  // namespace srv

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__UPLOAD_TRAJECTORY__STRUCT_HPP_
