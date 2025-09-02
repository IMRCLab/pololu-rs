// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_interfaces:srv/StartTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__START_TRAJECTORY__STRUCT_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__START_TRAJECTORY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__srv__StartTrajectory_Request __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__srv__StartTrajectory_Request __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartTrajectory_Request_
{
  using Type = StartTrajectory_Request_<ContainerAllocator>;

  explicit StartTrajectory_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->group_mask = 0;
      this->trajectory_id = 0;
      this->timescale = 0.0f;
      this->reversed = false;
      this->relative = false;
    }
  }

  explicit StartTrajectory_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->group_mask = 0;
      this->trajectory_id = 0;
      this->timescale = 0.0f;
      this->reversed = false;
      this->relative = false;
    }
  }

  // field types and members
  using _group_mask_type =
    uint8_t;
  _group_mask_type group_mask;
  using _trajectory_id_type =
    uint8_t;
  _trajectory_id_type trajectory_id;
  using _timescale_type =
    float;
  _timescale_type timescale;
  using _reversed_type =
    bool;
  _reversed_type reversed;
  using _relative_type =
    bool;
  _relative_type relative;

  // setters for named parameter idiom
  Type & set__group_mask(
    const uint8_t & _arg)
  {
    this->group_mask = _arg;
    return *this;
  }
  Type & set__trajectory_id(
    const uint8_t & _arg)
  {
    this->trajectory_id = _arg;
    return *this;
  }
  Type & set__timescale(
    const float & _arg)
  {
    this->timescale = _arg;
    return *this;
  }
  Type & set__reversed(
    const bool & _arg)
  {
    this->reversed = _arg;
    return *this;
  }
  Type & set__relative(
    const bool & _arg)
  {
    this->relative = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__srv__StartTrajectory_Request
    std::shared_ptr<crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__srv__StartTrajectory_Request
    std::shared_ptr<crazyflie_interfaces::srv::StartTrajectory_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartTrajectory_Request_ & other) const
  {
    if (this->group_mask != other.group_mask) {
      return false;
    }
    if (this->trajectory_id != other.trajectory_id) {
      return false;
    }
    if (this->timescale != other.timescale) {
      return false;
    }
    if (this->reversed != other.reversed) {
      return false;
    }
    if (this->relative != other.relative) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartTrajectory_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartTrajectory_Request_

// alias to use template instance with default allocator
using StartTrajectory_Request =
  crazyflie_interfaces::srv::StartTrajectory_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crazyflie_interfaces


#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__srv__StartTrajectory_Response __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__srv__StartTrajectory_Response __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartTrajectory_Response_
{
  using Type = StartTrajectory_Response_<ContainerAllocator>;

  explicit StartTrajectory_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit StartTrajectory_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__srv__StartTrajectory_Response
    std::shared_ptr<crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__srv__StartTrajectory_Response
    std::shared_ptr<crazyflie_interfaces::srv::StartTrajectory_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartTrajectory_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartTrajectory_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartTrajectory_Response_

// alias to use template instance with default allocator
using StartTrajectory_Response =
  crazyflie_interfaces::srv::StartTrajectory_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace crazyflie_interfaces
{

namespace srv
{

struct StartTrajectory
{
  using Request = crazyflie_interfaces::srv::StartTrajectory_Request;
  using Response = crazyflie_interfaces::srv::StartTrajectory_Response;
};

}  // namespace srv

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__START_TRAJECTORY__STRUCT_HPP_
