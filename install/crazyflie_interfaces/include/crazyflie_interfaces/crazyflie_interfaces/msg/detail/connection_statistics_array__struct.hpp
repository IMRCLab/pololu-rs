// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_interfaces:msg/ConnectionStatisticsArray.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS_ARRAY__STRUCT_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'stats'
#include "crazyflie_interfaces/msg/detail/connection_statistics__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__msg__ConnectionStatisticsArray __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__msg__ConnectionStatisticsArray __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ConnectionStatisticsArray_
{
  using Type = ConnectionStatisticsArray_<ContainerAllocator>;

  explicit ConnectionStatisticsArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ConnectionStatisticsArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _stats_type =
    std::vector<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator>>>;
  _stats_type stats;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__stats(
    const std::vector<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator>>> & _arg)
  {
    this->stats = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__msg__ConnectionStatisticsArray
    std::shared_ptr<crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__msg__ConnectionStatisticsArray
    std::shared_ptr<crazyflie_interfaces::msg::ConnectionStatisticsArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConnectionStatisticsArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->stats != other.stats) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConnectionStatisticsArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConnectionStatisticsArray_

// alias to use template instance with default allocator
using ConnectionStatisticsArray =
  crazyflie_interfaces::msg::ConnectionStatisticsArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS_ARRAY__STRUCT_HPP_
