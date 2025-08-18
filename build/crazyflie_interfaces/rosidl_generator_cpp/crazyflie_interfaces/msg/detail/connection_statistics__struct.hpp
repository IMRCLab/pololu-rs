// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_interfaces:msg/ConnectionStatistics.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__STRUCT_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__msg__ConnectionStatistics __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__msg__ConnectionStatistics __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ConnectionStatistics_
{
  using Type = ConnectionStatistics_<ContainerAllocator>;

  explicit ConnectionStatistics_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uri = "";
      this->sent_count = 0ull;
      this->sent_ping_count = 0ull;
      this->receive_count = 0ull;
      this->enqueued_count = 0ull;
      this->ack_count = 0ull;
    }
  }

  explicit ConnectionStatistics_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : uri(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->uri = "";
      this->sent_count = 0ull;
      this->sent_ping_count = 0ull;
      this->receive_count = 0ull;
      this->enqueued_count = 0ull;
      this->ack_count = 0ull;
    }
  }

  // field types and members
  using _uri_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _uri_type uri;
  using _sent_count_type =
    uint64_t;
  _sent_count_type sent_count;
  using _sent_ping_count_type =
    uint64_t;
  _sent_ping_count_type sent_ping_count;
  using _receive_count_type =
    uint64_t;
  _receive_count_type receive_count;
  using _enqueued_count_type =
    uint64_t;
  _enqueued_count_type enqueued_count;
  using _ack_count_type =
    uint64_t;
  _ack_count_type ack_count;

  // setters for named parameter idiom
  Type & set__uri(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->uri = _arg;
    return *this;
  }
  Type & set__sent_count(
    const uint64_t & _arg)
  {
    this->sent_count = _arg;
    return *this;
  }
  Type & set__sent_ping_count(
    const uint64_t & _arg)
  {
    this->sent_ping_count = _arg;
    return *this;
  }
  Type & set__receive_count(
    const uint64_t & _arg)
  {
    this->receive_count = _arg;
    return *this;
  }
  Type & set__enqueued_count(
    const uint64_t & _arg)
  {
    this->enqueued_count = _arg;
    return *this;
  }
  Type & set__ack_count(
    const uint64_t & _arg)
  {
    this->ack_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__msg__ConnectionStatistics
    std::shared_ptr<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__msg__ConnectionStatistics
    std::shared_ptr<crazyflie_interfaces::msg::ConnectionStatistics_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ConnectionStatistics_ & other) const
  {
    if (this->uri != other.uri) {
      return false;
    }
    if (this->sent_count != other.sent_count) {
      return false;
    }
    if (this->sent_ping_count != other.sent_ping_count) {
      return false;
    }
    if (this->receive_count != other.receive_count) {
      return false;
    }
    if (this->enqueued_count != other.enqueued_count) {
      return false;
    }
    if (this->ack_count != other.ack_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const ConnectionStatistics_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ConnectionStatistics_

// alias to use template instance with default allocator
using ConnectionStatistics =
  crazyflie_interfaces::msg::ConnectionStatistics_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__CONNECTION_STATISTICS__STRUCT_HPP_
