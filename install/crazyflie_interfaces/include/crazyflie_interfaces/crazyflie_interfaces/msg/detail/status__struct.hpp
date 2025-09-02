// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_interfaces:msg/Status.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__STRUCT_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__msg__Status __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__msg__Status __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Status_
{
  using Type = Status_<ContainerAllocator>;

  explicit Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->supervisor_info = 0;
      this->battery_voltage = 0.0f;
      this->pm_state = 0;
      this->rssi = 0;
      this->num_rx_broadcast = 0ul;
      this->num_tx_broadcast = 0ul;
      this->num_rx_unicast = 0ul;
      this->num_tx_unicast = 0ul;
      this->latency_unicast = 0;
    }
  }

  explicit Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->supervisor_info = 0;
      this->battery_voltage = 0.0f;
      this->pm_state = 0;
      this->rssi = 0;
      this->num_rx_broadcast = 0ul;
      this->num_tx_broadcast = 0ul;
      this->num_rx_unicast = 0ul;
      this->num_tx_unicast = 0ul;
      this->latency_unicast = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _supervisor_info_type =
    uint16_t;
  _supervisor_info_type supervisor_info;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _pm_state_type =
    uint8_t;
  _pm_state_type pm_state;
  using _rssi_type =
    uint8_t;
  _rssi_type rssi;
  using _num_rx_broadcast_type =
    uint32_t;
  _num_rx_broadcast_type num_rx_broadcast;
  using _num_tx_broadcast_type =
    uint32_t;
  _num_tx_broadcast_type num_tx_broadcast;
  using _num_rx_unicast_type =
    uint32_t;
  _num_rx_unicast_type num_rx_unicast;
  using _num_tx_unicast_type =
    uint32_t;
  _num_tx_unicast_type num_tx_unicast;
  using _latency_unicast_type =
    uint16_t;
  _latency_unicast_type latency_unicast;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__supervisor_info(
    const uint16_t & _arg)
  {
    this->supervisor_info = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__pm_state(
    const uint8_t & _arg)
  {
    this->pm_state = _arg;
    return *this;
  }
  Type & set__rssi(
    const uint8_t & _arg)
  {
    this->rssi = _arg;
    return *this;
  }
  Type & set__num_rx_broadcast(
    const uint32_t & _arg)
  {
    this->num_rx_broadcast = _arg;
    return *this;
  }
  Type & set__num_tx_broadcast(
    const uint32_t & _arg)
  {
    this->num_tx_broadcast = _arg;
    return *this;
  }
  Type & set__num_rx_unicast(
    const uint32_t & _arg)
  {
    this->num_rx_unicast = _arg;
    return *this;
  }
  Type & set__num_tx_unicast(
    const uint32_t & _arg)
  {
    this->num_tx_unicast = _arg;
    return *this;
  }
  Type & set__latency_unicast(
    const uint16_t & _arg)
  {
    this->latency_unicast = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint16_t SUPERVISOR_INFO_CAN_BE_ARMED =
    1u;
  static constexpr uint16_t SUPERVISOR_INFO_IS_ARMED =
    2u;
  static constexpr uint16_t SUPERVISOR_INFO_AUTO_ARM =
    4u;
  static constexpr uint16_t SUPERVISOR_INFO_CAN_FLY =
    8u;
  static constexpr uint16_t SUPERVISOR_INFO_IS_FLYING =
    16u;
  static constexpr uint16_t SUPERVISOR_INFO_IS_TUMBLED =
    32u;
  static constexpr uint16_t SUPERVISOR_INFO_IS_LOCKED =
    64u;
  static constexpr uint8_t PM_STATE_BATTERY =
    0u;
  static constexpr uint8_t PM_STATE_CHARGING =
    1u;
  static constexpr uint8_t PM_STATE_CHARGED =
    2u;
  static constexpr uint8_t PM_STATE_LOW_POWER =
    3u;
  static constexpr uint8_t PM_STATE_SHUTDOWN =
    4u;

  // pointer types
  using RawPtr =
    crazyflie_interfaces::msg::Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::msg::Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::msg::Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::msg::Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::msg::Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::msg::Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::msg::Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::msg::Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::msg::Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::msg::Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__msg__Status
    std::shared_ptr<crazyflie_interfaces::msg::Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__msg__Status
    std::shared_ptr<crazyflie_interfaces::msg::Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Status_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->supervisor_info != other.supervisor_info) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->pm_state != other.pm_state) {
      return false;
    }
    if (this->rssi != other.rssi) {
      return false;
    }
    if (this->num_rx_broadcast != other.num_rx_broadcast) {
      return false;
    }
    if (this->num_tx_broadcast != other.num_tx_broadcast) {
      return false;
    }
    if (this->num_rx_unicast != other.num_rx_unicast) {
      return false;
    }
    if (this->num_tx_unicast != other.num_tx_unicast) {
      return false;
    }
    if (this->latency_unicast != other.latency_unicast) {
      return false;
    }
    return true;
  }
  bool operator!=(const Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Status_

// alias to use template instance with default allocator
using Status =
  crazyflie_interfaces::msg::Status_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Status_<ContainerAllocator>::SUPERVISOR_INFO_CAN_BE_ARMED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Status_<ContainerAllocator>::SUPERVISOR_INFO_IS_ARMED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Status_<ContainerAllocator>::SUPERVISOR_INFO_AUTO_ARM;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Status_<ContainerAllocator>::SUPERVISOR_INFO_CAN_FLY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Status_<ContainerAllocator>::SUPERVISOR_INFO_IS_FLYING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Status_<ContainerAllocator>::SUPERVISOR_INFO_IS_TUMBLED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint16_t Status_<ContainerAllocator>::SUPERVISOR_INFO_IS_LOCKED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::PM_STATE_BATTERY;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::PM_STATE_CHARGING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::PM_STATE_CHARGED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::PM_STATE_LOW_POWER;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Status_<ContainerAllocator>::PM_STATE_SHUTDOWN;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__STATUS__STRUCT_HPP_
