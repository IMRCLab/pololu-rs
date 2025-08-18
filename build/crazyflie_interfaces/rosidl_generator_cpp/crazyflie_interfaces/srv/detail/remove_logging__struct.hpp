// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_interfaces:srv/RemoveLogging.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__REMOVE_LOGGING__STRUCT_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__REMOVE_LOGGING__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__srv__RemoveLogging_Request __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__srv__RemoveLogging_Request __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RemoveLogging_Request_
{
  using Type = RemoveLogging_Request_<ContainerAllocator>;

  explicit RemoveLogging_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic_name = "";
    }
  }

  explicit RemoveLogging_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : topic_name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->topic_name = "";
    }
  }

  // field types and members
  using _topic_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _topic_name_type topic_name;

  // setters for named parameter idiom
  Type & set__topic_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->topic_name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__srv__RemoveLogging_Request
    std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__srv__RemoveLogging_Request
    std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RemoveLogging_Request_ & other) const
  {
    if (this->topic_name != other.topic_name) {
      return false;
    }
    return true;
  }
  bool operator!=(const RemoveLogging_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RemoveLogging_Request_

// alias to use template instance with default allocator
using RemoveLogging_Request =
  crazyflie_interfaces::srv::RemoveLogging_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crazyflie_interfaces


#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__srv__RemoveLogging_Response __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__srv__RemoveLogging_Response __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct RemoveLogging_Response_
{
  using Type = RemoveLogging_Response_<ContainerAllocator>;

  explicit RemoveLogging_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit RemoveLogging_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__srv__RemoveLogging_Response
    std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__srv__RemoveLogging_Response
    std::shared_ptr<crazyflie_interfaces::srv::RemoveLogging_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RemoveLogging_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const RemoveLogging_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RemoveLogging_Response_

// alias to use template instance with default allocator
using RemoveLogging_Response =
  crazyflie_interfaces::srv::RemoveLogging_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace crazyflie_interfaces
{

namespace srv
{

struct RemoveLogging
{
  using Request = crazyflie_interfaces::srv::RemoveLogging_Request;
  using Response = crazyflie_interfaces::srv::RemoveLogging_Response;
};

}  // namespace srv

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__REMOVE_LOGGING__STRUCT_HPP_
