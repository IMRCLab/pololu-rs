// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_interfaces:srv/UpdateParams.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__STRUCT_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__srv__UpdateParams_Request __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__srv__UpdateParams_Request __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UpdateParams_Request_
{
  using Type = UpdateParams_Request_<ContainerAllocator>;

  explicit UpdateParams_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit UpdateParams_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _params_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _params_type params;

  // setters for named parameter idiom
  Type & set__params(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->params = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__srv__UpdateParams_Request
    std::shared_ptr<crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__srv__UpdateParams_Request
    std::shared_ptr<crazyflie_interfaces::srv::UpdateParams_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateParams_Request_ & other) const
  {
    if (this->params != other.params) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateParams_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateParams_Request_

// alias to use template instance with default allocator
using UpdateParams_Request =
  crazyflie_interfaces::srv::UpdateParams_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crazyflie_interfaces


#ifndef _WIN32
# define DEPRECATED__crazyflie_interfaces__srv__UpdateParams_Response __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_interfaces__srv__UpdateParams_Response __declspec(deprecated)
#endif

namespace crazyflie_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UpdateParams_Response_
{
  using Type = UpdateParams_Response_<ContainerAllocator>;

  explicit UpdateParams_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit UpdateParams_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_interfaces__srv__UpdateParams_Response
    std::shared_ptr<crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_interfaces__srv__UpdateParams_Response
    std::shared_ptr<crazyflie_interfaces::srv::UpdateParams_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateParams_Response_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateParams_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateParams_Response_

// alias to use template instance with default allocator
using UpdateParams_Response =
  crazyflie_interfaces::srv::UpdateParams_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace crazyflie_interfaces
{

namespace srv
{

struct UpdateParams
{
  using Request = crazyflie_interfaces::srv::UpdateParams_Request;
  using Response = crazyflie_interfaces::srv::UpdateParams_Response;
};

}  // namespace srv

}  // namespace crazyflie_interfaces

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__UPDATE_PARAMS__STRUCT_HPP_
