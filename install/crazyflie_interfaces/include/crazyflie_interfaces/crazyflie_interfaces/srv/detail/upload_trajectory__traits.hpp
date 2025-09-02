// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:srv/UploadTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__SRV__DETAIL__UPLOAD_TRAJECTORY__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__SRV__DETAIL__UPLOAD_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/srv/detail/upload_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pieces'
#include "crazyflie_interfaces/msg/detail/trajectory_polynomial_piece__traits.hpp"

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const UploadTrajectory_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: trajectory_id
  {
    out << "trajectory_id: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_id, out);
    out << ", ";
  }

  // member: piece_offset
  {
    out << "piece_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.piece_offset, out);
    out << ", ";
  }

  // member: pieces
  {
    if (msg.pieces.size() == 0) {
      out << "pieces: []";
    } else {
      out << "pieces: [";
      size_t pending_items = msg.pieces.size();
      for (auto item : msg.pieces) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UploadTrajectory_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: trajectory_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trajectory_id: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_id, out);
    out << "\n";
  }

  // member: piece_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "piece_offset: ";
    rosidl_generator_traits::value_to_yaml(msg.piece_offset, out);
    out << "\n";
  }

  // member: pieces
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pieces.size() == 0) {
      out << "pieces: []\n";
    } else {
      out << "pieces:\n";
      for (auto item : msg.pieces) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UploadTrajectory_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use crazyflie_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crazyflie_interfaces::srv::UploadTrajectory_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::UploadTrajectory_Request & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::UploadTrajectory_Request>()
{
  return "crazyflie_interfaces::srv::UploadTrajectory_Request";
}

template<>
inline const char * name<crazyflie_interfaces::srv::UploadTrajectory_Request>()
{
  return "crazyflie_interfaces/srv/UploadTrajectory_Request";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::UploadTrajectory_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::UploadTrajectory_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<crazyflie_interfaces::srv::UploadTrajectory_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace crazyflie_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const UploadTrajectory_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UploadTrajectory_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UploadTrajectory_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace crazyflie_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use crazyflie_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crazyflie_interfaces::srv::UploadTrajectory_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::srv::UploadTrajectory_Response & msg)
{
  return crazyflie_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::srv::UploadTrajectory_Response>()
{
  return "crazyflie_interfaces::srv::UploadTrajectory_Response";
}

template<>
inline const char * name<crazyflie_interfaces::srv::UploadTrajectory_Response>()
{
  return "crazyflie_interfaces/srv/UploadTrajectory_Response";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::UploadTrajectory_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::UploadTrajectory_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<crazyflie_interfaces::srv::UploadTrajectory_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<crazyflie_interfaces::srv::UploadTrajectory>()
{
  return "crazyflie_interfaces::srv::UploadTrajectory";
}

template<>
inline const char * name<crazyflie_interfaces::srv::UploadTrajectory>()
{
  return "crazyflie_interfaces/srv/UploadTrajectory";
}

template<>
struct has_fixed_size<crazyflie_interfaces::srv::UploadTrajectory>
  : std::integral_constant<
    bool,
    has_fixed_size<crazyflie_interfaces::srv::UploadTrajectory_Request>::value &&
    has_fixed_size<crazyflie_interfaces::srv::UploadTrajectory_Response>::value
  >
{
};

template<>
struct has_bounded_size<crazyflie_interfaces::srv::UploadTrajectory>
  : std::integral_constant<
    bool,
    has_bounded_size<crazyflie_interfaces::srv::UploadTrajectory_Request>::value &&
    has_bounded_size<crazyflie_interfaces::srv::UploadTrajectory_Response>::value
  >
{
};

template<>
struct is_service<crazyflie_interfaces::srv::UploadTrajectory>
  : std::true_type
{
};

template<>
struct is_service_request<crazyflie_interfaces::srv::UploadTrajectory_Request>
  : std::true_type
{
};

template<>
struct is_service_response<crazyflie_interfaces::srv::UploadTrajectory_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__SRV__DETAIL__UPLOAD_TRAJECTORY__TRAITS_HPP_
