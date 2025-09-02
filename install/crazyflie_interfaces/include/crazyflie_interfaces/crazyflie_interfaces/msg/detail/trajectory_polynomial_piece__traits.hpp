// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_interfaces:msg/TrajectoryPolynomialPiece.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__TRAITS_HPP_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_interfaces/msg/detail/trajectory_polynomial_piece__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'duration'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace crazyflie_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrajectoryPolynomialPiece & msg,
  std::ostream & out)
{
  out << "{";
  // member: poly_x
  {
    if (msg.poly_x.size() == 0) {
      out << "poly_x: []";
    } else {
      out << "poly_x: [";
      size_t pending_items = msg.poly_x.size();
      for (auto item : msg.poly_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: poly_y
  {
    if (msg.poly_y.size() == 0) {
      out << "poly_y: []";
    } else {
      out << "poly_y: [";
      size_t pending_items = msg.poly_y.size();
      for (auto item : msg.poly_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: poly_z
  {
    if (msg.poly_z.size() == 0) {
      out << "poly_z: []";
    } else {
      out << "poly_z: [";
      size_t pending_items = msg.poly_z.size();
      for (auto item : msg.poly_z) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: poly_yaw
  {
    if (msg.poly_yaw.size() == 0) {
      out << "poly_yaw: []";
    } else {
      out << "poly_yaw: [";
      size_t pending_items = msg.poly_yaw.size();
      for (auto item : msg.poly_yaw) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: duration
  {
    out << "duration: ";
    to_flow_style_yaml(msg.duration, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrajectoryPolynomialPiece & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: poly_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poly_x.size() == 0) {
      out << "poly_x: []\n";
    } else {
      out << "poly_x:\n";
      for (auto item : msg.poly_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: poly_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poly_y.size() == 0) {
      out << "poly_y: []\n";
    } else {
      out << "poly_y:\n";
      for (auto item : msg.poly_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: poly_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poly_z.size() == 0) {
      out << "poly_z: []\n";
    } else {
      out << "poly_z:\n";
      for (auto item : msg.poly_z) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: poly_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poly_yaw.size() == 0) {
      out << "poly_yaw: []\n";
    } else {
      out << "poly_yaw:\n";
      for (auto item : msg.poly_yaw) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: duration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "duration:\n";
    to_block_style_yaml(msg.duration, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrajectoryPolynomialPiece & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace crazyflie_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use crazyflie_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crazyflie_interfaces::msg::TrajectoryPolynomialPiece & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_interfaces::msg::TrajectoryPolynomialPiece & msg)
{
  return crazyflie_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_interfaces::msg::TrajectoryPolynomialPiece>()
{
  return "crazyflie_interfaces::msg::TrajectoryPolynomialPiece";
}

template<>
inline const char * name<crazyflie_interfaces::msg::TrajectoryPolynomialPiece>()
{
  return "crazyflie_interfaces/msg/TrajectoryPolynomialPiece";
}

template<>
struct has_fixed_size<crazyflie_interfaces::msg::TrajectoryPolynomialPiece>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<crazyflie_interfaces::msg::TrajectoryPolynomialPiece>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<crazyflie_interfaces::msg::TrajectoryPolynomialPiece>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__TRAITS_HPP_
