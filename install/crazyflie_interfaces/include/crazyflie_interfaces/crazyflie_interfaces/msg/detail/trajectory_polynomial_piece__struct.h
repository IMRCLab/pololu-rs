// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_interfaces:msg/TrajectoryPolynomialPiece.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__STRUCT_H_
#define CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'poly_x'
// Member 'poly_y'
// Member 'poly_z'
// Member 'poly_yaw'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'duration'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in msg/TrajectoryPolynomialPiece in the package crazyflie_interfaces.
typedef struct crazyflie_interfaces__msg__TrajectoryPolynomialPiece
{
  rosidl_runtime_c__float__Sequence poly_x;
  rosidl_runtime_c__float__Sequence poly_y;
  rosidl_runtime_c__float__Sequence poly_z;
  rosidl_runtime_c__float__Sequence poly_yaw;
  builtin_interfaces__msg__Duration duration;
} crazyflie_interfaces__msg__TrajectoryPolynomialPiece;

// Struct for a sequence of crazyflie_interfaces__msg__TrajectoryPolynomialPiece.
typedef struct crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence
{
  crazyflie_interfaces__msg__TrajectoryPolynomialPiece * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_interfaces__msg__TrajectoryPolynomialPiece__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_INTERFACES__MSG__DETAIL__TRAJECTORY_POLYNOMIAL_PIECE__STRUCT_H_
