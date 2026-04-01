cmake_minimum_required(VERSION 3.8)
project(pololu)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# add dependencies
add_subdirectory(deps/crazyflie-link-cpp)

include_directories(
  deps/crazyflie-link-cpp/include
  deps/crazyflie-link-cpp/examples
)

add_executable(teleop 
  src/teleop.cpp
)

add_executable(mocap_broadcast
  src/mocap_broadcast.cpp
)

add_executable(controller_interface
  src/controller_interface.cpp
)

# ADD MISSING teleop dependencies:
target_link_libraries(teleop
  crazyflieLinkCpp
)
ament_target_dependencies(teleop 
  rclcpp
  sensor_msgs
  std_srvs
  geometry_msgs
  motion_capture_tracking_interfaces
)

target_link_libraries(controller_interface
  crazyflieLinkCpp
)
ament_target_dependencies(controller_interface 
  rclcpp
  sensor_msgs
  std_srvs
  geometry_msgs
  motion_capture_tracking_interfaces
)

target_link_libraries(mocap_broadcast
  crazyflieLinkCpp
)
ament_target_dependencies(mocap_broadcast 
  rclcpp
  sensor_msgs
  std_srvs
  geometry_msgs
  motion_capture_tracking_interfaces
)

# FIX Install section - add ALL executables:
install(TARGETS
  teleop
  mocap_broadcast
  controller_interface  # ADD THIS LINE
  DESTINATION lib/${PROJECT_NAME})

# Install launch, config, and urdf files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
