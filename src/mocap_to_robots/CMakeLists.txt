cmake_minimum_required(VERSION 3.10)
project(mocap_to_robots)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)
find_package(crazyflie REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../crazyflie/deps/crazyflie_tools/crazyflie_cpp/include
)

add_executable(mocap_to_robots src/mocap_to_robots.cpp)

ament_target_dependencies(mocap_to_robots
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  motion_capture_tracking_interfaces
  crazyflie
)

target_link_libraries(mocap_to_robots
  crazyflie_cpp
)

install(TARGETS mocap_to_robots DESTINATION lib/${PROJECT_NAME})
ament_package()
