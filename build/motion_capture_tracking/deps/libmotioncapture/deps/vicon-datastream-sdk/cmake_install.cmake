# Install script for directory: /home/vincent/Ground_Robots/mocap_broadcast_ros/src/motion_capture_tracking/motion_capture_tracking/deps/libmotioncapture/deps/vicon-datastream-sdk

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/vincent/Ground_Robots/mocap_broadcast_ros/install/motion_capture_tracking")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/vincent/Ground_Robots/mocap_broadcast_ros/build/motion_capture_tracking/deps/libmotioncapture/deps/vicon-datastream-sdk/libViconDataStreamSDK_CPP.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/vicon-datastream-sdk/vicon-datastream-sdkTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/vicon-datastream-sdk/vicon-datastream-sdkTargets.cmake"
         "/home/vincent/Ground_Robots/mocap_broadcast_ros/build/motion_capture_tracking/deps/libmotioncapture/deps/vicon-datastream-sdk/CMakeFiles/Export/5428a4105c4ee63c37f975b722639a51/vicon-datastream-sdkTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/vicon-datastream-sdk/vicon-datastream-sdkTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/vicon-datastream-sdk/vicon-datastream-sdkTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/vicon-datastream-sdk" TYPE FILE FILES "/home/vincent/Ground_Robots/mocap_broadcast_ros/build/motion_capture_tracking/deps/libmotioncapture/deps/vicon-datastream-sdk/CMakeFiles/Export/5428a4105c4ee63c37f975b722639a51/vicon-datastream-sdkTargets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/vicon-datastream-sdk" TYPE FILE FILES "/home/vincent/Ground_Robots/mocap_broadcast_ros/build/motion_capture_tracking/deps/libmotioncapture/deps/vicon-datastream-sdk/CMakeFiles/Export/5428a4105c4ee63c37f975b722639a51/vicon-datastream-sdkTargets-noconfig.cmake")
  endif()
endif()

