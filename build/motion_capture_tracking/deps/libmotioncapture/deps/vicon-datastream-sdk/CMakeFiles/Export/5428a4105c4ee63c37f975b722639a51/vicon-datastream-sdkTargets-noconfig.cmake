#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vicon-datastream-sdk::ViconDataStreamSDK_CPP" for configuration ""
set_property(TARGET vicon-datastream-sdk::ViconDataStreamSDK_CPP APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(vicon-datastream-sdk::ViconDataStreamSDK_CPP PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libViconDataStreamSDK_CPP.a"
  )

list(APPEND _cmake_import_check_targets vicon-datastream-sdk::ViconDataStreamSDK_CPP )
list(APPEND _cmake_import_check_files_for_vicon-datastream-sdk::ViconDataStreamSDK_CPP "${_IMPORT_PREFIX}/lib/libViconDataStreamSDK_CPP.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
