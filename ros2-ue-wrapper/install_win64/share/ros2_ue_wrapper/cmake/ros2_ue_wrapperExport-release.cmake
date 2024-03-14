#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ros2_ue_wrapper::ros2_ue_wrapper" for configuration "Release"
set_property(TARGET ros2_ue_wrapper::ros2_ue_wrapper APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ros2_ue_wrapper::ros2_ue_wrapper PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/ros2_ue_wrapper.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/ros2_ue_wrapper.dll"
  )

list(APPEND _cmake_import_check_targets ros2_ue_wrapper::ros2_ue_wrapper )
list(APPEND _cmake_import_check_files_for_ros2_ue_wrapper::ros2_ue_wrapper "${_IMPORT_PREFIX}/lib/ros2_ue_wrapper.lib" "${_IMPORT_PREFIX}/bin/ros2_ue_wrapper.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
