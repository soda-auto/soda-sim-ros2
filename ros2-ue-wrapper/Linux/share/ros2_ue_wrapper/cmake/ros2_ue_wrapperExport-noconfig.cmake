#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ros2_ue_wrapper::ros2_ue_wrapper" for configuration ""
set_property(TARGET ros2_ue_wrapper::ros2_ue_wrapper APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ros2_ue_wrapper::ros2_ue_wrapper PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libros2_ue_wrapper.so"
  IMPORTED_SONAME_NOCONFIG "libros2_ue_wrapper.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ros2_ue_wrapper::ros2_ue_wrapper )
list(APPEND _IMPORT_CHECK_FILES_FOR_ros2_ue_wrapper::ros2_ue_wrapper "${_IMPORT_PREFIX}/lib/libros2_ue_wrapper.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
