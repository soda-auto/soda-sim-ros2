#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ros2_ue_wrapper::ros2_ue_wrapper" for configuration "RelWithDebInfo"
set_property(TARGET ros2_ue_wrapper::ros2_ue_wrapper APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(ros2_ue_wrapper::ros2_ue_wrapper PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libros2_ue_wrapper.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libros2_ue_wrapper.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ros2_ue_wrapper::ros2_ue_wrapper )
list(APPEND _IMPORT_CHECK_FILES_FOR_ros2_ue_wrapper::ros2_ue_wrapper "${_IMPORT_PREFIX}/lib/libros2_ue_wrapper.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
