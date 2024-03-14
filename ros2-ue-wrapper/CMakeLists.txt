cmake_minimum_required(VERSION 3.8)
project(ros2_ue_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
	src/wrappers.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs nav_msgs tf2_msgs ackermann_msgs)

if(WIN32)
  target_compile_definitions(${PROJECT_NAME} PUBLIC "_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS")
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

#target_link_libraries(${PROJECT_NAME} PRIVATE
#  rclcpp::rclcpp
#)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "ROS2_UE_WRAPPER_BUILDING_DLL"
)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

install(
  DIRECTORY include/ 
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME} EXPORT ros2_ue_wrapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
	
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

