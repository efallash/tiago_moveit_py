cmake_minimum_required(VERSION 3.8)
project(moveit_custom_adapters)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(srdfdom REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)

find_package(
  Boost
  REQUIRED
  system
  filesystem
  date_time
  program_options
  thread
  chrono)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  Eigen3
  generate_parameter_library
  message_filters
  moveit_core
  moveit_msgs
  moveit_ros_occupancy_map_monitor
  pluginlib
  rclcpp
  rclcpp_components
  srdfdom
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  urdf
  # moveit_ros_perception
)


# Set library name
set(LIBRARY_NAME moveit_custom_adapters)

# Define the library
add_library(${LIBRARY_NAME} SHARED src/fix_start_state_bounds.cpp)

# Link required dependencies
target_link_libraries(${LIBRARY_NAME})
ament_target_dependencies(${LIBRARY_NAME} moveit_core moveit_ros_planning pluginlib rclcpp)

# Export plugin description file
pluginlib_export_plugin_description_file(moveit_core custom_request_adapters_plugin_description.xml)

# Install the library
install(
  TARGETS ${LIBRARY_NAME}
  EXPORT ${LIBRARY_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/moveit_custom_adapters
)

# Install plugin description file
install(FILES custom_request_adapters_plugin_description.xml DESTINATION share/${PROJECT_NAME})

# Export package information
ament_export_targets(${LIBRARY_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core moveit_ros_planning pluginlib rclcpp)

# Enable testing if needed
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
