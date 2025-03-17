cmake_minimum_required(VERSION 3.8)
project(p2p_navigation)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(p2p_navigation_interfaces REQUIRED)

# Configure compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directories
include_directories(
  include
)

# Build the custom behavior tree node
add_library(custom_nav_to_pose SHARED
  src/custom_nav_to_pose.cpp
  src/plugin.cpp
)

target_include_directories(custom_nav_to_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(custom_nav_to_pose
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  behaviortree_cpp
  nav_msgs
  geometry_msgs
  p2p_navigation_interfaces
)

# Build the main BT node
add_executable(p2p_nav_bt_node
  src/p2p_nav_bt_node.cpp
)

target_include_directories(p2p_nav_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(p2p_nav_bt_node
  custom_nav_to_pose
)

ament_target_dependencies(p2p_nav_bt_node
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  behaviortree_cpp
  nav_msgs
  geometry_msgs
)

# Install BT XML files
install(DIRECTORY
  behaviour_trees
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries and executables
install(TARGETS
  custom_nav_to_pose
  p2p_nav_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()