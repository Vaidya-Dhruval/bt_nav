cmake_minimum_required(VERSION 3.8)
project(navigation_client)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(p2p_navigation_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Add executable for the navigation client
add_executable(navigation_client src/navigation_client.cpp)

ament_target_dependencies(navigation_client
  rclcpp
  rclcpp_action
  p2p_navigation_interfaces
  geometry_msgs
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  navigation_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
