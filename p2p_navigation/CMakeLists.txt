cmake_minimum_required(VERSION 3.5)
project(p2p_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(p2p_nav_bt_node src/p2p_nav_bt_node.cpp)
ament_target_dependencies(p2p_nav_bt_node rclcpp rclcpp_action nav2_msgs behaviortree_cpp)

install(TARGETS
  p2p_nav_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
