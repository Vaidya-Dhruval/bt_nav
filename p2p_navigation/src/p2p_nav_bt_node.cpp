// src/p2p_nav_bt_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include "p2p_navigation/custom_nav_to_pose.hpp"
#include <string>
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("p2p_bt_runner");
  
  // Get the XML file path from a parameter or use default
  std::string bt_xml_file = "p2p_ws/src/bt_nav/p2p_navigation/behaviour_trees/custom_p2p_bt.xml";
  node->declare_parameter("bt_xml_file", bt_xml_file);
  node->get_parameter("bt_xml_file", bt_xml_file);
  
  RCLCPP_INFO(node->get_logger(), "Loading BT from: %s", bt_xml_file.c_str());
  
  BT::BehaviorTreeFactory factory;
  
  // Register the CustomNavToPose node type
  factory.registerNodeType<p2p_navigation::CustomNavToPose>("CustomNavToPose");
  
  auto tree = factory.createTreeFromFile(bt_xml_file);
  
  RCLCPP_INFO(node->get_logger(), "BT loaded successfully, starting execution");
  
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
      status = tree.tickWhileRunning();
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }  
  
  rclcpp::shutdown();
  return 0;
}