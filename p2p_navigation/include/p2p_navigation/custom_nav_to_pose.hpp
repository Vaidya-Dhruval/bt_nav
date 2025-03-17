// include/p2p_navigation/custom_nav_to_pose.hpp
#ifndef CUSTOM_NAV_TO_POSE_HPP
#define CUSTOM_NAV_TO_POSE_HPP

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "p2p_navigation_interfaces/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace p2p_navigation
{
class CustomNavToPose : public BT::ActionNodeBase
{
public:
  CustomNavToPose(const std::string& name, const BT::NodeConfiguration& config);
  virtual ~CustomNavToPose() = default;
  
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<p2p_navigation_interfaces::action::NavigateToPose>::SharedPtr action_client_;
  rclcpp_action::Client<p2p_navigation_interfaces::action::NavigateToPose>::GoalHandle::SharedPtr goal_handle_;
  bool goal_completed_;
};
}  // namespace p2p_navigation

#endif // CUSTOM_NAV_TO_POSE_HPP