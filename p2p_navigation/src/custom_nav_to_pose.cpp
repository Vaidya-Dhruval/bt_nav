// src/custom_nav_to_pose.cpp
#include "p2p_navigation/custom_nav_to_pose.hpp"

namespace p2p_navigation
{

BT::PortsList CustomNavToPose::providedPorts()
{
  return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
}

CustomNavToPose::CustomNavToPose(const std::string &name, const BT::NodeConfiguration &config)
  : BT::ActionNodeBase(name, config),
    node_(rclcpp::Node::make_shared("custom_nav_to_pose")),
    goal_completed_(false)
{
  action_client_ = rclcpp_action::create_client<p2p_navigation_interfaces::action::NavigateToPose>(
    node_, "/navigate_to_pose");
}

BT::NodeStatus CustomNavToPose::tick()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToPose action server not available.");
    return BT::NodeStatus::FAILURE;
  }

  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal)
  {
    RCLCPP_ERROR(node_->get_logger(), "No goal received in BT node.");
    return BT::NodeStatus::FAILURE;
  }

  p2p_navigation_interfaces::action::NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal.value();

  auto send_goal_options = rclcpp_action::Client<p2p_navigation_interfaces::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [this](const auto &result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(node_->get_logger(), "Navigation Succeeded!");
      goal_completed_ = true;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Navigation failed!");
      goal_completed_ = false;
    }
  };

  // Fix: Store future and get the goal handle from it
  auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
  
  // We can wait for the future to be ready if needed (optional)
  // auto status = future_goal_handle.wait_for(std::chrono::seconds(5));
  
  // Get the goal handle from the future
  goal_handle_ = future_goal_handle.get();
  
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server");
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}

void CustomNavToPose::halt()
{
  RCLCPP_INFO(node_->get_logger(), "Halting CustomNavToPose BT Node.");
  // Add code here to cancel the goal if needed
  if (goal_handle_) {
    auto future = action_client_->async_cancel_goal(goal_handle_);
  }
}

}  // namespace p2p_navigation