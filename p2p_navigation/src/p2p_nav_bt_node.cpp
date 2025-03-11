#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>  // Added missing header for BehaviorTreeFactory

class CustomNavToPose : public BT::ActionNodeBase
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    
    CustomNavToPose(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ActionNodeBase(name, config),
          node_(rclcpp::Node::make_shared("p2p_nav_bt_node"))
    {
        action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "/navigate_to_pose");
    }
    
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
    }
    
    BT::NodeStatus tick() override
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
        
        NavigateToPose::Goal goal_msg;
        goal_msg.pose = goal.value();
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [](const GoalHandleNavToPose::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                RCLCPP_INFO(rclcpp::get_logger("p2p_nav_bt_node"), "Navigation Succeeded!");
        };
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
        return BT::NodeStatus::SUCCESS;
    }
    
    void halt() override
    {
        RCLCPP_INFO(node_->get_logger(), "Halting CustomNavToPose BT Node.");
    }
    
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

// Register the custom node with BehaviorTree.CPP
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<CustomNavToPose>("CustomNavToPose");
}