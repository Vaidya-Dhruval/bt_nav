#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "p2p_navigation_interfaces/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavigationClient : public rclcpp::Node
{
public:
    using NavigateToPose = p2p_navigation_interfaces::action::NavigateToPose;
    using Client = rclcpp_action::Client<NavigateToPose>;

    NavigationClient() : Node("navigation_client")
    {
        // Create the action client
        client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Wait for the action server to be up and running
        client_->wait_for_action_server();
    }

    void sendGoal(const geometry_msgs::msg::PoseStamped& pose, const std::string& behavior_tree)
    {
        // Create the goal message
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = pose;
        goal_msg.behavior_tree = behavior_tree;

        // Set up the callback options for handling the result
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;
        send_goal_options.result_callback = std::bind(&NavigationClient::resultCallback, this, std::placeholders::_1);

        // Send the goal to the action server
        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    // Callback for result
    void resultCallback(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Navigation failed");
        }
    }

    std::shared_ptr<Client> client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationClient>();

    // Send a sample goal
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 1.0;
    pose.pose.orientation.w = 1.0;

    node->sendGoal(pose, "SampleBehaviorTree");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
