#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cstdlib>
#include <string>

class MissionSwitcher : public rclcpp::Node
{
public:
    MissionSwitcher() : Node("mission_switcher"), switched_(false)
    {
        sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/switch_to_navigation", 10,
            std::bind(&MissionSwitcher::callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Waiting for /switch_to_navigation...");
    }

private:
    void callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !switched_)
        {
            RCLCPP_INFO(this->get_logger(), "Switching from exploration to navigation...");

            // Stop explorer node (namespace is /explorer)
            std::system("ros2 node kill /explorer/explorer_node");

            // Launch far_planner node (adjust executable name if needed)
            std::system("ros2 run far_planner far_planner_node &");

            // Publish goal
            std::string goal_cmd =
                "ros2 topic pub /goal_point geometry_msgs/msg/PointStamped "
                "'{header: {frame_id: \"map\"}, point: {x: -3.0, y: 2.0, z: 0.0}}' --once";

            std::system(goal_cmd.c_str());

            switched_ = true;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
    bool switched_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MissionSwitcher>());
    rclcpp::shutdown();
    return 0;
}
