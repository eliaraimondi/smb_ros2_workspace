#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

class ExplorationManager : public rclcpp::Node
{
public:
    ExplorationManager() : Node("exploration_manager")
    {
        // Create publisher for /way_point topic
        exploration_manager_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/way_point", 10);
        
        // Create subscriber for /mission_complete topic
        mission_complete_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mission_complete", 10,
            std::bind(&ExplorationManager::mission_complete_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Node started");
        RCLCPP_INFO(this->get_logger(), "Listening for mission_complete messages...");
    }

private:
    void mission_complete_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == true)
        {
            RCLCPP_INFO(this->get_logger(), "Mission complete received! Publishing waypoint...");
            
            // Create and populate the PointStamped message
            auto waypoint_msg = geometry_msgs::msg::PointStamped();
            
            // Set header
            waypoint_msg.header.stamp = this->now();
            waypoint_msg.header.frame_id = "map";
            
            // Set point coordinates
            waypoint_msg.point.x = 0.0;
            waypoint_msg.point.y = 0.0;
            waypoint_msg.point.z = 0.0;
            
            // Publish the message
            exploration_manager_->publish(waypoint_msg);
            
            RCLCPP_INFO(this->get_logger(), "Published waypoint: (%.1f, %.1f, %.1f) in frame '%s'",
                       waypoint_msg.point.x, waypoint_msg.point.y, waypoint_msg.point.z,
                       waypoint_msg.header.frame_id.c_str());
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Mission not complete (received false)");
        }
    }
    
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr exploration_manager_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_complete_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ExplorationManager>();
    
    RCLCPP_INFO(node->get_logger(), "Spinning waypoint publisher node...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}