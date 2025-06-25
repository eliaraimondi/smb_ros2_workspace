#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

class ExplorationManager : public rclcpp::Node
{
public:
    ExplorationManager() : Node("exploration_manager"), going_home_(false)
    {
        // Create publisher for /way_point topic
        exploration_manager_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/way_point", 10);
        
        // Create subscriber for /mission_complete topic
        mission_complete_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mission_complete", 10,
            std::bind(&ExplorationManager::mission_complete_callback, this, std::placeholders::_1));
        
        // Create timer that triggers after X seconds
        timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(120),
            std::bind(&ExplorationManager::timeout_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Waypoint Publisher Node started");
        RCLCPP_INFO(this->get_logger(), "Listening for mission_complete messages...");
        RCLCPP_INFO(this->get_logger(), "Timeout timer started");
    }

private:
    void mission_complete_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == true)
        {
            RCLCPP_INFO(this->get_logger(), "Mission complete received!");
            start_going_home("Mission complete");
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Mission not complete (received false)");
        }
    }
    
    void timeout_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Timeout reached!");
        start_going_home("Timeout");
    }
    
    void start_going_home(const std::string& reason)
    {
        // Prevent starting multiple home timers
        if (going_home_)
        {
            RCLCPP_INFO(this->get_logger(), "Already going home, ignoring %s trigger", reason.c_str());
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Starting to send robot home due to: %s", reason.c_str());
        
        // Mark that we're going home
        going_home_ = true;
        
        // Cancel the timeout timer to prevent further triggers
        if (timeout_timer_)
        {
            timeout_timer_->cancel();
        }
        
        // Start continuous publishing timer (10 Hz = every 100ms)
        continuous_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ExplorationManager::publish_home_waypoint, this));
        
        RCLCPP_INFO(this->get_logger(), "Started continuous home waypoint publishing at 10 Hz");
    }
    
    void publish_home_waypoint()
    {
        // Create and populate the PointStamped message
        auto waypoint_msg = geometry_msgs::msg::PointStamped();
        
        // Set header
        waypoint_msg.header.stamp = this->now();
        waypoint_msg.header.frame_id = "map";
        
        // Set point coordinates (home position)
        waypoint_msg.point.x = 0.0;
        waypoint_msg.point.y = 0.0;
        waypoint_msg.point.z = 0.0;
        
        // Publish the message
        exploration_manager_->publish(waypoint_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published home waypoint: (%.1f, %.1f, %.1f)",
                    waypoint_msg.point.x, waypoint_msg.point.y, waypoint_msg.point.z);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr exploration_manager_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_complete_subscriber_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr continuous_timer_;
    bool going_home_;
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