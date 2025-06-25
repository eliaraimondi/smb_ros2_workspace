#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vector>
#include <chrono>

enum class ExplorationState {
    IDLE,
    EXPLORING,
    NAVIGATING_TO_OBJECT,
    CAPTURING_IMAGES,
    GOING_HOME,
    MISSION_COMPLETE
};

struct DetectedObject {
    int id;
    geometry_msgs::msg::Point location;
    bool visited;
    bool images_captured;
};

class ExplorationManager : public rclcpp::Node
{
public:
    ExplorationManager() : Node("exploration_manager"), 
                          current_state_(ExplorationState::IDLE),
                          mission_start_time_(this->now()),
                          max_mission_time_(std::chrono::minutes(2))
    {
        // Publishers
        exploration_command_pub_ = this->create_publisher<std_msgs::msg::String>("/exploration_command", 10);
        navigation_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/navigation_goal", 10);
        image_capture_pub_ = this->create_publisher<std_msgs::msg::Empty>("/capture_images", 10);
        go_home_pub_ = this->create_publisher<std_msgs::msg::Empty>("/go_home", 10);
        waypoint_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 10);
        
        // Subscribers  
        mission_complete_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mission_complete", 10,
            std::bind(&ExplorationManager::mission_complete_callback, this, std::placeholders::_1));
            
        detected_object_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/detected_objects", 10,
            std::bind(&ExplorationManager::detected_object_callback, this, std::placeholders::_1));
            
        navigation_status_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "/navigation_status", 10,
            std::bind(&ExplorationManager::navigation_status_callback, this, std::placeholders::_1));
            
        image_capture_status_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/image_capture_complete", 10,
            std::bind(&ExplorationManager::image_capture_status_callback, this, std::placeholders::_1));

        // Timer to check mission time and state
        state_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ExplorationManager::state_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Exploration Manager Node started");
        
        // Start exploration automatically
        start_exploration();
    }

private:
    // State variables
    ExplorationState current_state_;
    rclcpp::Time mission_start_time_;
    std::chrono::minutes max_mission_time_;
    std::vector<DetectedObject> detected_objects_;
    int current_object_index_ = -1;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr exploration_command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr navigation_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr image_capture_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr go_home_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_complete_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr detected_object_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navigation_status_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr image_capture_status_subscriber_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr state_timer_;

    void start_exploration()
    {
        RCLCPP_INFO(this->get_logger(), "Starting exploration mission!");
        current_state_ = ExplorationState::EXPLORING;
        mission_start_time_ = this->now();
        
        // Send exploration command
        auto msg = std_msgs::msg::String();
        msg.data = "start_exploration";
        exploration_command_pub_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Published exploration start command");
    }

    void detected_object_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Check if this object is already in our list (simple distance check)
        bool is_new_object = true;
        for (const auto& obj : detected_objects_) {
            double distance = std::sqrt(
                std::pow(obj.location.x - msg->point.x, 2) +
                std::pow(obj.location.y - msg->point.y, 2)
            );
            if (distance < 1.0) { // Consider objects within 1m as the same
                is_new_object = false;
                break;
            }
        }
        
        if (is_new_object) {
            DetectedObject new_obj;
            new_obj.id = detected_objects_.size();
            new_obj.location = msg->point;
            new_obj.visited = false;
            new_obj.images_captured = false;
            
            detected_objects_.push_back(new_obj);
            
            RCLCPP_INFO(this->get_logger(), "New object detected at (%.2f, %.2f, %.2f) - Object ID: %d",
                       msg->point.x, msg->point.y, msg->point.z, new_obj.id);
            
            // If we're exploring and not currently handling an object, go to this one
            if (current_state_ == ExplorationState::EXPLORING) {
                navigate_to_next_object();
            }
        }
    }

    void navigate_to_next_object()
    {
        // Find next unvisited object
        for (size_t i = 0; i < detected_objects_.size(); ++i) {
            if (!detected_objects_[i].visited) {
                current_object_index_ = i;
                current_state_ = ExplorationState::NAVIGATING_TO_OBJECT;
                
                // Send navigation goal
                auto nav_msg = geometry_msgs::msg::PoseStamped();
                nav_msg.header.stamp = this->now();
                nav_msg.header.frame_id = "map";
                nav_msg.pose.position = detected_objects_[i].location;
                nav_msg.pose.orientation.w = 1.0; // Default orientation
                
                navigation_goal_pub_->publish(nav_msg);
                
                RCLCPP_INFO(this->get_logger(), "Navigating to object %ld at (%.2f, %.2f, %.2f)",
                           i, detected_objects_[i].location.x, 
                           detected_objects_[i].location.y, 
                           detected_objects_[i].location.z);
                return;
            }
        }
        
        // No more unvisited objects, continue exploration
        current_state_ = ExplorationState::EXPLORING;
        auto msg = std_msgs::msg::String();
        msg.data = "continue_exploration";
        exploration_command_pub_->publish(msg);
    }

    void navigation_status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (current_state_ == ExplorationState::NAVIGATING_TO_OBJECT && msg->data == "goal_reached") {
            RCLCPP_INFO(this->get_logger(), "Reached object %d, starting image capture", current_object_index_);
            current_state_ = ExplorationState::CAPTURING_IMAGES;
            
            // Start image capture
            auto capture_msg = std_msgs::msg::Empty();
            image_capture_pub_->publish(capture_msg);
            
            // Mark as visited
            if (current_object_index_ >= 0 && current_object_index_ < static_cast<int>(detected_objects_.size())) {
                detected_objects_[current_object_index_].visited = true;
            }
        }
    }

    void image_capture_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (current_state_ == ExplorationState::CAPTURING_IMAGES && msg->data == true) {
            RCLCPP_INFO(this->get_logger(), "Image capture complete for object %d", current_object_index_);
            
            // Mark images as captured
            if (current_object_index_ >= 0 && current_object_index_ < static_cast<int>(detected_objects_.size())) {
                detected_objects_[current_object_index_].images_captured = true;
            }
            
            // Look for next object or continue exploration
            navigate_to_next_object();
        }
    }

    void state_timer_callback()
    {
        // Check if mission time exceeded
        auto elapsed_time = this->now() - mission_start_time_;
        if (elapsed_time.seconds() >= max_mission_time_.count() * 60) {
            if (current_state_ != ExplorationState::GOING_HOME && 
                current_state_ != ExplorationState::MISSION_COMPLETE) {
                RCLCPP_WARN(this->get_logger(), "Mission time limit reached (25 min), returning home");
                go_home();
            }
        }
        
        // Log current state periodically
        static int log_counter = 0;
        if (++log_counter >= 30) { // Log every 30 seconds
            log_counter = 0;
            RCLCPP_INFO(this->get_logger(), "Current state: %s, Objects found: %zu, Time elapsed: %.1f min",
                       state_to_string(current_state_).c_str(), 
                       detected_objects_.size(),
                       elapsed_time.seconds() / 60.0);
        }
    }

    void go_home()
    {
        current_state_ = ExplorationState::GOING_HOME;
        RCLCPP_INFO(this->get_logger(), "Mission complete! Going home...");
        
        // Send go home command
        auto msg = std_msgs::msg::Empty();
        go_home_pub_->publish(msg);
        
        // Also publish waypoint to home (0,0,0)
        auto waypoint_msg = geometry_msgs::msg::PointStamped();
        waypoint_msg.header.stamp = this->now();
        waypoint_msg.header.frame_id = "map";
        waypoint_msg.point.x = 0.0;
        waypoint_msg.point.y = 0.0;
        waypoint_msg.point.z = 0.0;
        waypoint_pub_->publish(waypoint_msg);
        
        // Log mission summary
        int visited_objects = 0;
        int captured_images = 0;
        for (const auto& obj : detected_objects_) {
            if (obj.visited) visited_objects++;
            if (obj.images_captured) captured_images++;
        }
        
        RCLCPP_INFO(this->get_logger(), "Mission Summary: %d objects detected, %d visited, %d with images captured",
                   static_cast<int>(detected_objects_.size()), visited_objects, captured_images);
    }

    void mission_complete_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == true && current_state_ == ExplorationState::GOING_HOME) {
            current_state_ = ExplorationState::MISSION_COMPLETE;
            RCLCPP_INFO(this->get_logger(), "Robot returned home successfully! Mission complete.");
        }
    }

    std::string state_to_string(ExplorationState state)
    {
        switch (state) {
            case ExplorationState::IDLE: return "IDLE";
            case ExplorationState::EXPLORING: return "EXPLORING";
            case ExplorationState::NAVIGATING_TO_OBJECT: return "NAVIGATING_TO_OBJECT";
            case ExplorationState::CAPTURING_IMAGES: return "CAPTURING_IMAGES";
            case ExplorationState::GOING_HOME: return "GOING_HOME";
            case ExplorationState::MISSION_COMPLETE: return "MISSION_COMPLETE";
            default: return "UNKNOWN";
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ExplorationManager>();
    
    RCLCPP_INFO(node->get_logger(), "Spinning exploration manager node...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}