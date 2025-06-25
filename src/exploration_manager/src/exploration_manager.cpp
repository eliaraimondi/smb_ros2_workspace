#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum class RobotState {
    WAITING_FOR_INITIAL_POSE,
    MOVING_FORWARD_INITIAL,
    SAVING_INITIAL_POSE,
    EXPLORING,
    RETURNING_HOME,
    MOVING_BACKWARD_FINAL,
    MAINTAINING_START_POSITION,
    COMPLETE
};

class ExplorationManager : public rclcpp::Node
{
public:
    ExplorationManager() : Node("exploration_manager"), current_state_(RobotState::WAITING_FOR_INITIAL_POSE), initial_pose_received_(false)
    {
        // Create publisher for /goal_point topic
        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/way_point", 10);
        
        // Create publisher for /start_exploration topic
        start_exploration_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/start_exploration", 10);
        
        // Create subscriber for /exploration_finish topic
        exploration_finish_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/exploration_finish", 10,
            std::bind(&ExplorationManager::exploration_finish_callback, this, std::placeholders::_1));
        
        // Create subscriber for current robot pose from state estimation
        pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/state_estimation", 10,
            std::bind(&ExplorationManager::state_estimation_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "ExplorationManager Node started");
        RCLCPP_INFO(this->get_logger(), "Waiting for initial robot pose from /state_estimation...");
        RCLCPP_INFO(this->get_logger(), "Will return home on: timeout (60s) OR /exploration_finish=true");
        RCLCPP_INFO(this->get_logger(), "Note: Robot will end at starting position with 180° rotated orientation");
    }

private:
    // Helper function to extract yaw angle from quaternion
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
    {
        // Convert quaternion to yaw angle
        return atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }
    
    void state_estimation_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract pose from odometry message
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;
        
        // If we're waiting for initial pose, start the forward movement sequence
        if (current_state_ == RobotState::WAITING_FOR_INITIAL_POSE && !initial_pose_received_)
        {
            initial_pose_received_ = true;
            starting_pose_ = current_pose_;
            
            RCLCPP_INFO(this->get_logger(), "Initial pose received: (%.2f, %.2f, %.2f)", 
                        starting_pose_.pose.position.x, 
                        starting_pose_.pose.position.y, 
                        starting_pose_.pose.position.z);
            
            start_initial_forward_movement();
        }
        
        // Check if robot has reached the forward target position
        if (current_state_ == RobotState::MOVING_FORWARD_INITIAL)
        {
            double distance_to_target = sqrt(
                pow(current_pose_.pose.position.x - forward_target_position_.x, 2) +
                pow(current_pose_.pose.position.y - forward_target_position_.y, 2) +
                pow(current_pose_.pose.position.z - forward_target_position_.z, 2)
            );
            
            RCLCPP_DEBUG(this->get_logger(), "Distance to forward target: %.3f meters", distance_to_target);
            
            if (distance_to_target < 0.2)  // 20cm tolerance
            {
                RCLCPP_INFO(this->get_logger(), "Reached forward target position! Distance: %.3f m", distance_to_target);
                
                // Stop continuous publishing
                if (forward_goal_timer_)
                {
                    forward_goal_timer_->cancel();
                }
                
                save_initial_pose_and_start_exploration();
            }
        }
        
        // Check if robot has reached the return home position
        if (current_state_ == RobotState::RETURNING_HOME)
        {
            double distance_to_home = sqrt(
                pow(current_pose_.pose.position.x - initial_pose_.pose.position.x, 2) +
                pow(current_pose_.pose.position.y - initial_pose_.pose.position.y, 2) +
                pow(current_pose_.pose.position.z - initial_pose_.pose.position.z, 2)
            );
            
            RCLCPP_DEBUG(this->get_logger(), "Distance to home position: %.3f meters", distance_to_home);
            
            if (distance_to_home < 0.3)  // 30cm tolerance for return home
            {
                RCLCPP_INFO(this->get_logger(), "Reached home position! Distance: %.3f m", distance_to_home);
                start_final_backward_movement();
            }
        }
        
        // Check if robot has reached the starting position (with 180° rotated orientation)
        if (current_state_ == RobotState::MOVING_BACKWARD_FINAL)
        {
            double distance_to_start = sqrt(
                pow(current_pose_.pose.position.x - starting_pose_.pose.position.x, 2) +
                pow(current_pose_.pose.position.y - starting_pose_.pose.position.y, 2) +
                pow(current_pose_.pose.position.z - starting_pose_.pose.position.z, 2)
            );
            
            RCLCPP_DEBUG(this->get_logger(), "Distance to starting position: %.3f meters", distance_to_start);
            
            if (distance_to_start < 0.15)  // 15cm tolerance - only check position, not orientation
            {
                RCLCPP_INFO(this->get_logger(), "Reached starting position! Distance: %.3f m (orientation is 180° rotated)", distance_to_start);
                start_maintaining_position();
            }
        }
    }
    
    void start_initial_forward_movement()
    {
        RCLCPP_INFO(this->get_logger(), "Starting initial forward movement (1 meter)");
        current_state_ = RobotState::MOVING_FORWARD_INITIAL;
        
        // Get yaw angle from current pose quaternion
        double yaw = getYawFromQuaternion(starting_pose_.pose.orientation);
        
        // Calculate target position 1 meter forward from current position
        forward_target_position_.x = starting_pose_.pose.position.x + 1.0 * cos(yaw);
        forward_target_position_.y = starting_pose_.pose.position.y + 1.0 * sin(yaw);
        forward_target_position_.z = starting_pose_.pose.position.z;
        
        RCLCPP_INFO(this->get_logger(), "Forward target calculated: (%.2f, %.2f, %.2f) based on yaw: %.2f rad", 
                    forward_target_position_.x, forward_target_position_.y, forward_target_position_.z, yaw);
        
        // Start continuous publishing of forward goal (5 Hz)
        forward_goal_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&ExplorationManager::publish_forward_goal, this));
        
        RCLCPP_INFO(this->get_logger(), "Started continuous forward goal publishing at 5 Hz");
    }
    
    void publish_forward_goal()
    {
        auto forward_goal = geometry_msgs::msg::PointStamped();
        forward_goal.header.stamp = this->now();
        forward_goal.header.frame_id = "map";
        forward_goal.point = forward_target_position_;
        
        goal_publisher_->publish(forward_goal);
        
        RCLCPP_DEBUG(this->get_logger(), "Published forward goal: (%.2f, %.2f, %.2f)", 
                    forward_goal.point.x, forward_goal.point.y, forward_goal.point.z);
    }
    
    void save_initial_pose_and_start_exploration()
    {
        RCLCPP_INFO(this->get_logger(), "Saving initial pose and starting exploration");
        current_state_ = RobotState::EXPLORING;
        
        // Save the current pose as initial pose (after moving 1m forward)
        initial_pose_ = current_pose_;
        
        RCLCPP_INFO(this->get_logger(), "Initial pose saved: (%.2f, %.2f, %.2f)", 
                    initial_pose_.pose.position.x, 
                    initial_pose_.pose.position.y, 
                    initial_pose_.pose.position.z);
        
        // Start exploration timeout timer
        timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(60),
            std::bind(&ExplorationManager::timeout_callback, this));
        
        // Signal the tare planner to start exploration
        auto start_exploration_msg = std_msgs::msg::Bool();
        start_exploration_msg.data = true;
        start_exploration_publisher_->publish(start_exploration_msg);
        
        RCLCPP_INFO(this->get_logger(), "Published start_exploration = true");
        RCLCPP_INFO(this->get_logger(), "Exploration phase started with 60-second timeout");
    }
    
    void exploration_finish_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data == true && current_state_ == RobotState::EXPLORING)
        {
            RCLCPP_INFO(this->get_logger(), "Exploration finish received (true) - exploration completed!");
            start_returning_home("Exploration finished");
        }
        else if (msg->data == false && current_state_ == RobotState::EXPLORING)
        {
            RCLCPP_INFO(this->get_logger(), "Exploration finish received (false) - exploration still ongoing");
        }
        else if (current_state_ != RobotState::EXPLORING)
        {
            RCLCPP_DEBUG(this->get_logger(), "Exploration finish received but not in exploration state");
        }
    }
    
    void timeout_callback()
    {
        if (current_state_ == RobotState::EXPLORING)
        {
            RCLCPP_INFO(this->get_logger(), "Exploration timeout reached!");
            start_returning_home("Timeout");
        }
    }
    
    void start_returning_home(const std::string& reason)
    {
        RCLCPP_INFO(this->get_logger(), "Starting to return to initial pose due to: %s", reason.c_str());
        current_state_ = RobotState::RETURNING_HOME;
        
        // Cancel the timeout timer
        if (timeout_timer_)
        {
            timeout_timer_->cancel();
        }
        
        // Signal the tare planner to stop exploration
        auto stop_exploration_msg = std_msgs::msg::Bool();
        stop_exploration_msg.data = false;
        start_exploration_publisher_->publish(stop_exploration_msg);
        RCLCPP_INFO(this->get_logger(), "Published start_exploration = false (stopping exploration)");
        
        // Start continuous publishing timer to return to initial pose (10 Hz)
        continuous_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ExplorationManager::publish_return_home_goal, this));
        
        RCLCPP_INFO(this->get_logger(), "Started continuous return-home goal publishing at 10 Hz");
    }
    
    void publish_return_home_goal()
    {
        // Create and populate the PointStamped message to return to initial pose
        auto goalpoint_msg = geometry_msgs::msg::PointStamped();
        
        // Set header
        goalpoint_msg.header.stamp = this->now();
        goalpoint_msg.header.frame_id = "map";
        
        // Set point coordinates to initial saved pose
        goalpoint_msg.point.x = initial_pose_.pose.position.x;
        goalpoint_msg.point.y = initial_pose_.pose.position.y;
        goalpoint_msg.point.z = initial_pose_.pose.position.z;
        
        // Publish the message
        goal_publisher_->publish(goalpoint_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Published return-home goalpoint: (%.2f, %.2f, %.2f)",
                    goalpoint_msg.point.x, goalpoint_msg.point.y, goalpoint_msg.point.z);
    }
    
    void start_final_backward_movement()
    {
        RCLCPP_INFO(this->get_logger(), "Starting final forward movement back to starting position");
        current_state_ = RobotState::MOVING_BACKWARD_FINAL;
        
        // Cancel continuous timer
        if (continuous_timer_)
        {
            continuous_timer_->cancel();
        }
        
        // Target: move forward from initial_pose toward the starting position
        // Robot will arrive at starting position facing opposite direction (180° rotated)
        forward_to_start_target_ = starting_pose_.pose.position;
        
        RCLCPP_INFO(this->get_logger(), "Target: starting position (%.2f, %.2f, %.2f) - robot will arrive facing opposite direction",
                    forward_to_start_target_.x, forward_to_start_target_.y, forward_to_start_target_.z);
        
        // Start continuous publishing of forward-to-start goal (5 Hz)
        backward_goal_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&ExplorationManager::publish_forward_to_start_goal, this));
        
        RCLCPP_INFO(this->get_logger(), "Started continuous forward-to-start goal publishing at 5 Hz");
    }
    
    void publish_forward_to_start_goal()
    {
        auto forward_goal = geometry_msgs::msg::PointStamped();
        forward_goal.header.stamp = this->now();
        forward_goal.header.frame_id = "map";
        forward_goal.point = forward_to_start_target_;
        
        goal_publisher_->publish(forward_goal);
        
        RCLCPP_DEBUG(this->get_logger(), "Published forward-to-start goal: (%.2f, %.2f, %.2f)", 
                    forward_goal.point.x, forward_goal.point.y, forward_goal.point.z);
    }
    
    void start_maintaining_position()
    {
        RCLCPP_INFO(this->get_logger(), "Sequence complete! Now maintaining starting position (with 180° rotated orientation)");
        current_state_ = RobotState::MAINTAINING_START_POSITION;
        
        // Cancel forward-to-start goal timer
        if (backward_goal_timer_)
        {
            backward_goal_timer_->cancel();
        }
        
        // Start continuous publishing of starting position to maintain position (2 Hz)
        maintain_position_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ExplorationManager::publish_maintain_position, this));
        
        RCLCPP_INFO(this->get_logger(), "Started maintaining position at starting pose - robot will stay here");
    }
    
    void publish_maintain_position()
    {
        auto point_goal = geometry_msgs::msg::PointStamped();
        point_goal.header.stamp = this->now();
        point_goal.header.frame_id = "map";
        point_goal.point = starting_pose_.pose.position;
        
        goal_publisher_->publish(point_goal);
        
        RCLCPP_DEBUG(this->get_logger(), "Maintaining position: (%.2f, %.2f, %.2f)", 
                    point_goal.point.x, point_goal.point.y, point_goal.point.z);
    }
    
    void sequence_complete()
    {
        // This function is now only called in error cases
        RCLCPP_WARN(this->get_logger(), "sequence_complete() called - this should not happen in normal operation");
        current_state_ = RobotState::COMPLETE;
        
        // Cancel any active timers
        if (backward_goal_timer_)
        {
            backward_goal_timer_->cancel();
        }
        if (maintain_position_timer_)
        {
            maintain_position_timer_->cancel();
        }
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_exploration_publisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_finish_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr continuous_timer_;
    rclcpp::TimerBase::SharedPtr forward_goal_timer_;
    rclcpp::TimerBase::SharedPtr backward_goal_timer_;
    rclcpp::TimerBase::SharedPtr maintain_position_timer_;
    
    RobotState current_state_;
    geometry_msgs::msg::PoseStamped starting_pose_;       // Robot's original pose when node starts
    geometry_msgs::msg::PoseStamped initial_pose_;        // Robot's pose after moving 1m forward (exploration start)
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Point forward_target_position_;   // Target position 1m forward from start
    geometry_msgs::msg::Point forward_to_start_target_;   // Target position: starting position (for return)
    bool initial_pose_received_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ExplorationManager>();
    
    RCLCPP_INFO(node->get_logger(), "Spinning ExplorationManager node...");
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}