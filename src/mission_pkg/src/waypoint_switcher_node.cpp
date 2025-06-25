#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class WaypointSwitcher : public rclcpp::Node {
public:
    WaypointSwitcher()
    : Node("waypoint_switcher_node"),
      obj_active_(false),
      timeout_sec_(3.0),
      error_threshold_(0.5)
    {
        mux_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 10);

        obj_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/obj_pos_topic", 10,
            std::bind(&WaypointSwitcher::objCallback, this, std::placeholders::_1));

        tare_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/tare_waypoint", 10,
            std::bind(&WaypointSwitcher::tareCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/state_estimation", 10,
            std::bind(&WaypointSwitcher::poseCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&WaypointSwitcher::timerCallback, this));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr mux_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obj_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tare_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PointStamped last_obj_pos_;
    geometry_msgs::msg::PointStamped last_tare_pos_;
    rclcpp::Time last_obj_time_;
    geometry_msgs::msg::Point current_pose_;
    bool obj_active_;
    double timeout_sec_;
    double error_threshold_;

    void objCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        last_obj_pos_ = *msg;
        last_obj_time_ = this->now();
        obj_active_ = true;
    }

    void tareCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        last_tare_pos_ = *msg;
    }

    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose.position;
    }

    void timerCallback() {
        if (obj_active_) {
            double dx = current_pose_.x - last_obj_pos_.point.x;
            double dy = current_pose_.y - last_obj_pos_.point.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance > error_threshold_) {
                mux_pub_->publish(last_obj_pos_);
            } else if ((this->now() - last_obj_time_).seconds() > timeout_sec_) {
                obj_active_ = false;
            }
        }

        if (!obj_active_) {
            mux_pub_->publish(last_tare_pos_);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointSwitcher>());
    rclcpp::shutdown();
    return 0;
}

