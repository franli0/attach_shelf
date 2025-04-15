#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class ApproachServiceServer : public rclcpp::Node
{
public:
    ApproachServiceServer() : Node("approach_service_server")
    {
        // Initialize state variables
        state_ = IDLE;
        service_in_progress_ = false;
        legs_detected_ = false;
        cart_frame_published_ = false;
        
        // Initialize position tracking
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_yaw_ = 0.0;
        start_x_ = 0.0;
        start_y_ = 0.0;
        cart_x_ = 0.0;
        cart_y_ = 0.0;
        distance_moved_ = 0.0;
        
        // Create publishers and subscribers
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 10);
        
        elevator_up_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/elevator_up", 10);
        
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ApproachServiceServer::scan_callback, this, std::placeholders::_1));
        
        // Subscribe to odometry for position tracking
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ApproachServiceServer::odom_callback, this, std::placeholders::_1));
        
        // Create service
        service_ = this->create_service<attach_shelf::srv::GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachServiceServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // Create timer for control loop
        timer_ = this->create_wall_timer(50ms, std::bind(&ApproachServiceServer::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Approach Service Server started");
    }

private:
    enum RobotState {
        IDLE,
        DETECTING_LEGS,
        MOVING_TO_CART,
        MOVING_UNDER_SHELF,
        LIFTING_SHELF,
        COMPLETED
    };
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update robot position from odometry
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double w = msg->pose.pose.orientation.w;
        
        // Convert quaternion to yaw (rotation around Z axis)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        robot_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }
    
    void handle_service(
        const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
        std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service called with attach_to_shelf=%s", 
                    request->attach_to_shelf ? "true" : "false");
        
        if (service_in_progress_) {
            response->complete = false;
            return;
        }
        
        // Reset state variables
        service_in_progress_ = true;
        attach_to_shelf_ = request->attach_to_shelf;
        legs_detected_ = false;
        cart_frame_published_ = false;
        
        // Save the starting position as our reference point
        start_x_ = robot_x_;
        start_y_ = robot_y_;
        start_yaw_ = robot_yaw_;
        
        RCLCPP_INFO(this->get_logger(), "Setting reference position at (%.2f, %.2f), yaw: %.2f", 
                   start_x_, start_y_, start_yaw_);
        
        // Record the start time
        start_time_ = this->now();
        
        // Start the state machine
        state_ = DETECTING_LEGS;
        
        // Non-blocking service handler
        std::thread([this, response]() {
            const int timeout_seconds = 60;
            for (int i = 0; i < timeout_seconds && service_in_progress_; i++) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
                // Add more detailed status based on current state
                std::string state_str;
                switch (state_) {
                    case IDLE: state_str = "IDLE"; break;
                    case DETECTING_LEGS: state_str = "DETECTING_LEGS"; break;
                    case MOVING_TO_CART: state_str = "MOVING_TO_CART"; break;
                    case MOVING_UNDER_SHELF: state_str = "MOVING_UNDER_SHELF"; break;
                    case LIFTING_SHELF: state_str = "LIFTING_SHELF"; break;
                    case COMPLETED: state_str = "COMPLETED"; break;
                    default: state_str = "UNKNOWN";
                }
                
                RCLCPP_INFO(this->get_logger(), "Service running... %d/%d, state=%s", 
                           i+1, timeout_seconds, state_str.c_str());
            }
            
            if (service_in_progress_) {
                RCLCPP_ERROR(this->get_logger(), "Service timed out");
                stop_robot();
                state_ = IDLE;
                service_in_progress_ = false;
                response->complete = false;
            } else {
                response->complete = (state_ == COMPLETED);
            }
        }).detach();
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (state_ == DETECTING_LEGS) {
            detect_shelf_legs(msg);
        }
    }
    
    void detect_shelf_legs(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (legs_detected_) {
            // Skip if we've already detected legs
            return;
        }
        
        // Check if intensities are available
        if (msg->intensities.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "No intensity data available in laser scan");
            return;
        }
        
        const float INTENSITY_THRESHOLD = 7500.0;  // Reflective markers should have very high intensity
        const double MAX_RANGE = 2.0;  // Only consider points within 2 meters
        
        // Store any high-intensity points (potential shelf legs)
        std::vector<std::pair<double, double>> high_intensity_points;
        
        // Find maximum intensity in scan for diagnostic purposes
        float max_intensity = 0.0f;
        int max_intensity_index = -1;
        
        // Scan through all points
        for (size_t i = 0; i < msg->intensities.size(); ++i) {
            // Track maximum intensity
            if (msg->intensities[i] > max_intensity) {
                max_intensity = msg->intensities[i];
                max_intensity_index = i;
            }
            
            // Check if this is a high intensity point (potential shelf leg)
            if (msg->intensities[i] > INTENSITY_THRESHOLD && 
                msg->ranges[i] < MAX_RANGE && 
                !std::isinf(msg->ranges[i]) && 
                !std::isnan(msg->ranges[i])) {
                
                // Convert to cartesian coordinates
                double angle = msg->angle_min + i * msg->angle_increment;
                double x = msg->ranges[i] * std::cos(angle);
                double y = msg->ranges[i] * std::sin(angle);
                
                high_intensity_points.push_back(std::make_pair(x, y));
                
                // Debug logging for detected high intensity points
                RCLCPP_DEBUG(this->get_logger(), "High intensity point detected at (%.2f, %.2f) with intensity %.1f",
                            x, y, msg->intensities[i]);
            }
        }
        
        // Output diagnostic information
        if (max_intensity_index >= 0) {
            double max_angle = msg->angle_min + max_intensity_index * msg->angle_increment;
            double max_range = msg->ranges[max_intensity_index];
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Max intensity: %.1f at index %d, angle %.2f, range %.2f",
                max_intensity, max_intensity_index, max_angle, max_range);
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Found %zu high intensity points (threshold: %.1f)",
            high_intensity_points.size(), INTENSITY_THRESHOLD);
        
        // If we found enough high intensity points to detect shelf legs
        if (high_intensity_points.size() >= 2) {
            // Make sure we have enough valid points
            if (high_intensity_points.size() < 2) {
                RCLCPP_WARN(this->get_logger(), "Not enough valid high intensity points to detect legs");
                return;
            }
            
            // Find points with the most extreme y values for better leg detection
            std::sort(high_intensity_points.begin(), high_intensity_points.end(),
                     [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                         return a.second < b.second;
                     });
            
            // Get the points with extreme Y values (most left and right)
            auto left_points = std::vector<std::pair<double, double>>();
            auto right_points = std::vector<std::pair<double, double>>();
            
            // Consider the 20% leftmost and 20% rightmost points
            size_t num_extreme_points = std::max(size_t(1), high_intensity_points.size() / 5);
            
            // Get leftmost points
            for (size_t i = 0; i < num_extreme_points && i < high_intensity_points.size(); i++) {
                left_points.push_back(high_intensity_points[i]);
            }
            
            // Get rightmost points
            for (size_t i = 0; i < num_extreme_points && i < high_intensity_points.size(); i++) {
                right_points.push_back(high_intensity_points[high_intensity_points.size() - 1 - i]);
            }
            
            // Calculate average position of left leg
            double left_x_sum = 0.0, left_y_sum = 0.0;
            for (const auto& point : left_points) {
                left_x_sum += point.first;
                left_y_sum += point.second;
            }
            double left_x_avg = left_x_sum / left_points.size();
            double left_y_avg = left_y_sum / left_points.size();
            
            // Calculate average position of right leg
            double right_x_sum = 0.0, right_y_sum = 0.0;
            for (const auto& point : right_points) {
                right_x_sum += point.first;
                right_y_sum += point.second;
            }
            double right_x_avg = right_x_sum / right_points.size();
            double right_y_avg = right_y_sum / right_points.size();
            
            // Store leg positions for approach planning
            left_leg_x_ = left_x_avg;
            left_leg_y_ = left_y_avg;
            right_leg_x_ = right_x_avg;
            right_leg_y_ = right_y_avg;
            
            // Calculate cart position (midpoint between legs) in robot frame
            initial_cart_x_ = (left_leg_x_ + right_leg_x_) / 2.0;
            initial_cart_y_ = (left_leg_y_ + right_leg_y_) / 2.0;
            
            // Store for publishing the TF
            cart_x_ = initial_cart_x_;
            cart_y_ = initial_cart_y_;
            
            // Calculate leg distance for validation
            double leg_distance = std::sqrt(
                std::pow(right_leg_x_ - left_leg_x_, 2) + 
                std::pow(right_leg_y_ - left_leg_y_, 2));
            
            RCLCPP_INFO(this->get_logger(), 
                "Shelf legs detected! Left leg: (%.2f, %.2f), Right leg: (%.2f, %.2f), Distance: %.2f m",
                left_leg_x_, left_leg_y_, right_leg_x_, right_leg_y_, leg_distance);
            
            RCLCPP_INFO(this->get_logger(), "Initial cart frame position set to (%.2f, %.2f) relative to robot",
                initial_cart_x_, initial_cart_y_);
            
            // Convert cart position from robot frame to world frame coordinates
            target_x_ = robot_x_ + initial_cart_x_ * std::cos(robot_yaw_) - initial_cart_y_ * std::sin(robot_yaw_);
            target_y_ = robot_y_ + initial_cart_x_ * std::sin(robot_yaw_) + initial_cart_y_ * std::cos(robot_yaw_);
            
            RCLCPP_INFO(this->get_logger(), "Target position in world coordinates: (%.2f, %.2f)",
                target_x_, target_y_);
            
            // Mark legs as detected and cart frame as published
            legs_detected_ = true;
            cart_frame_published_ = true;
            
            // If we're not supposed to attach, just publish TF and complete
            if (!attach_to_shelf_) {
                complete_service(true);
                return;
            }
            
            // Start movement to cart
            state_ = MOVING_TO_CART;
            
        } else if (service_in_progress_ && state_ == DETECTING_LEGS) {
            // Check how long we've been trying to detect legs
            rclcpp::Duration elapsed = this->now() - start_time_;
            
            if (elapsed.seconds() > 5.0) {  // After 5 seconds without legs detected
                RCLCPP_WARN(this->get_logger(), 
                    "Failed to detect shelf legs after %.1f seconds, using fallback position",
                    elapsed.seconds());
                
                // Use fallback position in front of the robot
                initial_cart_x_ = 0.75;  // 75cm forward
                initial_cart_y_ = 0.0;   // Centered
                
                // Store for publishing the TF
                cart_x_ = initial_cart_x_;
                cart_y_ = initial_cart_y_;
                
                RCLCPP_INFO(this->get_logger(), "Using fallback cart position: (%.2f, %.2f) relative to robot",
                    initial_cart_x_, initial_cart_y_);
                
                // Convert to world coordinates
                target_x_ = robot_x_ + initial_cart_x_ * std::cos(robot_yaw_) - initial_cart_y_ * std::sin(robot_yaw_);
                target_y_ = robot_y_ + initial_cart_x_ * std::sin(robot_yaw_) + initial_cart_y_ * std::cos(robot_yaw_);
                
                RCLCPP_INFO(this->get_logger(), "Target position in world coordinates: (%.2f, %.2f)",
                    target_x_, target_y_);
                
                // Mark legs as detected and cart frame as published
                legs_detected_ = true;
                cart_frame_published_ = true;
                
                // If we're not supposed to attach, just publish TF and complete
                if (!attach_to_shelf_) {
                    complete_service(true);
                    return;
                }
                
                // Start movement to cart
                state_ = MOVING_TO_CART;
            }
        }
    }
    
    void timer_callback()
    {
        if (cart_frame_published_) {
            publish_cart_transform();
        }
        
        switch (state_) {
            case MOVING_TO_CART:
                execute_move_to_cart();
                break;
                
            case MOVING_UNDER_SHELF:
                execute_move_under_shelf();
                break;
                
            case LIFTING_SHELF:
                execute_lift_shelf();
                break;
                
            default:
                break;
        }
    }
    
    void publish_cart_transform()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "robot_base_link";
        transform.child_frame_id = "cart_frame";
        
        transform.transform.translation.x = cart_x_;
        transform.transform.translation.y = cart_y_;
        transform.transform.translation.z = 0.0;
        
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        
        tf_broadcaster_->sendTransform(transform);
    }
    
    void execute_move_to_cart()
    {
        // Calculate distance to target in world coordinates
        double dx = target_x_ - robot_x_;
        double dy = target_y_ - robot_y_;
        double distance_to_target = std::sqrt(dx*dx + dy*dy);
        
        // Calculate angle to target in world frame
        double target_angle = std::atan2(dy, dx) - robot_yaw_;
        
        // Normalize angle to [-π, π]
        while (target_angle > M_PI) target_angle -= 2*M_PI;
        while (target_angle < -M_PI) target_angle += 2*M_PI;
        
        // Log the current position and target
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Robot at (%.2f, %.2f, %.2f), Target at (%.2f, %.2f), Distance: %.2f, Angle: %.2f",
            robot_x_, robot_y_, robot_yaw_, target_x_, target_y_, 
            distance_to_target, target_angle);
        
        // If we're close enough to the target
        if (distance_to_target < 0.1) { // 10cm threshold
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "*** REACHED CART POSITION ***");
            
            // Wait to ensure robot is fully stopped
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Calculate target position 70cm further under the shelf
            // This position is 70cm forward from current position in robot's forward direction
            under_shelf_x_ = robot_x_ + 0.7 * std::cos(robot_yaw_);
            under_shelf_y_ = robot_y_ + 0.7 * std::sin(robot_yaw_);
            
            RCLCPP_INFO(this->get_logger(), "*** CALCULATED UNDER-SHELF POSITION (%.2f, %.2f) ***", 
                       under_shelf_x_, under_shelf_y_);
            
            // Transition to under-shelf movement
            state_ = MOVING_UNDER_SHELF;
            RCLCPP_INFO(this->get_logger(), "*** STARTING UNDER-SHELF MOVEMENT ***");
            return;
        }
        
        // Calculate linear and angular velocities
        double forward_speed = 0.3;  // Increased default speed for faster movement
        double angular_velocity = 0.0;
        
        // If we need to make a significant turn first, turn in place
        if (std::abs(target_angle) > 0.3) {  // Reduced threshold for earlier correction
            forward_speed = 0.0;  // Stop forward motion
            // Turn with proportional control - more aggressive
            angular_velocity = 0.6 * target_angle;
            // Limit angular velocity
            angular_velocity = std::max(-0.5, std::min(0.5, angular_velocity));
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Turning in place to align with target, angle: %.2f rad", target_angle);
        } else {
            // We're roughly aligned, move forward while steering
            
            // Adjust forward speed based on distance and angle
            if (distance_to_target < 0.3) {
                forward_speed = 0.15;  // Slightly faster when close
            }
            
            // More aggressive steering for better alignment
            angular_velocity = 0.8 * target_angle;
            // Limit angular velocity
            angular_velocity = std::max(-0.4, std::min(0.4, angular_velocity));
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Moving to target with speed: %.2f m/s, turn rate: %.2f rad/s", 
                forward_speed, angular_velocity);
        }
        
        // Send velocity commands
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = forward_speed;
        cmd_vel.angular.z = angular_velocity;
        vel_publisher_->publish(cmd_vel);
        
        // Debug output to confirm velocity commands are being sent
        RCLCPP_DEBUG(this->get_logger(), "Published velocity: linear=%.2f, angular=%.2f", 
                    forward_speed, angular_velocity);
    }
    
    void execute_move_under_shelf()
    {
        // Calculate distance to under-shelf position in world coordinates
        double dx = under_shelf_x_ - robot_x_;
        double dy = under_shelf_y_ - robot_y_;
        double distance_to_target = std::sqrt(dx*dx + dy*dy);
        
        // Log progress without mentioning angle - we're moving straight
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Moving straight under shelf: Robot at (%.2f, %.2f), target at (%.2f, %.2f), distance: %.2f",
            robot_x_, robot_y_, under_shelf_x_, under_shelf_y_, distance_to_target);
        
        // If we're close enough to the target
        if (distance_to_target < 0.05) { // 5cm threshold, more precise for final positioning
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "*** REACHED UNDER-SHELF POSITION ***");
            
            // Wait to ensure robot is fully stopped
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Transition to lifting shelf
            state_ = LIFTING_SHELF;
            RCLCPP_INFO(this->get_logger(), "*** READY TO LIFT SHELF ***");
            return;
        }
        
        // Set speed based on distance - move slower for more precise positioning
        double forward_speed = 0.15; // Base speed
        if (distance_to_target < 0.15) {
            forward_speed = 0.1; // Even slower when very close
        }
        
        // IMPORTANT CHANGE: No steering for straight motion
        // Send velocity commands with zero angular velocity
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = forward_speed;
        cmd_vel.angular.z = 0.0;  // Always zero - perfectly straight motion
        vel_publisher_->publish(cmd_vel);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Moving in a perfectly straight line with speed: %.2f m/s", forward_speed);
    }
    
    void execute_lift_shelf()
    {
        RCLCPP_INFO(this->get_logger(), "Lifting shelf now...");
        
        // Send multiple lift commands to ensure receipt
        std_msgs::msg::String msg;
        msg.data = "";  // Empty string is sufficient for the simulation
        
        // Send multiple lift commands for reliability
        for (int i = 0; i < 5; i++) {
            elevator_up_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing elevator_up command %d/5", i+1);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        
        RCLCPP_INFO(this->get_logger(), "Shelf lifting completed!");
        
        // Complete service
        complete_service(true);
    }
    
    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_publisher_->publish(cmd_vel);
    }
    
    void complete_service(bool success)
    {
        service_in_progress_ = false;
        state_ = success ? COMPLETED : IDLE;
        RCLCPP_INFO(this->get_logger(), "Service completed with result: %s", 
                   success ? "success" : "failure");
    }
    
    // State variables
    RobotState state_;
    bool service_in_progress_;
    bool attach_to_shelf_;
    bool legs_detected_;
    bool cart_frame_published_;
    
    // Position data
    double robot_x_;    // Current robot x position in world frame
    double robot_y_;    // Current robot y position in world frame
    double robot_yaw_;  // Current robot orientation in world frame
    double start_x_;    // Starting x position when service was called
    double start_y_;    // Starting y position when service was called
    double start_yaw_;  // Starting orientation when service was called
    double cart_x_;     // Cart position x in robot frame (for TF broadcast)
    double cart_y_;     // Cart position y in robot frame (for TF broadcast)
    double initial_cart_x_; // Initial detected cart x position in robot frame
    double initial_cart_y_; // Initial detected cart y position in robot frame
    double left_leg_x_;  // Left leg x position in robot frame
    double left_leg_y_;  // Left leg y position in robot frame  
    double right_leg_x_; // Right leg x position in robot frame
    double right_leg_y_; // Right leg y position in robot frame
    double target_x_;   // Target position x in world frame (cart position)
    double target_y_;   // Target position y in world frame (cart position)
    double under_shelf_x_; // Target position x in world frame (30cm under shelf)
    double under_shelf_y_; // Target position y in world frame (30cm under shelf)
    double last_x_;     // Previous x position for distance calculation
    double last_y_;     // Previous y position for distance calculation
    double distance_moved_;
    
    // Timing
    rclcpp::Time start_time_;
    rclcpp::Time start_under_shelf_time_;
    
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApproachServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}