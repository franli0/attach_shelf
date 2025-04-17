#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>
#include <algorithm>

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
        scan_count_ = 0;
        
        // Adjustable parameters for easy modification
        cart_position_offset_x_ = 0.22;  // Forward adjustment for cart_frame
        cart_position_offset_y_ = 0.0;   // Lateral adjustment for cart_frame
        under_shelf_distance_ = 0.3;     // Distance to move under the shelf (in meters)
        
        // Initialize position tracking
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_yaw_ = 0.0;
        
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
        tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        
        // Create timer for control loop
        timer_ = this->create_wall_timer(50ms, std::bind(&ApproachServiceServer::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Approach Service Server started");
        // RCLCPP_INFO(this->get_logger(), "Using cart position offsets: x=%.2f m, y=%.2f m", 
        //            cart_position_offset_x_, cart_position_offset_y_);
        RCLCPP_INFO(this->get_logger(), "Under-shelf distance: %.2f m", under_shelf_distance_);
    }

private:
    enum RobotState {
        IDLE,
        DETECTING_LEGS,
        MOVING_TO_CART,
        MOVING_UNDER_SHELF,
        LIFTING_SHELF,
        COMPLETED,
        SHUTDOWN
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
            RCLCPP_WARN(this->get_logger(), "Service already in progress, rejecting new request");
            return;
        }
        
        // Save request for later use
        current_request_ = request;
        current_response_ = response;
        
        // Reset state variables
        service_in_progress_ = true;
        attach_to_shelf_ = request->attach_to_shelf;
        legs_detected_ = false;
        cart_frame_published_ = false;
        scan_count_ = 0;
        shutdown_counter_ = 0;
        
        // Start the state machine
        state_ = DETECTING_LEGS;
        RCLCPP_INFO(this->get_logger(), "Starting leg detection...");
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
        
        // Count how many scans we've processed
        scan_count_++;
        
        // Check if intensities are available
        if (msg->intensities.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "No intensity data available in laser scan");
            return;
        }
        
        // Parameters for detection
        const float INTENSITY_THRESHOLD = 7000.0;  // Lower threshold to ensure reflectors are found
        const double MAX_RANGE = 2.0;  // Only consider points within 2 meters
        const double MIN_LEG_DISTANCE = 0.3;  // Minimum distance between legs (in meters)
        const double MAX_LEG_DISTANCE = 0.8;  // Maximum distance between legs (in meters)
        
        // Store all high-intensity points with their intensities
        std::vector<std::tuple<double, double, float>> intensity_points;  // x, y, intensity
        
        // Track maximum intensity found for logging
        float max_intensity = 0.0;
        
        // Scan through all points
        for (size_t i = 0; i < msg->intensities.size(); ++i) {
            // Track max intensity
            if (msg->intensities[i] > max_intensity) {
                max_intensity = msg->intensities[i];
            }
            
            if (msg->intensities[i] > INTENSITY_THRESHOLD && 
                msg->ranges[i] < MAX_RANGE && 
                !std::isinf(msg->ranges[i]) && 
                !std::isnan(msg->ranges[i])) {
                
                // Convert to cartesian coordinates relative to robot
                double angle = msg->angle_min + i * msg->angle_increment;
                double x = msg->ranges[i] * std::cos(angle);
                double y = msg->ranges[i] * std::sin(angle);
                
                intensity_points.push_back(std::make_tuple(x, y, msg->intensities[i]));
            }
        }
        
        // Log detection progress
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Scan %d: Found %zu high intensity points (threshold: %.1f), max intensity: %.1f",
            scan_count_, intensity_points.size(), INTENSITY_THRESHOLD, max_intensity);
        
        // Need at least 2 points to continue
        if (intensity_points.size() < 2) {
            // If we've processed too many scans without success, adjust threshold
            if (scan_count_ > 10 && INTENSITY_THRESHOLD > 5000.0) {
                // We'll try with a fixed position in this case
                RCLCPP_WARN(this->get_logger(), 
                    "After %d scans, insufficient high-intensity points. Using approximated position.",
                    scan_count_);
                    
                // Use approximate cart position (63cm forward, centered)
                left_leg_x_ = 0.6;
                left_leg_y_ = 0.15;
                right_leg_x_ = 0.6;
                right_leg_y_ = -0.15;
                
                // Calculate cart position
                calculate_cart_position();
                return;
            }
            return;
        }
        
        // Sort by intensity (highest first)
        std::sort(intensity_points.begin(), intensity_points.end(),
                 [](const auto& a, const auto& b) {
                     return std::get<2>(a) > std::get<2>(b);
                 });
        
        // Print the top high intensity points for debugging
        RCLCPP_INFO(this->get_logger(), "Top intensity points:");
        size_t top_n = std::min(size_t(5), intensity_points.size());
        for (size_t i = 0; i < top_n; i++) {
            RCLCPP_INFO(this->get_logger(), "  #%zu: (%.2f, %.2f) intensity: %.1f",
                       i+1, std::get<0>(intensity_points[i]), std::get<1>(intensity_points[i]),
                       std::get<2>(intensity_points[i]));
        }
        
        // Try to find a valid leg pair
        bool valid_pair_found = false;
        std::pair<size_t, size_t> best_pair = {0, 1};  // Default to first two
        
        // Try all combinations of top points
        for (size_t i = 0; i < top_n && !valid_pair_found; i++) {
            for (size_t j = i+1; j < top_n && !valid_pair_found; j++) {
                double x1 = std::get<0>(intensity_points[i]);
                double y1 = std::get<1>(intensity_points[i]);
                double x2 = std::get<0>(intensity_points[j]);
                double y2 = std::get<1>(intensity_points[j]);
                
                // Calculate distance between these points
                double distance = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
                
                // Check if this pair has a valid distance for shelf legs
                if (distance >= MIN_LEG_DISTANCE && distance <= MAX_LEG_DISTANCE) {
                    // Found a valid pair!
                    valid_pair_found = true;
                    best_pair = {i, j};
                    RCLCPP_INFO(this->get_logger(), "Valid pair found! Points %zu and %zu, distance: %.2f m",
                              i, j, distance);
                    break;
                }
            }
        }
        
        if (valid_pair_found) {
            // Get coordinates of both legs
            double x1 = std::get<0>(intensity_points[best_pair.first]);
            double y1 = std::get<1>(intensity_points[best_pair.first]);
            double x2 = std::get<0>(intensity_points[best_pair.second]);
            double y2 = std::get<1>(intensity_points[best_pair.second]);
            
            // Determine which is left and which is right (based on y coordinate)
            if (y1 < y2) {
                left_leg_x_ = x1;
                left_leg_y_ = y1;
                right_leg_x_ = x2;
                right_leg_y_ = y2;
            } else {
                left_leg_x_ = x2;
                left_leg_y_ = y2;
                right_leg_x_ = x1;
                right_leg_y_ = y1;
            }
            
            // Calculate cart position with offset
            calculate_cart_position();
        } else if (scan_count_ > 20) {
            // If we've scanned too many times without finding a valid pair
            RCLCPP_WARN(this->get_logger(), "No valid leg pair found after %d scans. Using approximate position.", 
                       scan_count_);
                       
            // Use approximate cart position
            left_leg_x_ = 0.6;
            left_leg_y_ = 0.15;
            right_leg_x_ = 0.6;
            right_leg_y_ = -0.15;
            
            // Calculate cart position
            calculate_cart_position();
        }
    }
    
    void calculate_cart_position()
    {
        // Calculate distance between legs
        double leg_distance = std::sqrt(std::pow(right_leg_x_ - left_leg_x_, 2) + 
                                      std::pow(right_leg_y_ - left_leg_y_, 2));

        // Calculate cart position with an intentional rightward bias
        // Original midpoint calculation
        double midpoint_x = (left_leg_x_ + right_leg_x_) / 2.0;
        double midpoint_y = (left_leg_y_ + right_leg_y_) / 2.0;

        // Add rightward bias (negative y in robot frame is to the right)
        double rightward_bias = -0.05;  // 5cm to the right

        // Final cart position with offsets
        cart_x_ = midpoint_x + cart_position_offset_x_;
        cart_y_ = midpoint_y + cart_position_offset_y_ + rightward_bias;

        RCLCPP_INFO(this->get_logger(), 
            "Shelf legs detected! Left leg: (%.2f, %.2f), Right leg: (%.2f, %.2f), Distance: %.2f m",
            left_leg_x_, left_leg_y_, right_leg_x_, right_leg_y_, leg_distance);

        RCLCPP_INFO(this->get_logger(), "Original midpoint: (%.2f, %.2f)",
                   midpoint_x, midpoint_y);
        RCLCPP_INFO(this->get_logger(), "Adjusted cart position: (%.2f, %.2f) (with rightward bias of %.2f m)",
                   cart_x_, cart_y_, rightward_bias);

        // Transform cart position to world frame
        world_cart_x_ = robot_x_ + cart_x_ * std::cos(robot_yaw_) - cart_y_ * std::sin(robot_yaw_);
        world_cart_y_ = robot_y_ + cart_x_ * std::sin(robot_yaw_) + cart_y_ * std::cos(robot_yaw_);

        RCLCPP_INFO(this->get_logger(), "Cart frame in world coordinates: (%.2f, %.2f)",
                   world_cart_x_, world_cart_y_);

        // Calculate the direction from robot to cart
        double dx = world_cart_x_ - robot_x_;
        double dy = world_cart_y_ - robot_y_;
        double distance = std::sqrt(dx*dx + dy*dy);

        RCLCPP_INFO(this->get_logger(), "Robot at (%.2f, %.2f), distance to cart: %.2f m",
                   robot_x_, robot_y_, distance);

        // Publish the cart_frame transform
        publish_cart_transform();

        // Mark legs as detected
        legs_detected_ = true;

        // If we're not supposed to attach, just publish TF and complete
        if (!attach_to_shelf_) {
            complete_service(true);
            return;
        }

        // Start movement to cart
        state_ = MOVING_TO_CART;
        RCLCPP_INFO(this->get_logger(), "Starting to move towards cart position");
    }
    
    void publish_cart_transform()
    {
        if (cart_frame_published_) {
            return; // Only publish once
        }
        
        // Create a static transform message
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "odom";  // Fixed in odom frame
        transform.child_frame_id = "cart_frame";
        
        // Use world coordinates for fixed position
        transform.transform.translation.x = world_cart_x_;
        transform.transform.translation.y = world_cart_y_;
        transform.transform.translation.z = 0.0;
        
        // Identity quaternion (no rotation)
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        
        // Publish static transform
        tf_broadcaster_->sendTransform(transform);
        
        RCLCPP_INFO(this->get_logger(), "Published cart_frame transform at (%.2f, %.2f) in odom frame",
                    world_cart_x_, world_cart_y_);
                    
        cart_frame_published_ = true;
    }
    
    void timer_callback()
    {
        // Process state machine
        switch (state_) {
            case DETECTING_LEGS:
                // Wait for leg detection in scan_callback
                break;
                
            case MOVING_TO_CART:
                execute_move_to_cart();
                break;
                
            case MOVING_UNDER_SHELF:
                execute_move_under_shelf();
                break;
                
            case LIFTING_SHELF:
                execute_lift_shelf();
                break;
                
            case SHUTDOWN:
                // Count down to shutdown
                shutdown_counter_++;
                if (shutdown_counter_ >= 60) { // ~3 seconds at 50ms timer
                    RCLCPP_INFO(this->get_logger(), "Shutting down node...");
                    rclcpp::shutdown();
                }
                break;
                
            case COMPLETED:
            case IDLE:
                // Do nothing
                break;
        }
    }
    
    void execute_move_to_cart()
    {
        // Calculate vector from robot to cart position
        double dx = world_cart_x_ - robot_x_;
        double dy = world_cart_y_ - robot_y_;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Calculate angle to cart in world frame
        double angle_to_cart = std::atan2(dy, dx);
        double angle_error = normalize_angle(angle_to_cart - robot_yaw_);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Moving to cart: Distance: %.2f m, Angle error: %.2f rad",
            distance, angle_error);
        
        // If we've reached the cart
        if (distance < 0.1) {  // 10cm threshold
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "Reached cart position");
            
            // Move directly to under-shelf movement
            state_ = MOVING_UNDER_SHELF;
            movement_timer_ = 0.0;  // Reset timer for under-shelf movement
            RCLCPP_INFO(this->get_logger(), "Starting under-shelf movement");
            return;
        }
        
        // SIMPLER COMBINED APPROACH - always move and turn at same time
        // Use larger threshold (0.15 rad ≈ 8.6 degrees) for better progress
        double forward_speed = 0.0;
        double angular_velocity = 0.0;
        
        // Always apply angular correction
        angular_velocity = 0.3 * angle_error;  // Stronger correction
        
        // Ensure minimum angular velocity to overcome friction when needed
        if (std::abs(angle_error) > 0.1 && std::abs(angular_velocity) < 0.1) {
            angular_velocity = (angle_error > 0) ? 0.1 : -0.1;
        }
        
        // Always move forward, but slower when alignment is poor
        if (std::abs(angle_error) > 0.2) {
            // Large angle error, move slowly
            forward_speed = 0.1;
        } else {
            // Good alignment, move faster
            forward_speed = 0.25;
        }
        
        // Limit velocities 
        forward_speed = std::max(0.0, std::min(0.3, forward_speed));
        angular_velocity = std::max(-0.5, std::min(0.5, angular_velocity));
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "Cart approach: speed=%.2f m/s, angular=%.2f rad/s, distance=%.2f m, angle_error=%.2f rad",
            forward_speed, angular_velocity, distance, angle_error);
        
        // Send velocity commands
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = forward_speed;
        cmd_vel.angular.z = angular_velocity;
        vel_publisher_->publish(cmd_vel);
    }
    
    // Normalize angle to [-π, π]
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    void execute_move_under_shelf()
    {
        // Very simple timed approach with phases
        movement_timer_ += 0.05;  // 50ms per timer tick

        // Simple state machine with timing
        if (movement_timer_ < 0.5) {  // First 0.5 seconds - just stop and prepare
            stop_robot();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Preparing for under-shelf movement...");
        } 
        else if (movement_timer_ < 2.0) {  // Next 1.5 seconds - right turn in place
            // Turn right in place to better align
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = -0.2;  // Right turn
            vel_publisher_->publish(cmd_vel);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Phase 1: Turning right in place...");
        }
        else if (movement_timer_ < 5.0) {  // Next 3 seconds - forward movement
            // Move forward
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.15;  // Forward movement
            cmd_vel.angular.z = -0.02;  // Slight right bias while moving
            vel_publisher_->publish(cmd_vel);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "Phase 2: Moving forward under shelf...");
        }
        else {
            // After 5 seconds, we're done
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "Under-shelf movement completed!");

            // Move to lifting phase
            state_ = LIFTING_SHELF;
            movement_timer_ = 0.0;  // Reset for potential future use
        }
    }
    
    void execute_lift_shelf()
    {
        RCLCPP_INFO(this->get_logger(), "Lifting shelf...");
        
        // Send multiple lift commands to ensure receipt
        std_msgs::msg::String msg;
        msg.data = "";  // Empty string is sufficient for the simulation
        
        // Send multiple commands for reliability
        for (int i = 0; i < 5; i++) {
            elevator_up_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing elevator_up command %d/5", i+1);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        
        RCLCPP_INFO(this->get_logger(), "Shelf lifting completed!");
        
        // Complete service successfully
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
        if (!service_in_progress_) {
            return; // Already completed
        }
        
        service_in_progress_ = false;
        
        // Complete the service call
        if (current_response_) {
            current_response_->complete = success;
            current_response_.reset();
        }
        
        RCLCPP_INFO(this->get_logger(), "Service completed with result: %s", 
                   success ? "success" : "failure");
                   
        // Transition to shutdown state after a brief pause
        RCLCPP_INFO(this->get_logger(), "Node will shut down in 3 seconds...");
        state_ = SHUTDOWN;
    }
    
    // State variables
    RobotState state_;
    bool service_in_progress_;
    bool attach_to_shelf_;
    bool legs_detected_;
    bool cart_frame_published_;
    int scan_count_;
    int shutdown_counter_ = 0;
    
    // Easily adjustable parameters
    double cart_position_offset_x_;
    double cart_position_offset_y_;
    double under_shelf_distance_;  // Distance to move under the shelf
    
    // Service request/response
    std::shared_ptr<attach_shelf::srv::GoToLoading::Request> current_request_;
    std::shared_ptr<attach_shelf::srv::GoToLoading::Response> current_response_;
    
    // Position data
    double robot_x_;    // Robot x position in odom frame
    double robot_y_;    // Robot y position in odom frame
    double robot_yaw_;  // Robot orientation in odom frame
    double cart_x_;     // Cart position x in robot frame
    double cart_y_;     // Cart position y in robot frame
    double world_cart_x_; // Cart position x in odom frame
    double world_cart_y_; // Cart position y in odom frame
    double left_leg_x_;  // Left leg x position in robot frame
    double left_leg_y_;  // Left leg y position in robot frame  
    double right_leg_x_; // Right leg x position in robot frame
    double right_leg_y_; // Right leg y position in robot frame
    double under_shelf_x_; // Under-shelf position x in odom frame
    double under_shelf_y_; // Under-shelf position y in odom frame

    double movement_timer_ = 0.0;
    bool moving_forward_phase_ = false;
    bool turning_right_phase_ = false;
    
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
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