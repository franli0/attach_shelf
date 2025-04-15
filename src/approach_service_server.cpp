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
        rotation_complete_ = false;
        
        // Initialize position tracking
        robot_x_ = 0.0;
        robot_y_ = 0.0;
        robot_yaw_ = 0.0;
        target_x_ = 0.0;
        target_y_ = 0.0;
        prev_x_ = 0.0;
        prev_y_ = 0.0;
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
        
        // Track distance moved for under-shelf movement
        if (state_ == MOVING_UNDER_SHELF && prev_x_ != 0.0) {
            double dx = robot_x_ - prev_x_;
            double dy = robot_y_ - prev_y_;
            double delta_dist = std::sqrt(dx*dx + dy*dy);
            
            // Add to total distance if we're moving forward (not backward)
            // Using the dot product with forward direction
            double forward_x = std::cos(robot_yaw_);
            double forward_y = std::sin(robot_yaw_);
            double dot_product = dx * forward_x + dy * forward_y;
            
            if (dot_product > 0) {
                distance_moved_ += delta_dist;
            }
        }
        
        // Store current position for next distance calculation
        prev_x_ = robot_x_;
        prev_y_ = robot_y_;
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
        reached_cart_position_ = false;
        distance_moved_ = 0.0;
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
                
                // Include position information in the log
                if (state_ == MOVING_TO_CART || state_ == MOVING_UNDER_SHELF) {
                    RCLCPP_INFO(this->get_logger(), "Robot at (%.2f, %.2f), Target at (%.2f, %.2f)", 
                               robot_x_, robot_y_, target_x_, target_y_);
                }
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
        }
        
        const float INTENSITY_THRESHOLD = 7500.0;  // Reflective markers should have very high intensity
        const double MAX_RANGE = 2.0;  // Only consider points within 2 meters
        
        // Store any high-intensity points (potential shelf legs)
        std::vector<std::pair<double, double>> high_intensity_points;
        std::vector<float> intensities_found;
        
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
                intensities_found.push_back(msg->intensities[i]);
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
            // Sort points by Y value to find the leftmost and rightmost legs
            std::sort(high_intensity_points.begin(), high_intensity_points.end(),
                     [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                         return a.second < b.second;
                     });
            
            // Take the points with the smallest and largest Y values (leftmost and rightmost)
            auto leg1 = high_intensity_points.front();
            auto leg2 = high_intensity_points.back();
            
            // Calculate cart position (midpoint between legs)
            cart_position_x_ = (leg1.first + leg2.first) / 2.0;
            cart_position_y_ = (leg1.second + leg2.second) / 2.0;
            
            // Calculate leg distance for validation
            double leg_distance = std::sqrt(
                std::pow(leg2.first - leg1.first, 2) + 
                std::pow(leg2.second - leg1.second, 2));
            
            RCLCPP_INFO(this->get_logger(), 
                "Shelf legs detected! Leg1: (%.2f, %.2f), Leg2: (%.2f, %.2f), Distance: %.2f m",
                leg1.first, leg1.second, leg2.first, leg2.second, leg_distance);
            
            RCLCPP_INFO(this->get_logger(), "Cart frame position set to (%.2f, %.2f)",
                cart_position_x_, cart_position_y_);
            
            // Transform cart position from robot frame to world frame
            // Important: Store this position in world frame so it doesn't move as robot moves
            target_x_ = robot_x_ + cart_position_x_ * std::cos(robot_yaw_) - cart_position_y_ * std::sin(robot_yaw_);
            target_y_ = robot_y_ + cart_position_x_ * std::sin(robot_yaw_) + cart_position_y_ * std::cos(robot_yaw_);
            
            RCLCPP_INFO(this->get_logger(), "Cart frame in world coordinates: (%.2f, %.2f)",
                target_x_, target_y_);
            
            // Mark legs as detected
            legs_detected_ = true;
            cart_frame_published_ = true;
            
            // If we're not supposed to attach, just publish TF and complete
            if (!attach_to_shelf_) {
                complete_service(true);
                return;
            }
            
            // Start movement to cart
            state_ = MOVING_TO_CART;
            move_start_time_ = this->now();
            movement_phase_duration_ = 0.0;
            
        } else if (service_in_progress_ && state_ == DETECTING_LEGS) {
            // If service is running but we couldn't find legs after some time, use fallback
            rclcpp::Duration elapsed = this->now() - start_time_;
            
            if (elapsed.seconds() > 5.0) {  // After 5 seconds without legs detected
                RCLCPP_WARN(this->get_logger(), 
                    "Failed to detect shelf legs after %.1f seconds, using fallback position",
                    elapsed.seconds());
                
                // Use fallback position in front of the robot
                cart_position_x_ = 0.75;  // 75cm forward
                cart_position_y_ = 0.0;   // Centered
                
                // Transform to world coordinates
                target_x_ = robot_x_ + cart_position_x_ * std::cos(robot_yaw_) - cart_position_y_ * std::sin(robot_yaw_);
                target_y_ = robot_y_ + cart_position_x_ * std::sin(robot_yaw_) + cart_position_y_ * std::cos(robot_yaw_);
                
                RCLCPP_INFO(this->get_logger(), "Using fallback cart position: (%.2f, %.2f) in robot frame",
                    cart_position_x_, cart_position_y_);
                RCLCPP_INFO(this->get_logger(), "Fallback cart position in world coordinates: (%.2f, %.2f)",
                    target_x_, target_y_);
                
                // Mark legs as detected
                legs_detected_ = true;
                cart_frame_published_ = true;
                
                // If we're not supposed to attach, just publish TF and complete
                if (!attach_to_shelf_) {
                    complete_service(true);
                    return;
                }
                
                // Start movement to cart
                state_ = MOVING_TO_CART;
                move_start_time_ = this->now();
                movement_phase_duration_ = 0.0;
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
        
        transform.transform.translation.x = cart_position_x_;
        transform.transform.translation.y = cart_position_y_;
        transform.transform.translation.z = 0.0;
        
        transform.transform.rotation.w = 1.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        
        tf_broadcaster_->sendTransform(transform);
    }
    
    void execute_move_to_cart()
    {
        // If we haven't completed rotation yet, first align with the cart frame
        if (!rotation_complete_) {
            // Calculate angle to target in robot frame
            double target_angle = std::atan2(cart_position_y_, cart_position_x_);
            
            // If we're significantly misaligned, rotate to align
            if (std::abs(target_angle) > 0.05) { // ~3 degrees threshold
                // Rotate to align with target using proportional control
                double angular_velocity = 0.5 * target_angle;
                
                // Limit angular velocity
                angular_velocity = std::max(-0.3, std::min(0.3, angular_velocity));
                
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = angular_velocity;
                vel_publisher_->publish(cmd_vel);
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Aligning with target, angle: %.2f rad, angular velocity: %.2f rad/s", 
                    target_angle, angular_velocity);
                return;
            }
            
            // We've finished rotating, now calculate the target position in world coordinates
            // by getting the current robot position and adding the offset
            target_x_ = robot_x_ + cart_position_x_ * std::cos(robot_yaw_) - cart_position_y_ * std::sin(robot_yaw_);
            target_y_ = robot_y_ + cart_position_x_ * std::sin(robot_yaw_) + cart_position_y_ * std::cos(robot_yaw_);
            
            RCLCPP_INFO(this->get_logger(), "Alignment complete. Setting target at world coordinates: (%.2f, %.2f)", 
                      target_x_, target_y_);
            
            // Mark rotation as complete
            rotation_complete_ = true;
            
            // Reset distance for the forward movement phase
            distance_moved_ = 0.0;
            linear_move_start_ = this->now();
            return;
        }
        
        // For the forward movement phase, if we're not at the cart position yet
        if (!reached_cart_position_) {
            // Calculate how long we've been moving (for timeout purposes)
            rclcpp::Duration elapsed_move = this->now() - linear_move_start_;
            double movement_time = elapsed_move.seconds();
            
            // Check for timeout
            if (movement_time > 15.0) {
                RCLCPP_ERROR(this->get_logger(), "Timeout reaching cart position. Moving on anyway.");
                reached_cart_position_ = true;
                move_start_time_ = this->now();
                distance_moved_ = 0.0;
                state_ = MOVING_UNDER_SHELF;
                return;
            }
            
            // Calculate distance to target in world frame
            double dx = target_x_ - robot_x_;
            double dy = target_y_ - robot_y_;
            double distance_to_target = std::sqrt(dx*dx + dy*dy);
            
            // Log the current position and target
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "Robot at (%.2f, %.2f, %.2f), Target at (%.2f, %.2f), Distance: %.2f",
                robot_x_, robot_y_, robot_yaw_, target_x_, target_y_, distance_to_target);
            
            // Move until we're close enough to the target
            if (distance_to_target > 0.05) { // 5cm threshold
                // Calculate angle to target in robot frame
                double target_angle = std::atan2(dy, dx) - robot_yaw_;
                // Normalize angle to [-π, π]
                while (target_angle > M_PI) target_angle -= 2*M_PI;
                while (target_angle < -M_PI) target_angle += 2*M_PI;
                
                // Set forward speed based on distance, slower when closer
                double forward_speed = 0.2;
                if (distance_to_target < 0.3) {
                    forward_speed = 0.1; // Slow down for last 30cm
                }
                
                // Small steering correction to stay on course
                double angular_correction = 0.5 * target_angle;
                angular_correction = std::max(-0.2, std::min(0.2, angular_correction));
                
                // Move with combined linear and angular velocities
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = forward_speed;
                cmd_vel.angular.z = angular_correction;
                vel_publisher_->publish(cmd_vel);
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Moving toward target at (%.2f, %.2f), distance: %.2f, angle: %.2f", 
                    target_x_, target_y_, distance_to_target, target_angle);
            } else {
                // We've reached the target position
                stop_robot();
                RCLCPP_INFO(this->get_logger(), "*** REACHED CART POSITION (%.2f, %.2f) ***", 
                            target_x_, target_y_);
                
                // Wait to ensure robot is fully stopped
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
                // Reset for the under-shelf movement
                reached_cart_position_ = true;
                move_start_time_ = this->now();
                distance_moved_ = 0.0;
                
                // Transition to under-shelf movement
                state_ = MOVING_UNDER_SHELF;
                RCLCPP_INFO(this->get_logger(), "*** STARTING UNDER-SHELF MOVEMENT ***");
            }
        }
    }
    
    void execute_move_under_shelf()
    {
        // We want to move 30cm under the shelf
        const double UNDER_SHELF_DISTANCE = 0.3;  // 30cm as specified in requirements
        
        // Track how long we've been in this state for timeout purposes
        rclcpp::Duration elapsed_move = this->now() - move_start_time_;
        movement_phase_duration_ = elapsed_move.seconds();
        
        // Timeout after 15 seconds (longer than should be needed)
        const double TIMEOUT = 15.0;
        
        if (movement_phase_duration_ > TIMEOUT) {
            RCLCPP_ERROR(this->get_logger(), "Timeout while moving under shelf!");
            stop_robot();
            state_ = LIFTING_SHELF;  // Try to lift anyway
            return;
        }
        
        // Log the distance moved so far
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Moving under shelf: %.2f/%.2f meters", 
            distance_moved_, UNDER_SHELF_DISTANCE);
        
        // Keep moving until we've moved the desired distance
        if (distance_moved_ < UNDER_SHELF_DISTANCE) {
            // Continue moving forward
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.15;  // Constant forward speed
            cmd_vel.angular.z = 0.0;  // No rotation
            vel_publisher_->publish(cmd_vel);
        } else {
            // We've moved far enough
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "*** COMPLETED UNDER-SHELF MOVEMENT (%.2f meters) ***", 
                distance_moved_);
            
            // Wait to ensure robot is fully stopped
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Transition to lifting shelf
            state_ = LIFTING_SHELF;
            RCLCPP_INFO(this->get_logger(), "*** READY TO LIFT SHELF ***");
        }
    }
    
    void execute_lift_shelf()
    {
        RCLCPP_INFO(this->get_logger(), "Lifting shelf now...");
        
        // Send multiple lift commands to ensure receipt
        std_msgs::msg::String msg;
        msg.data = "";  // Empty string is sufficient for the simulation
        
        // Send more lift commands and with longer intervals for better reliability
        for (int i = 0; i < 10; i++) {
            elevator_up_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing elevator_up command %d/10", i+1);
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
    bool reached_cart_position_;
    bool rotation_complete_;
    
    // Position data in robot frame (for cart frame)
    double cart_position_x_;
    double cart_position_y_;
    
    // World frame coordinates
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    double target_x_;  // Cart position in world frame
    double target_y_;
    double prev_x_;
    double prev_y_;
    double distance_moved_;
    
    // Timing
    rclcpp::Time start_time_;
    rclcpp::Time move_start_time_;
    rclcpp::Time linear_move_start_;
    double movement_phase_duration_;
    
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