#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cmath>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

class ApproachServiceServer : public rclcpp::Node
{
public:
    ApproachServiceServer() : Node("approach_service_server"),
                             state_(IDLE),
                             service_in_progress_(false),
                             legs_detected_(false),
                             cart_frame_published_(false)
    {
        // Create service
        service_ = this->create_service<attach_shelf::srv::GoToLoading>(
            "/approach_shelf",
            std::bind(&ApproachServiceServer::handle_service, this, std::placeholders::_1, std::placeholders::_2));
        
        // Create velocity publisher
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 10);
        
        // Create elevator control publishers
        elevator_up_publisher_ = this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
        
        // Create subscription to laser scan
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ApproachServiceServer::scan_callback, this, std::placeholders::_1));
        
        // Create transform publisher
        tf_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/tf", 10);
        
        // Create timer for transform publishing and motion control
        timer_ = this->create_wall_timer(
            50ms, std::bind(&ApproachServiceServer::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Approach Service Server started");
    }

private:
    enum ServiceState {
        IDLE,
        DETECTING_LEGS,
        MOVING_TO_CART,
        MOVING_UNDER_SHELF,
        LIFTING_SHELF,
        COMPLETED
    };
    
    void handle_service(
        const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> request,
        std::shared_ptr<attach_shelf::srv::GoToLoading::Response> response)
    {
        if (service_in_progress_) {
            RCLCPP_WARN(this->get_logger(), "Service already in progress, rejecting new request");
            response->complete = false;
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Service called with attach_to_shelf=%s", 
                   request->attach_to_shelf ? "true" : "false");
        
        // Reset state
        legs_detected_ = false;
        cart_frame_published_ = false;
        attach_to_shelf_ = request->attach_to_shelf;
        
        // Start detecting shelf legs
        state_ = DETECTING_LEGS;
        service_in_progress_ = true;
        start_time_ = this->now();
        
        // Block until service completes or times out
        const int timeout_seconds = 30;  // 30 second timeout
        for (int i = 0; i < timeout_seconds && service_in_progress_; i++) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_DEBUG(this->get_logger(), "Waiting for service to complete... %d/%d", i+1, timeout_seconds);
        }
        
        if (service_in_progress_) {
            // If we get here, the service timed out
            RCLCPP_ERROR(this->get_logger(), "Service timed out after %d seconds", timeout_seconds);
            state_ = IDLE;
            service_in_progress_ = false;
            response->complete = false;
        } else {
            // Service completed within timeout
            response->complete = (state_ == COMPLETED);
        }
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (state_ == DETECTING_LEGS) {
            detect_shelf_legs(msg);
        }
    }
    
    void detect_shelf_legs(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Look for high intensity values which indicate shelf legs
        const float intensity_threshold = 7000.0;
        std::vector<size_t> high_intensity_indices;
        
        for (size_t i = 0; i < msg->intensities.size(); i++) {
            if (msg->intensities[i] > intensity_threshold) {
                high_intensity_indices.push_back(i);
            }
        }
        
        // Need at least 2 legs
        if (high_intensity_indices.size() < 2) {
            // Check if we've been searching too long without finding legs
            rclcpp::Duration elapsed = this->now() - start_time_;
            if (elapsed.seconds() > 5.0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to detect both shelf legs after 5 seconds");
                complete_service(false);
            }
            return;
        }
        
        // Get the first two high intensity points (the legs)
        size_t leg1_idx = high_intensity_indices[0];
        size_t leg2_idx = high_intensity_indices[1];
        
        // Get angle and range for each leg
        double angle1 = msg->angle_min + leg1_idx * msg->angle_increment;
        double angle2 = msg->angle_min + leg2_idx * msg->angle_increment;
        double range1 = msg->ranges[leg1_idx];
        double range2 = msg->ranges[leg2_idx];
        
        // Convert to cartesian coordinates
        double x1 = range1 * std::cos(angle1);
        double y1 = range1 * std::sin(angle1);
        double x2 = range2 * std::cos(angle2);
        double y2 = range2 * std::sin(angle2);
        
        // Calculate midpoint between legs (cart position)
        cart_position_x_ = (x1 + x2) / 2.0;
        cart_position_y_ = (y1 + y2) / 2.0;
        
        RCLCPP_INFO(this->get_logger(), "Detected shelf legs at: (%f, %f) and (%f, %f)", x1, y1, x2, y2);
        RCLCPP_INFO(this->get_logger(), "Cart frame position: (%f, %f)", cart_position_x_, cart_position_y_);
        
        // Store the cart position for approach
        legs_detected_ = true;
        cart_frame_published_ = true;
        
        // If we're not supposed to attach to the shelf, complete the service now
        if (!attach_to_shelf_) {
            RCLCPP_INFO(this->get_logger(), "Cart frame published, not proceeding with approach as requested");
            complete_service(true);
            return;
        }
        
        // Otherwise, proceed to move towards the cart
        state_ = MOVING_TO_CART;
        RCLCPP_INFO(this->get_logger(), "Proceeding with final approach to shelf");
    }
    
    void timer_callback()
    {
        // Publish the cart frame transform if legs have been detected
        if (cart_frame_published_) {
            publish_cart_frame();
        }
        
        // State machine for robot movement
        switch (state_) {
            case MOVING_TO_CART:
                move_to_cart();
                break;
                
            case MOVING_UNDER_SHELF:
                move_under_shelf();
                break;
                
            case LIFTING_SHELF:
                lift_shelf();
                break;
                
            default:
                break;
        }
    }
    
    void publish_cart_frame()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "base_link";
        transform.child_frame_id = "cart_frame";
        
        // Set translation
        transform.transform.translation.x = cart_position_x_;
        transform.transform.translation.y = cart_position_y_;
        transform.transform.translation.z = 0.0;
        
        // Set rotation (identity quaternion)
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;
        
        // Publish the transform
        tf_publisher_->publish(transform);
    }
    
    void move_to_cart()
    {
        if (!legs_detected_) {
            return; // Can't move if we haven't detected the legs
        }
        
        // Calculate distance and angle to cart position
        double distance = std::sqrt(cart_position_x_ * cart_position_x_ + cart_position_y_ * cart_position_y_);
        double angle_to_cart = std::atan2(cart_position_y_, cart_position_x_);
        
        geometry_msgs::msg::Twist cmd_vel;
        
        // First align with the cart
        if (std::abs(angle_to_cart) > 0.05) { // ~3 degrees
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = angle_to_cart > 0 ? 0.3 : -0.3;
            vel_publisher_->publish(cmd_vel);
            return;
        }
        
        // Then move towards the cart
        if (distance > 0.1) { // Keep 10cm distance from cart center
            cmd_vel.linear.x = std::min(0.2, distance * 0.5); // Slow down as we approach
            cmd_vel.angular.z = 0.0;
            vel_publisher_->publish(cmd_vel);
            return;
        }
        
        // Stop when we reach the cart position
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        vel_publisher_->publish(cmd_vel);
        
        // Move to next state
        RCLCPP_INFO(this->get_logger(), "Reached cart position, moving under shelf");
        state_ = MOVING_UNDER_SHELF;
        approach_start_time_ = this->now();
    }
    
    void move_under_shelf()
    {
        // We need to move forward 30cm more to get under the shelf
        rclcpp::Duration elapsed = this->now() - approach_start_time_;
        double move_duration = 3.0; // 3 seconds to move 30cm at 0.1 m/s
        
        geometry_msgs::msg::Twist cmd_vel;
        
        if (elapsed.seconds() < move_duration) {
            cmd_vel.linear.x = 0.1; // Move slowly forward
            cmd_vel.angular.z = 0.0;
            vel_publisher_->publish(cmd_vel);
        } else {
            // Stop when we've moved far enough
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            vel_publisher_->publish(cmd_vel);
            
            // Move to next state
            RCLCPP_INFO(this->get_logger(), "Positioned under shelf, preparing to lift");
            state_ = LIFTING_SHELF;
        }
    }
    
    void lift_shelf()
    {
        // Publish to the elevator_up topic
        std_msgs::msg::String msg;
        elevator_up_publisher_->publish(msg);
        
        RCLCPP_INFO(this->get_logger(), "Shelf lifted successfully");
        
        // Complete the service
        complete_service(true);
    }
    
    void complete_service(bool success)
    {
        if (success) {
            state_ = COMPLETED;
        } else {
            state_ = IDLE;
        }
        
        service_in_progress_ = false;
        RCLCPP_INFO(this->get_logger(), "Service completed with result: %s", success ? "success" : "failure");
    }
    
    // Service state and parameters
    ServiceState state_;
    bool service_in_progress_;
    bool legs_detected_;
    bool cart_frame_published_;
    bool attach_to_shelf_;
    rclcpp::Time start_time_;
    rclcpp::Time approach_start_time_;
    
    // Cart position
    double cart_position_x_;
    double cart_position_y_;
    
    // ROS interfaces
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
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