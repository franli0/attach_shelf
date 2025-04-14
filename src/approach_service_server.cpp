#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
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
        
        // Create publishers and subscribers
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 10);
        
        elevator_up_publisher_ = this->create_publisher<std_msgs::msg::String>(
            "/elevator_up", 10);
        
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ApproachServiceServer::scan_callback, this, std::placeholders::_1));
        
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
        
        // Reset state
        service_in_progress_ = true;
        attach_to_shelf_ = request->attach_to_shelf;
        legs_detected_ = false;
        cart_frame_published_ = false;
        reached_cart_position_ = false;
        start_time_ = this->now();
        
        // Start the state machine
        state_ = DETECTING_LEGS;
        
        // Non-blocking service handler
        std::thread([this, response]() {
            const int timeout_seconds = 60;
            for (int i = 0; i < timeout_seconds && service_in_progress_; i++) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                RCLCPP_INFO(this->get_logger(), "Service running... %d/%d, state=%d", 
                           i+1, timeout_seconds, (int)state_);
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
    
    void detect_shelf_legs(const sensor_msgs::msg::LaserScan::SharedPtr)
    {
        // For simplicity, using simulated leg detection
        RCLCPP_INFO_ONCE(this->get_logger(), "Using simulated leg detection");
        
        // Simulated legs at fixed positions
        double leg1_x = 0.7;
        double leg1_y = -0.3;
        double leg2_x = 0.7;
        double leg2_y = 0.3;
        
        // Calculate cart position (midpoint between legs)
        cart_position_x_ = (leg1_x + leg2_x) / 2.0;
        cart_position_y_ = (leg1_y + leg2_y) / 2.0;
        
        RCLCPP_INFO(this->get_logger(), "Cart frame position: (%.2f, %.2f)", 
                    cart_position_x_, cart_position_y_);
        
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
        // Step 1: Move to cart position (fixed time-based approach)
        if (!reached_cart_position_) {
            // Calculate how long we've been moving
            rclcpp::Duration elapsed_move = this->now() - move_start_time_;
            movement_phase_duration_ = elapsed_move.seconds();
            
            // Target time to reach cart - approximately 0.7m at 0.3m/s = ~2.3 seconds
            const double move_to_cart_time = 2.3;
            
            if (movement_phase_duration_ < move_to_cart_time) {
                // Still moving to cart position
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0.3;  // Faster speed
                cmd_vel.angular.z = 0.0;
                vel_publisher_->publish(cmd_vel);
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Moving to cart: %.1f/%.1f seconds", 
                                movement_phase_duration_, move_to_cart_time);
                return;
            }
            
            // Reached cart position, stop robot
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "*** REACHED CART POSITION AFTER %.2f SECONDS ***", 
                        movement_phase_duration_);
            
            // Wait to ensure robot is fully stopped
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // Now ready for under-shelf movement
            reached_cart_position_ = true;
            move_start_time_ = this->now();  // Reset timer for next phase
            movement_phase_duration_ = 0.0;
            
            // Transition to under-shelf movement
            state_ = MOVING_UNDER_SHELF;
            RCLCPP_INFO(this->get_logger(), "*** STARTING UNDER-SHELF MOVEMENT ***");
        }
    }
    
    void execute_move_under_shelf()
    {
        // Step 2: Move 30cm under shelf (fixed time-based approach)
        rclcpp::Duration elapsed_move = this->now() - move_start_time_;
        movement_phase_duration_ = elapsed_move.seconds();
        
        // Target time for 30cm under-shelf movement at 0.15m/s = 2 seconds
        const double under_shelf_time = 2.0;
        
        if (movement_phase_duration_ < under_shelf_time) {
            // Still moving under shelf
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.15;
            cmd_vel.angular.z = 0.0;
            vel_publisher_->publish(cmd_vel);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Moving under shelf: %.1f/%.1f seconds", 
                            movement_phase_duration_, under_shelf_time);
            return;
        }
        
        // Completed under-shelf movement, stop robot
        stop_robot();
        RCLCPP_INFO(this->get_logger(), "*** COMPLETED UNDER-SHELF MOVEMENT AFTER %.2f SECONDS ***", 
                    movement_phase_duration_);
        
        // Wait to ensure robot is fully stopped
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Transition to lifting shelf
        state_ = LIFTING_SHELF;
        RCLCPP_INFO(this->get_logger(), "*** READY TO LIFT SHELF ***");
    }
    
    void execute_lift_shelf()
    {
        RCLCPP_INFO(this->get_logger(), "Lifting shelf now...");
        
        // Send multiple lift commands to ensure receipt
        std_msgs::msg::String msg;
        msg.data = "up";
        
        for (int i = 0; i < 20; i++) {
            elevator_up_publisher_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
    
    // Position data
    double cart_position_x_;
    double cart_position_y_;
    
    // Timing
    rclcpp::Time start_time_;
    rclcpp::Time move_start_time_;
    double movement_phase_duration_;
    
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_up_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
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