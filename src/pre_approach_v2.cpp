#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include <cmath>

class PreApproachV2Node : public rclcpp::Node
{
public:
    PreApproachV2Node() : Node("pre_approach_v2_node"), 
                       state_(MOVING_FORWARD), 
                       rotation_started_(false),
                       service_called_(false),
                       current_yaw_(0.0),
                       start_yaw_(0.0),
                       target_rotation_(0.0),
                       target_yaw_(0.0)
    {
        // Declare and get parameters
        this->declare_parameter<double>("obstacle", 0.5);
        this->declare_parameter<int>("degrees", -90);
        this->declare_parameter<bool>("final_approach", true);
        
        obstacle_distance_ = this->get_parameter("obstacle").as_double();
        rotation_degrees_ = static_cast<double>(this->get_parameter("degrees").as_int());
        final_approach_ = this->get_parameter("final_approach").as_bool();
        
        // Convert degrees to radians
        rotation_radians_ = rotation_degrees_ * M_PI / 180.0;
        
        RCLCPP_INFO(this->get_logger(), "Pre-Approach V2 Node started. Will stop at %f meters and rotate %f degrees. Final approach: %s",
                    obstacle_distance_, rotation_degrees_, final_approach_ ? "true" : "false");
        
        // Create publisher for robot velocity
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/diffbot_base_controller/cmd_vel_unstamped", 10);
        
        // Create subscription to laser scan
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PreApproachV2Node::scan_callback, this, std::placeholders::_1));
        
        // Subscribe to odometry for better rotation control
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PreApproachV2Node::odom_callback, this, std::placeholders::_1));
        
        // Create service client
        client_ = this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
        
        // Timer for publishing velocity commands
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // Faster update rate for smoother control
            std::bind(&PreApproachV2Node::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Publishing velocity commands to /diffbot_base_controller/cmd_vel_unstamped");
    }

private:
    enum RobotState {
        MOVING_FORWARD,
        ROTATING,
        CALLING_SERVICE,
        STOPPED
    };
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (state_ == MOVING_FORWARD)
        {
            // Get the range directly in front of the robot
            size_t front_index = msg->ranges.size() / 2;
            double front_distance = msg->ranges[front_index];
            
            // Check if we're close enough to the obstacle
            if (!std::isinf(front_distance) && front_distance < obstacle_distance_)
            {
                RCLCPP_INFO(this->get_logger(), "Obstacle detected at %f meters. Stopping and preparing to rotate.", 
                           front_distance);
                state_ = ROTATING;
                rotation_started_ = false;
            }
        }
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract the yaw from the quaternion directly
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double w = msg->pose.pose.orientation.w;
        
        // Convert quaternion to yaw (simpler version without tf2)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }
    
    double normalize_angle(double angle)
    {
        // Normalize angle to be between -π and π
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    double calculate_angular_difference()
    {
        // Calculate the angular difference (how much we've rotated so far)
        double difference = normalize_angle(current_yaw_ - start_yaw_);
        return difference;
    }
    
    void timer_callback()
    {
        geometry_msgs::msg::Twist cmd_vel;
        
        switch (state_)
        {
            case MOVING_FORWARD:
            {
                // Move forward with perfect straight line (zero angular velocity)
                cmd_vel.linear.x = 0.2;
                cmd_vel.angular.z = 0.0;
                break;
            }
            case ROTATING:
            {
                if (!rotation_started_)
                {
                    // Stop first
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    vel_publisher_->publish(cmd_vel);
                    
                    // Record starting yaw angle
                    start_yaw_ = current_yaw_;
                    
                    // Calculate target rotation amount (add small buffer to ensure full rotation)
                    target_rotation_ = rotation_radians_ * 1.02; // Add 2% buffer
                    
                    RCLCPP_INFO(this->get_logger(), "Starting rotation from %f radians, target rotation: %f radians", 
                                start_yaw_, target_rotation_);
                    
                    rotation_started_ = true;
                    return;
                }
                
                // Calculate how much we've rotated so far
                double rotated_amount = calculate_angular_difference();
                
                // Determine how much more we need to rotate (sign matters for direction)
                double remaining_rotation = target_rotation_ - rotated_amount;
                
                // Check if we've rotated enough
                if (std::abs(remaining_rotation) > 0.02) // ~1 degree threshold
                {
                    // Continue rotating with proportional control for precision
                    double k_p = 0.8; // Proportional gain
                    double angular_velocity = std::max(-0.5, std::min(0.5, k_p * remaining_rotation));
                    
                    // Ensure minimum velocity to overcome static friction
                    if (std::abs(angular_velocity) < 0.1 && std::abs(remaining_rotation) > 0.02) {
                        angular_velocity = (remaining_rotation > 0) ? 0.1 : -0.1;
                    }
                    
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = angular_velocity;
                }
                else
                {
                    // Stop rotating
                    cmd_vel.linear.x = 0.0;
                    cmd_vel.angular.z = 0.0;
                    
                    RCLCPP_INFO(this->get_logger(), "Rotation completed. Target: %f, Actual: %f, Error: %f degrees",
                                rotation_degrees_, rotated_amount * 180.0 / M_PI, 
                                (target_rotation_ - rotated_amount) * 180.0 / M_PI);
                    
                    // Move to service calling state
                    state_ = CALLING_SERVICE;
                }
                break;
            }
            case CALLING_SERVICE:
            {
                // Stop the robot
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                
                // Call the approach service
                if (!service_called_) {
                    call_approach_service();
                    service_called_ = true;
                }
                break;
            }
            case STOPPED:
            {
                // Do nothing, stay stopped
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;
                break;
            }
        }
        
        vel_publisher_->publish(cmd_vel);
    }
    
    void call_approach_service()
    {
        // Wait for service to be available with a timeout
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service /approach_shelf not available. Retrying...");
            service_called_ = false;  // Reset to try again next iteration
            return;
        }
        
        // Create request
        auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
        request->attach_to_shelf = final_approach_;
        
        RCLCPP_INFO(this->get_logger(), "Calling approach service with attach_to_shelf=%s", 
                   final_approach_ ? "true" : "false");
        
        // Send async request
        auto result_future = client_->async_send_request(
            request,
            [this](rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
                auto result = future.get();
                RCLCPP_INFO(this->get_logger(), "Service call completed with result: %s", 
                           result->complete ? "success" : "failure");
                
                state_ = STOPPED;
            }
        );
    }
    
    // Node state variables
    RobotState state_;
    double obstacle_distance_;
    double rotation_degrees_;
    double rotation_radians_;
    bool rotation_started_;
    bool final_approach_;
    bool service_called_;
    
    // Orientation tracking
    double current_yaw_;
    double start_yaw_;
    double target_rotation_;
    double target_yaw_;
    
    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PreApproachV2Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}