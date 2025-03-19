#pragma once
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <mutex>
#include "algorithms/kinematics.hpp"

class EncoderNode {
public:
    // Constructor takes a shared_ptr to an existing node instead of creating one.
    EncoderNode(const rclcpp::Node::SharedPtr &node)
        : node_(node), start_time_(node_->now()), 
          last_encoder_left_(0), last_encoder_right_(0),
          robot_x_(0.0), robot_y_(0.0), robot_theta_(0.0),
          first_reading_(true) {
            
        // Initialize kinematics with robot parameters
        constexpr float WHEEL_RADIUS = 0.0325;  // meters
        constexpr float WHEEL_BASE = 0.128;     // meters
        constexpr int PULSES_PER_ROTATION = 584;
        kinematics_ = std::make_unique<algorithms::Kinematics>(
            WHEEL_RADIUS, WHEEL_BASE, PULSES_PER_ROTATION);
            
        // Initialize the subscriber for encoder data
        const std::string subscribe_topic = "/bpc_prp_robot/encoders";
        subscriber_ = node_->create_subscription<std_msgs::msg::UInt32MultiArray>(
            subscribe_topic, 10, 
            std::bind(&EncoderNode::subscriber_callback, this, std::placeholders::_1));
            
        // Initialize publisher for robot pose
        pose_publisher_ = node_->create_publisher<geometry_msgs::msg::Pose2D>(
            "/robot_pose", 10);
            
        RCLCPP_INFO(node_->get_logger(), "Encoder node initialized");
    }
    
    // Get the current robot position
    algorithms::Coordinates get_current_position() {
        std::lock_guard<std::mutex> lock(position_mutex_);
        return algorithms::Coordinates(robot_x_, robot_y_, robot_theta_);
    }
    
private:
    void subscriber_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(node_->get_logger(), "Received incomplete encoder data");
            return;
        }
        
        uint32_t encoder_left = msg->data[0];
        uint32_t encoder_right = msg->data[1];
        
        //RCLCPP_INFO(node_->get_logger(), "Received encoders: L=%u R=%u", encoder_left, encoder_right);
        
        // Skip the first reading to establish a baseline
        if (first_reading_) {
            last_encoder_left_ = encoder_left;
            last_encoder_right_ = encoder_right;
            first_reading_ = false;
            return;
        }
        
        // Calculate encoder differences (handle potential rollover)
        int32_t diff_left = static_cast<int32_t>(encoder_left - last_encoder_left_);
        int32_t diff_right = static_cast<int32_t>(encoder_right - last_encoder_right_);
        
        // Use the kinematics class to convert encoder differences to displacement
        algorithms::Encoders encoder_diffs(-diff_left, diff_right);
        algorithms::Coordinates displacement = kinematics_->forward(encoder_diffs);
        
        // Update robot position (apply the displacement in the robot's frame to the global position)
        update_position(displacement);
        
        // Publish the updated position
        publish_position();
        
        // Update last encoder values for next calculation
        last_encoder_left_ = encoder_left;
        last_encoder_right_ = encoder_right;
    }
    
    void update_position(const algorithms::Coordinates& displacement) {
        std::lock_guard<std::mutex> lock(position_mutex_);
        
        // Apply displacement in the robot's current reference frame
        // For a differential drive robot, we need to transform the local displacement
        // to the global reference frame based on the current orientation
        
        // Calculate displacement in global frame
        double dx_global = displacement.x * std::cos(robot_theta_) - displacement.y * std::sin(robot_theta_);
        double dy_global = displacement.x * std::sin(robot_theta_) + displacement.y * std::cos(robot_theta_);
        
        // Update the robot's position and orientation
        robot_x_ += dx_global;
        robot_y_ += dy_global;
        robot_theta_ += displacement.theta;
        
        // Normalize theta to -π to π
        while (robot_theta_ > M_PI) robot_theta_ -= 2.0 * M_PI;
        while (robot_theta_ < -M_PI) robot_theta_ += 2.0 * M_PI;
        
        /*RCLCPP_INFO(node_->get_logger(), "Updated position: x=%.3f, y=%.3f, theta=%.3f", 
                    robot_x_, robot_y_, robot_theta_);*/
    }
    
    void publish_position() {
        geometry_msgs::msg::Pose2D pose_msg;
        
        {
            std::lock_guard<std::mutex> lock(position_mutex_);
            pose_msg.x = robot_x_;
            pose_msg.y = robot_y_;
            pose_msg.theta = robot_theta_;
        }
        
        pose_publisher_->publish(pose_msg);
    }
    
    // Shared pointer to the main ROS node
    rclcpp::Node::SharedPtr node_;
    
    // Publisher, subscriber
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
    
    // Start time for uptime calculation
    rclcpp::Time start_time_;
    
    // Kinematics calculator
    std::unique_ptr<algorithms::Kinematics> kinematics_;
    
    // Track last encoder values to calculate differences
    uint32_t last_encoder_left_;
    uint32_t last_encoder_right_;
    
    // Current robot position and orientation
    double robot_x_;
    double robot_y_;
    double robot_theta_;
    
    // Flag for first reading
    bool first_reading_;
    
    // Mutex for thread-safe access to position
    std::mutex position_mutex_;
};