#pragma once

#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "algorithms/kinematics.hpp"  // Include the kinematics logic

class MotorNode
{
public:
    MotorNode(const rclcpp::Node::SharedPtr &node)
        : node_(node),
          start_time_(node_->now()),
          // Initialize kinematics with wheel radius (m), wheel base (m), and ticks per rotation
          kinematics_(0.033, 0.16, 360),
          // Initialize movement state to stopped
          movement_enabled_(false)
    {
        const std::string motors_topic = "/bpc_prp_robot/set_motor_speeds";
        const std::string lidar_topic = "/bpc_prp_robot/lidar_avg";
        const std::string buttons_topic = "/bpc_prp_robot/buttons";

        // Initialize the motor speeds publisher
        motors_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(motors_topic, 1);

        // Subscribe to the lidar averaged data
        lidar_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            lidar_topic, 1, std::bind(&MotorNode::lidar_callback, this, std::placeholders::_1));

        // Subscribe to the buttons data
        buttons_subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
            buttons_topic, 1, std::bind(&MotorNode::buttons_callback, this, std::placeholders::_1));

        // Create a timer for publishing motor speeds (50 Hz)
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MotorNode::timer_callback, this));

        RCLCPP_INFO(node_->get_logger(), "Simplified motor control node started with button control!");
    }

private:
    // Map wheel speed to motor command between 127 and 255
    uint8_t map_speed_to_motor_cmd(float wheel_speed)
    {
        // Normalize the wheel speed to a motor command in [127, 255]
        // TODO: the fuck does this do?
        float normalized_value = (wheel_speed + max_wheel_speed) / (2 * max_wheel_speed);
        int mapped_value = 127 + static_cast<int>((255 - 127) * normalized_value);
        // Clamp the value between 127 and 255
        return static_cast<uint8_t>(std::min(255, std::max(127, mapped_value)));
    }

    // Callback to update wheel speeds based on the lidar measurements
    void lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Ensure we have the expected data format (front, right, left)
        if (msg->data.size() >= 3)
        {
            float front_dist = msg->data[0];
            float right_dist = msg->data[1];
            float left_dist = msg->data[2];

            // Emergency stop if too close to front obstacle
            if (front_dist < 0.2) {
                movement_enabled_ = false;
                RCLCPP_WARN(node_->get_logger(), "Emergency stop: obstacle too close (%.2f m)", front_dist);
                left_speed_ = 127;
                right_speed_ = 127;
                return;
            }

            // Calculate corridor offset (positive if closer to left wall, negative if closer to right wall)
            float corridor_offset = right_dist - left_dist;
            
            // Directly use the offset to calculate angular velocity with a gain factor
            float angular_velocity = steering_gain * corridor_offset;
            
            // Limit the maximum angular velocity
            angular_velocity = std::min(std::max(angular_velocity, -max_angular_velocity), max_angular_velocity);

            // Set linear velocity based on movement state
            float linear_velocity = movement_enabled_ ? 0.001f : 0.0f;

            // Create robot speed structure
            algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);
            
            // Use inverse kinematics to get wheel speeds
            algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);

            RCLCPP_INFO(node_->get_logger(), "raw left: %f, raw right: %f", wheel_speeds.l, wheel_speeds.r);
            
            // Map wheel speeds to motor commands and store them
            left_speed_ = map_speed_to_motor_cmd(wheel_speeds.l);
            right_speed_ = map_speed_to_motor_cmd(wheel_speeds.r);

            RCLCPP_INFO(node_->get_logger(), "mapped left: %d, mapped right: %d", left_speed_, right_speed_);

            // If movement is disabled, set speeds to the stop value (127)
            if (!movement_enabled_)
            {
                left_speed_ = 127;
                right_speed_ = 127;
            }

            // RCLCPP_INFO(node_->get_logger(), 
            //             "Corridor: L=%.2f, R=%.2f, Offset=%.2f, Angular=%.2f, Enabled=%s", 
            //             left_dist, right_dist, corridor_offset, angular_velocity,
            //             movement_enabled_ ? "true" : "false");
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Unexpected lidar data format!");
        }
    }
    
    // Callback to handle button presses
    void buttons_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        RCLCPP_INFO(node_->get_logger(), "Button pressed %d", msg->data);

        // Toggle movement state when button 0 is pressed
        if (msg->data == 0) {
            movement_enabled_ = !movement_enabled_;
            RCLCPP_INFO(node_->get_logger(), "Movement %s", movement_enabled_ ? "enabled" : "disabled");
        }
    }

    // Timer callback to publish the current motor speeds
    void timer_callback()
    {
        auto msg_speeds = std_msgs::msg::UInt8MultiArray();
        msg_speeds.data = { static_cast<uint8_t>(left_speed_),
                            static_cast<uint8_t>(right_speed_) };
        motors_publisher_->publish(msg_speeds);

        RCLCPP_DEBUG(node_->get_logger(),
                     "Motors: L=%d, R=%d",
                     static_cast<int>(msg_speeds.data[0]),
                     static_cast<int>(msg_speeds.data[1]));
    }

    // ROS node handle
    rclcpp::Node::SharedPtr node_;
    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr buttons_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation
    rclcpp::Time start_time_;
    
    // Kinematics calculator
    algorithms::Kinematics kinematics_;

    // Configuration parameters
    const float base_linear_velocity = 0.01f;      // Base forward speed in m/s
    const float steering_gain = -0.01f;             // Gain factor for converting offset to angular velocity
    const float max_angular_velocity = 1.5f;       // Maximum angular velocity in rad/s
    const float max_wheel_speed = 10.0f;            // Maximum wheel speed in rad/s for mapping

    // Button state tracking
    bool movement_enabled_;                        // Flag to enable/disable movement

    // Current motor speeds
    uint8_t left_speed_ = 127;
    uint8_t right_speed_ = 127;
};