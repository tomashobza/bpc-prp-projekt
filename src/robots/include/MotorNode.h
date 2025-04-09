#pragma once

#include <iostream>
#include <cmath>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "algorithms/pid.h"  // Include the PID logic

class MotorNode
{
public:
    MotorNode(const rclcpp::Node::SharedPtr &node)
        : node_(node),
          start_time_(node_->now()),
          // Initialize PID controllers with the given gains (kp, ki, kd)
          pid_left_(5.0f, 0.2f, 2.0f),
          pid_right_(5.0f, 0.2f, 2.0f)
    {
        const std::string motors_topic = "/bpc_prp_robot/set_motor_speeds";
        const std::string line_topic = "/line_pose";

        // Initialize the motor speeds publisher.
        motors_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(motors_topic, 1);

        // Subscribe to the line position.
        line_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
            line_topic, 1, std::bind(&MotorNode::line_callback, this, std::placeholders::_1));

        // Create a timer for publishing motor speeds (50 Hz).
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorNode::timer_callback, this));

        RCLCPP_INFO(node_->get_logger(), "Motor control node started!");
    }

private:
    // Optional helper to map a PID output into a motor speed between 127 and 255.
    uint8_t map_pid_to_speed(float pid_output)
    {
        // Normalize the PID output to [0,1] assuming it is in [-1, 1].
        float normalized_value = (pid_output + 1.0f) / 2.0f;
        // Map the normalized value to the speed range [127, 255].
        int mapped_value = 127 + static_cast<int>((255 - 127) * normalized_value);
        // Clamp the value between 127 and 255.
        return static_cast<uint8_t>(std::min(255, std::max(127, mapped_value)));
    }

    // Callback to update PID outputs based on the current line offset.
    void line_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        float line_offset = msg->data; // Negative = line on left, positive = line on right.
        if (std::isnan(line_offset))
        {
            line_offset = 0.0f;
        }

        // Define errors: for left wheel the error is the negative offset,
        // for right wheel the error is the positive offset.
        float p_left = -line_offset;
        float p_right = line_offset;

        // Compute the PID outputs using the separated PID logic.
        float pid_output_left = pid_left_.compute(p_left);
        float pid_output_right = pid_right_.compute(p_right);

        // Update wheel speeds based on the PID outputs.
        // (The original mapping in your code swaps the outputs between wheels.)
        right_speed_ = base_speed + static_cast<int>(pid_output_left);
        left_speed_ = base_speed + static_cast<int>(pid_output_right);

        RCLCPP_INFO(node_->get_logger(), "PID_L: %f, PID_R: %f", pid_output_left, pid_output_right);
    }

    // Timer callback to publish the current motor speeds.
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

    // ROS node handle.
    rclcpp::Node::SharedPtr node_;
    // Publishers and subscribers.
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr line_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation.
    rclcpp::Time start_time_;
    size_t SIZE = 10;
    std::deque<float> buffer;

    // PID controllers for left and right wheels.
    PIDController pid_left_;
    PIDController pid_right_;

    // Configuration parameters.
    const uint8_t base_speed = 130;
    const uint8_t max_speed = 255;

    // Current motor speeds.
    uint8_t left_speed_ = 130;
    uint8_t right_speed_ = 130;
};
