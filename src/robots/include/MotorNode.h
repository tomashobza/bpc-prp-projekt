#pragma once
#include <iostream>
#include <cmath>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

class MotorNode
{
public:
    MotorNode(const rclcpp::Node::SharedPtr &node)
        : node_(node), start_time_(node_->now())
    {

        const std::string motors_topic = "/bpc_prp_robot/set_motor_speeds";
        const std::string line_topic = "/line_pose";

        // Initialize the motor speeds publisher
        motors_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(motors_topic, 1);

        // Subscribe to line position
        line_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
            line_topic, 1, std::bind(&MotorNode::line_callback, this, std::placeholders::_1));

        // Create timer for publishing motor speeds (50Hz)
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorNode::timer_callback, this));

        RCLCPP_INFO(node_->get_logger(), "Motor control node started!");
    }

private:
    uint8_t map_pid_to_speed(float pid_output)
    {
        // Normalize the PID output to [0, 1] if it is in [-1, 1] range
        float normalized_value = (pid_output + 1.0f) / 2.0f;

        // Map the normalized value to the speed range [127, 255]
        int mapped_value = 127 + static_cast<int>((255 - 127) * normalized_value);

        // Ensure the value is within the range [127, 255]
        return static_cast<uint8_t>(std::min(255, std::max(127, mapped_value)));
    }

    void line_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        float line_offset = msg->data; // negative = line on left, positive = line on right
        if (std::isnan(line_offset))
        {
            line_offset = 0;
        }

        // Add deadband to ignore very small errors
        const float DEADBAND = 0.05f;
        if (std::abs(line_offset) < DEADBAND)
        {
            line_offset = 0;
        }

        // Calculate errors
        float p_left = -line_offset;
        float p_right = line_offset;

        // Add smoothing - use moving average for the error
        buffer.push_back(line_offset);
        if (buffer.size() > SIZE)
        {
            buffer.pop_front();
        }

        // Calculate average error
        float avg_error = 0;
        for (const auto &err : buffer)
        {
            avg_error += err;
        }
        avg_error /= buffer.size();

        // Use smoothed error for proportional term
        p_left = -avg_error;
        p_right = avg_error;

        // Integral term with anti-windup
        error_sum_left += p_left;
        error_sum_right += p_right;

        const float MAX_ERROR_SUM = 0.5f; // Reduced from 1.0
        error_sum_left = std::min(std::max(error_sum_left, -MAX_ERROR_SUM), MAX_ERROR_SUM);
        error_sum_right = std::min(std::max(error_sum_right, -MAX_ERROR_SUM), MAX_ERROR_SUM);

        // Derivative term - using unfiltered error for responsiveness
        float d_left = p_left - last_error_left;
        float d_right = p_right - last_error_right;

        // Suggested PID values - lower P, higher D
        // Try starting with: kp_left = 10.0, kd_left = 5.0, ki_left = 0.1

        // Calculate PID output for each wheel
        float pid_output_left = kp_left * p_left + ki_left * error_sum_left + kd_left * d_left;
        float pid_output_right = kp_right * p_right + ki_right * error_sum_right + kd_right * d_right;

        // Limit PID output
        const float MAX_PID_OUTPUT = 50.0f;
        pid_output_left = std::min(std::max(pid_output_left, -MAX_PID_OUTPUT), MAX_PID_OUTPUT);
        pid_output_right = std::min(std::max(pid_output_right, -MAX_PID_OUTPUT), MAX_PID_OUTPUT);

        // Calculate final speeds
        int left_speed_int = base_speed + static_cast<int>(pid_output_right); // Note: swapped based on your code
        int right_speed_int = base_speed + static_cast<int>(pid_output_left);

        // Ensure speeds stay within valid range (127-255)
        left_speed_ = static_cast<uint8_t>(std::min(255, std::max(127, left_speed_int)));
        right_speed_ = static_cast<uint8_t>(std::min(255, std::max(127, right_speed_int)));

        // Update last error for next iteration
        last_error_left = p_left;
        last_error_right = p_right;

        RCLCPP_INFO(node_->get_logger(), "PID_L: %f, PID_R: %f", pid_output_left, pid_output_right);
    }

    void timer_callback()
    {
        // Publish motor speeds
        auto msg_speeds = std_msgs::msg::UInt8MultiArray();
        msg_speeds.data = {static_cast<uint8_t>(left_speed_),
                           static_cast<uint8_t>(right_speed_)};
        motors_publisher_->publish(msg_speeds);

        RCLCPP_DEBUG(node_->get_logger(),
                     "Motors: L=%d, R=%d",
                     static_cast<int>(msg_speeds.data[0]),
                     static_cast<int>(msg_speeds.data[1]));
    }

    // Shared pointer to the main ROS node
    rclcpp::Node::SharedPtr node_;

    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr line_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation
    rclcpp::Time start_time_;

    size_t SIZE = 10;
    std::deque<float> buffer;

    // PID constants
    float kp_left = 8.0;
    float ki_left = 0.05;
    float kd_left = 5.0;

    float kp_right = kp_left;
    float ki_right = ki_left;
    float kd_right = kd_left;

    // PID error terms
    float error_sum_left = 0.0;
    float error_sum_right = 0.0;

    float last_error_left = 0.0;
    float last_error_right = 0.0;

    const uint8_t base_speed = 140;
    const uint8_t max_speed = 255;

    // Current motor speeds
    uint8_t left_speed_ = 130;
    uint8_t right_speed_ = 130;
};