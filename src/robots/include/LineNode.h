#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <mutex>
#include <deque>
#include "LineEstimator.h"

class LineNode
{
public:
    LineNode(const rclcpp::Node::SharedPtr &node)
        : node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "Line node started!");
        line_sensors_subscriber_ = node_->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "/bpc_prp_robot/line_sensors", 10,
            std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));

        line_pose_publisher_ = node_->create_publisher<std_msgs::msg::Float32>(
            "/line_pose", 10);

        // New publisher for sensor min/max values
        sensor_minmax_publisher_ = node_->create_publisher<std_msgs::msg::UInt16MultiArray>(
            "/line_sensor_minmax", 10);
    };

    ~LineNode() {};

    // relative pose to line in meters
    float get_continuous_line_pose() const
    {
        return LineEstimator::estimate_continuous_line_pose(left_normalized_, right_normalized_);
    }

    DiscreteLinePose get_discrete_line_pose() const
    {
        return LineEstimator::estimate_discrete_line_pose(left_normalized_, right_normalized_);
    }

private:
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensors_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr line_pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr sensor_minmax_publisher_;
    rclcpp::Node::SharedPtr node_;

    // Moving average filter buffer for line_pose values
    std::deque<float> line_pose_buffer_;
    const size_t buffer_size_ = 10; // Store last 10 values

    void on_line_sensors_msg(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2)
        {
            RCLCPP_WARN(node_->get_logger(), "Received incomplete sensor data");
            return;
        }

        float left_value = static_cast<float>(msg->data[0]);
        float right_value = static_cast<float>(msg->data[1]);

        // Update min/max values for calibration
        left_min_ = std::min(left_min_, left_value);
        left_max_ = std::max(left_max_, left_value);
        right_min_ = std::min(right_min_, right_value);
        right_max_ = std::max(right_max_, right_value);

        // Normalize sensor values to 0-1 range
        left_normalized_ = (left_value - left_min_) / (left_max_ - left_min_);
        right_normalized_ = (right_value - right_min_) / (right_max_ - right_min_);

        // Calculate the current line pose
        float current_line_pose = get_continuous_line_pose();

        // Add to buffer
        line_pose_buffer_.push_back(current_line_pose);

        // Keep buffer size limited
        if (line_pose_buffer_.size() > buffer_size_)
        {
            line_pose_buffer_.pop_front();
        }

        // Calculate average of values in buffer
        float sum = 0.0f;
        for (const auto &value : line_pose_buffer_)
        {
            sum += value;
        }
        float averaged_line_pose = sum / static_cast<float>(line_pose_buffer_.size());

        // Publish the filtered line pose
        auto msg_pose = std_msgs::msg::Float32();
        msg_pose.data = averaged_line_pose;
        line_pose_publisher_->publish(msg_pose);

        // Publish the min/max values for both sensors
        auto msg_minmax = std_msgs::msg::UInt16MultiArray();
        msg_minmax.data = {
            static_cast<uint16_t>(left_min_),
            static_cast<uint16_t>(left_max_),
            static_cast<uint16_t>(right_min_),
            static_cast<uint16_t>(right_max_)};
        sensor_minmax_publisher_->publish(msg_minmax);
    }

    float left_min_{1024.0f};
    float left_max_{0.0f};
    float right_min_{1024.0f};
    float right_max_{0.0f};
    float left_normalized_{0.0f};
    float right_normalized_{0.0f};
};