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
        const std::string update_topic = "/motor_speeds/update";

        // Initialize the motor speeds publisher
        motors_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(motors_topic, 1);

        motor_update_subscriber_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
            update_topic, 10,
            std::bind(&MotorNode::update_motor_speeds_callback, this, std::placeholders::_1));

        // Create timer for publishing motor speeds (50Hz)
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&MotorNode::timer_callback, this));

        RCLCPP_INFO(node_->get_logger(), "Motor control node started!");
    }

private:
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

    void update_motor_speeds_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2)
        {
            left_speed_ = msg->data[0];
            right_speed_ = msg->data[1];
            RCLCPP_INFO(node_->get_logger(), "Updated speeds -> L: %d, R: %d", left_speed_, right_speed_);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Invalid motor update received (size=%zu)", msg->data.size());
        }
    }

    // Shared pointer to the main ROS node
    rclcpp::Node::SharedPtr node_;

    // Publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_update_subscriber_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation
    rclcpp::Time start_time_;

    // Current motor speeds
    uint8_t left_speed_ = 127;
    uint8_t right_speed_ = 127;
};