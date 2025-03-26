#pragma once
#include <iostream>
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
    void line_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        float line_offset = msg->data; // negative = line on left, positive = line on right

        // Base speed for forward motion (in the upper half of uint8 range)
        const uint8_t base_speed = 130; // Choose a value between 127 and 255
        const float max_speed_diff = 5; // Maximum speed difference between wheels

        // Calculate speed difference based on line offset
        // line_offset is typically between -1 and 1
        // float speed_diff = line_offset * max_speed_diff;
        // float speed_diff = 0;
        // if (line_offset > 0)
        // {
        //     speed_diff = -line_offset * max_speed_diff;
        // }
        // else
        // {
        //     speed_diff = line_offset * max_speed_diff;
        // }
        // RCLCPP_INFO(node_->get_logger(), "speed_diff: %f", speed_diff);

        // Calculate left and right wheel speeds
        // left_speed_ = base_speed + speed_diff;
        // right_speed_ = base_speed - speed_diff;
        // if (line_offset > 0.1)
        // {
        //     left_speed_ = base_speed;
        //     right_speed_ = base_speed - line_offset * max_speed_diff;
        // }
        // else if (line_offset < -0.1)
        // {
        //     left_speed_ = base_speed + line_offset * max_speed_diff;
        //     right_speed_ = base_speed;
        // }
        // else
        // {
        //     left_speed_ = base_speed;
        //     right_speed_ = base_speed;
        // }
        // if (line_offset < 0.1)
        // {
        //     left_speed_ = base_speed;
        //     right_speed_ = base_speed - line_offset * max_speed_diff;
        //     RCLCPP_INFO(node_->get_logger(), "jedu vlevo %f", line_offset);
        // }

        // else if (line_offset > 0.1)
        // {
        //     left_speed_ = base_speed + line_offset * max_speed_diff;
        //     right_speed_ = base_speed;
        //     RCLCPP_INFO(node_->get_logger(), "jedu vpravo %f", line_offset);
        // }
        // else
        // {
        //     left_speed_ = base_speed + 5;
        //     right_speed_ = base_speed + 5;
        //     RCLCPP_INFO(node_->get_logger(), "jedu rovne %f", line_offset);
        // }
        left_speed_ = base_speed + line_offset * max_speed_diff;
        right_speed_ = base_speed - line_offset * max_speed_diff;

        // RCLCPP_INFO(node_->get_logger(), "left_speed: %f, right_speed: %f", left_speed_, right_speed_);

        // Ensure speeds stay within valid range (127-255)
        left_speed_ = std::min(255.0f, std::max(127.0f, left_speed_));
        right_speed_ = std::min(255.0f, std::max(127.0f, right_speed_));
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

    // Current motor speeds
    float left_speed_{130.0f};
    float right_speed_{130.0f};
};