
#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>

class MotorNode {

public:
    // Constructor takes a shared_ptr to an existing node instead of creating one.
    MotorNode(const rclcpp::Node::SharedPtr &node)
        : node_(node), start_time_(node_->now()), left_wheel_speed_(130), right_wheel_speed_(130) {

        const double freq = 0.1;
        const std::string publish_topic = "/bpc_prp_robot/set_motor_speeds";
        const std::string subscribe_topic = "/bpc_prp_robot/encoders";
        const std::string line_sensor_topic = "/bpc_prp_robot/line_sensors";

        // Initialize the publisher
        publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(publish_topic, 1);

        // Initialize the subscriber
        subscriber_ = node_->create_subscription<std_msgs::msg::UInt32MultiArray>(
            subscribe_topic, 1, std::bind(&MotorNode::subscriber_callback, this, std::placeholders::_1));

        line_sensor_subscriber_ = node_->create_subscription<std_msgs::msg::UInt16MultiArray>(
            line_sensor_topic, 1, std::bind(&MotorNode::line_sensor_callback, this, std::placeholders::_1));

        

        // Create a timer
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(freq * 1000)),
            std::bind(&MotorNode::timer_callback, this));

        RCLCPP_INFO(node_->get_logger(), "Subscribing to line sensor topic: %s", line_sensor_topic.c_str());

            // And check if the subscription is valid
        if (line_sensor_subscriber_) {
            RCLCPP_INFO(node_->get_logger(), "Line sensor subscription created successfully");
                } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create line sensor subscription");
        }

        // RCLCPP_INFO(node_->get_logger(), "Node setup complete for topic: %s", topic.c_str());
        RCLCPP_INFO(node_->get_logger(), "A jedeeeem!");
    }

private:
    void timer_callback() {
        RCLCPP_INFO(node_->get_logger(), "Timer triggered. Publishing uptime...");

        double uptime = (node_->now() - start_time_).seconds();
        publish_message(uptime);
    }

    void subscriber_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        //RCLCPP_INFO(node_->get_logger(), "Received: %d", msg->data[0]);
    }

    void publish_message(float value_to_publish) {
        auto msg = std_msgs::msg::UInt8MultiArray();
        // msg.data = value_to_publish;
        msg.data = {left_wheel_speed_, right_wheel_speed_};
        publisher_->publish(msg);
        // RCLCPP_INFO(node_->get_logger(), "Published: %f", msg.data);
    }

    void line_sensor_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
        std::string sensor_values = "Line Sensor Data: ";
        if(msg->data[0] > 400){
            right_wheel_speed_ = 140;
        } else {
            right_wheel_speed_= 130;
        }

        if(msg->data[1] > 400) {
            left_wheel_speed_ = 140;
        } else {
            left_wheel_speed_ = 130;
        }

        for (const auto &value : msg->data) {

            sensor_values += std::to_string(value) + " ";
        }
        RCLCPP_INFO(node_->get_logger(), "%s", sensor_values.c_str());
    }

    // Shared pointer to the main ROS node
    rclcpp::Node::SharedPtr node_;

    // Variables for wheel speeds
    uint8_t left_wheel_speed_;
    uint8_t right_wheel_speed_;

    // Publisher, subscriber, and timer
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr line_sensor_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation
    rclcpp::Time start_time_;
};

