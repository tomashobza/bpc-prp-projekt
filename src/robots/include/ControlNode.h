#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <mutex>
#include <cmath>
#include <numeric>


class ControlNode
{
public:
    ControlNode(const rclcpp::Node::SharedPtr &node)
        : node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "Control node started!");

        /*
        lidar_subscriber_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", 10,
            std::bind(&ControlNode::on_lidar_msg, this, std::placeholders::_1));
        
        speed_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
                "/motor_speeds/update", 10);
        */
        
    };

    ~ControlNode() {}

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr speed_publisher_;
    rclcpp::Node::SharedPtr node_;

    void on_lidar_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std_msgs::msg::UInt8MultiArray msg_out;
        msg_out.data = {127,127};
        speed_publisher_->publish(msg_out);

        /*
        
        RCLCPP_INFO(node_->get_logger(),
                    "Front: %.2f m | Left: %.2f m | Back: %.2f m | Right: %.2f m",
                    front_avg, left_avg, back_avg, right_avg);
        */
    }
};