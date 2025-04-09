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
        RCLCPP_INFO(node_->get_logger(), "Lidar node started!");
        lidar_subscriber_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", 10,
            std::bind(&ControlNode::on_lidar_msg, this, std::placeholders::_1));
        
        speed_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
                "/motor_speeds/update", 10);
        
    };

    ~ControlNode() {}

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr speed_publisher_;
    rclcpp::Node::SharedPtr node_;

    void on_lidar_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto &ranges = msg->ranges;
        size_t num_readings = ranges.size();

        float angle_min = msg->angle_min;
        float angle_increment = msg->angle_increment;

        auto get_avg = [&](float target_angle, float range = M_PI / 18) -> float {
            float sum = 0.0;
            int count = 0;

            for (size_t i = 0; i < num_readings; ++i)
            {
                float angle = angle_min + i * angle_increment;
                if (std::fabs(angle - target_angle) < range)
                {
                    float r = ranges[i];
                    if (std::isfinite(r))
                    {
                        sum += r;
                        ++count;
                    }
                }
            }
            return count > 0 ? sum / count : -1.0; // -1.0 indicates no valid data
        };

        float front_avg = get_avg(0.0);
        float left_avg  = get_avg(M_PI_2);
        float back_avg  = get_avg(M_PI);
        float right_avg = get_avg(-M_PI_2);


        std_msgs::msg::UInt8MultiArray msg_out;
        
        if(std::abs((left_avg - right_avg)) < 0.1) {
            msg_out.data = {140, 140};
            RCLCPP_INFO(node_->get_logger(), "go straight");
        } else if(left_avg > right_avg) {
            msg_out.data = {136, 133};
            RCLCPP_INFO(node_->get_logger(), "go right");
        } else if (right_avg > left_avg){
            msg_out.data = {133, 136};
            RCLCPP_INFO(node_->get_logger(), "go left");
        } else {
            msg_out.data = {127, 127};
        }

        if(back_avg < 0.25) {
            msg_out.data = {124,130};
        }
        
        speed_publisher_->publish(msg_out);

        /*
        
        RCLCPP_INFO(node_->get_logger(),
                    "Front: %.2f m | Left: %.2f m | Back: %.2f m | Right: %.2f m",
                    front_avg, left_avg, back_avg, right_avg);
        */
    }
};