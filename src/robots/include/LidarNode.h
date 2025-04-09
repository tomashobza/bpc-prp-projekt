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


class LidarNode
{
public:
    LidarNode(const rclcpp::Node::SharedPtr &node)
        : node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "Lidar node started!");
        lidar_subscriber_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", 10,
            std::bind(&LidarNode::on_lidar_msg, this, std::placeholders::_1));
    };

    ~LidarNode() {}

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
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

        RCLCPP_INFO(node_->get_logger(),
                    "Front: %.2f m | Left: %.2f m | Back: %.2f m | Right: %.2f m",
                    front_avg, left_avg, back_avg, right_avg);
    }
};