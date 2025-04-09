#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <limits>

class LidarNode
{
public:
  explicit LidarNode(const rclcpp::Node::SharedPtr &node)
    : node_(node)
  {
    RCLCPP_INFO(node_->get_logger(), "Lidar Node started!");

    // Create a subscription to the "/bpc_prp_robot/lidar" topic with a queue size of 10.
    lidar_subscription_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/bpc_prp_robot/lidar", 10,
      std::bind(&LidarNode::lidar_callback, this, std::placeholders::_1));
  }

  ~LidarNode() {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;

  // Helper function to compute the index for a given angle.
  int compute_index_for_angle(float desired_angle, const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float relative_angle = desired_angle - msg->angle_min;
    return static_cast<int>(std::round(relative_angle / msg->angle_increment));
  }

  // Callback function that processes each incoming LaserScan message.
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Log some general information about the incoming scan.
    RCLCPP_INFO(node_->get_logger(),
                "LaserScan received: angle_min: %.2f, angle_max: %.2f, angle_increment: %.4f, ranges count: %zu",
                msg->angle_min, msg->angle_max, msg->angle_increment, msg->ranges.size());

    // float back_angle = 0.0f;
    float front_angle = msg->angle_max;
    float right_angle = msg->angle_max / 2.0f;
    float left_angle = msg->angle_min / 2.0f;

    // Compute the indices corresponding to these angles.
    int front_index = compute_index_for_angle(front_angle, msg);
    // int back_index  = compute_index_for_angle(back_angle,  msg);
    int left_index  = compute_index_for_angle(left_angle,  msg);
    int right_index = compute_index_for_angle(right_angle, msg);

    // Retrieve the distances using the computed indices. If an index is out of range, return NaN.
    float front_distance = (front_index >= 0 && static_cast<size_t>(front_index) < msg->ranges.size())
                             ? msg->ranges[front_index] : std::numeric_limits<float>::quiet_NaN();
    // float back_distance  = (back_index  >= 0 && static_cast<size_t>(back_index)  < msg->ranges.size())
    //                          ? msg->ranges[back_index]  : std::numeric_limits<float>::quiet_NaN();
    float left_distance  = (left_index  >= 0 && static_cast<size_t>(left_index)  < msg->ranges.size())
                             ? msg->ranges[left_index]  : std::numeric_limits<float>::quiet_NaN();
    float right_distance = (right_index >= 0 && static_cast<size_t>(right_index) < msg->ranges.size())
                             ? msg->ranges[right_index] : std::numeric_limits<float>::quiet_NaN();

    // Log the distances for the calculated directions.
    RCLCPP_INFO(node_->get_logger(),
                "Distances -> Front: %.2f, Left: %.2f, Right: %.2f",
                front_distance,left_distance,right_distance);
  }
};
