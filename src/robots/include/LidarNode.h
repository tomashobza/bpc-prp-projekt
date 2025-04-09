#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cmath>
#include <limits>
#include <algorithm>

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

    // Create a publisher for the averaged measurements (order: front, right, left).
    lidar_avg_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/bpc_prp_robot/lidar_avg", 10);
  }

  ~LidarNode() {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_avg_publisher_;

  // Static constant for the angular window (in radians) used for averaging.
  static constexpr float k_angle_window = M_PI / 20.0f;

  // Helper function to compute the index in the ranges array corresponding to a given angle.
  int compute_index_for_angle(float desired_angle, const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float relative_angle = desired_angle - msg->angle_min;
    return static_cast<int>(std::round(relative_angle / msg->angle_increment));
  }

  // Helper function that computes the average range measurement around a desired angle.
  // It averages values within (desired_angle - k) and (desired_angle + k), ignoring non-finite measurements.
  float average_range_at_angle(float desired_angle, float k, const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    float lower_angle = desired_angle - k;
    float upper_angle = desired_angle + k;

    int lower_index = compute_index_for_angle(lower_angle, msg);
    int upper_index = compute_index_for_angle(upper_angle, msg);

    // Clamp the computed indices to valid bounds.
    lower_index = std::max(lower_index, 0);
    upper_index = std::min(upper_index, static_cast<int>(msg->ranges.size()) - 1);

    float sum = 0.0f;
    int count = 0;

    // Iterate over the indices in the window.
    for (int i = lower_index; i <= upper_index; i++)
    {
      float range_val = msg->ranges[i];
      // Only include valid (finite) measurements.
      if (std::isfinite(range_val))
      {
        sum += range_val;
        count++;
      }
    }

    return (count > 0) ? (sum / static_cast<float>(count)) : 0.0f;
  }

  // Callback function processes each incoming LaserScan message.
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Define the desired angles for the readings:
    float front_angle = msg->angle_max;
    float left_angle = msg->angle_max / 2.0f;
    float right_angle = msg->angle_min / 2.0f;

    // Compute the average distances for each direction using the angular window.
    float front_avg = average_range_at_angle(front_angle, k_angle_window, msg);
    float left_avg  = average_range_at_angle(left_angle,  k_angle_window, msg);
    float right_avg = average_range_at_angle(right_angle, k_angle_window, msg);

    // Log the offset (for example, right minus left).
    // RCLCPP_INFO(node_->get_logger(), "Offset (Right - Left): %g", right_avg - left_avg);

    // Create a message to publish the averaged measurements: order: front, right, left.
    std_msgs::msg::Float32MultiArray avg_msg;
    avg_msg.data.resize(3);
    avg_msg.data[0] = front_avg;
    avg_msg.data[1] = right_avg;
    avg_msg.data[2] = left_avg;

    // Publish the averaged measurements.
    lidar_avg_publisher_->publish(avg_msg);
  }
};
