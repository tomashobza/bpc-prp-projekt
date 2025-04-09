#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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
  }

  ~LidarNode() {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;

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
      // Only include valid (finite) measurements in the average.
      if (std::isfinite(range_val))
      {
        sum += range_val;
        count++;
      }
    }

    if (count > 0)
      return sum / count;
    else
      return 0.0f;
  }

  // Callback function processes each incoming LaserScan message.
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Log basic information about the scan.
    // RCLCPP_INFO(node_->get_logger(),
    //             "LaserScan received: angle_min: %.2f, angle_max: %.2f, angle_increment: %.4f, ranges count: %zu",
    //             msg->angle_min, msg->angle_max, msg->angle_increment, msg->ranges.size());

    // Define the desired angles for the readings:
    // - Front: assumed to be 0 rad (straight ahead)
    // - Left: computed as half of the minimum angle (you may adjust this based on your setup)
    // - Right: computed as half of the maximum angle
    float front_angle = 0.0f;
    float left_angle = msg->angle_max / 2.0f;
    float right_angle = msg->angle_min / 2.0f;

    // Compute the average distances for each direction using the angular window.
    float front_avg = average_range_at_angle(front_angle, k_angle_window, msg);
    float left_avg  = average_range_at_angle(left_angle,  k_angle_window, msg);
    float right_avg = average_range_at_angle(right_angle, k_angle_window, msg);

    // Log the averaged distances for diagnostics.
    // RCLCPP_INFO(node_->get_logger(),
    //             "Averaged Distances -> Front (%.2f ± %.2f rad): %.2f, Left (%.2f ± %.2f rad): %.2f, Right (%.2f ± %.2f rad): %.2f",
    //             front_angle, k_angle_window, front_avg,
    //             left_angle, k_angle_window, left_avg,
    //             right_angle, k_angle_window, right_avg);
    RCLCPP_INFO(node_->get_logger(), "Offset: %g", right_avg-left_avg);
  }
};
