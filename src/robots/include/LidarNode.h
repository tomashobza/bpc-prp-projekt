#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <cmath>
#include <limits>
#include <algorithm>
#include "algorithms/turns.hpp"

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

    // Create a publisher for the detected turn type
    lidar_turn_publisher_ = node_->create_publisher<std_msgs::msg::Int8>(
        "/bpc_prp_robot/detected_turn", 10);
  }

  ~LidarNode() {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_avg_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr lidar_turn_publisher_;

  // Static constant for the angular window (in radians) used for averaging.
  static constexpr float k_angle_window = M_PI / 40.0f;

  // Distance threshold to determine if a direction is "open"
  static constexpr float k_open_threshold = 0.5f;

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

  TurnType get_turn(float front, float left, float right)
  {
    // Determine which directions are "open" based on a threshold
    bool is_front_open = front > k_open_threshold;
    bool is_left_open = left > k_open_threshold;
    bool is_right_open = right > k_open_threshold;

    // Check for blind turn first (all directions closed)
    if (!is_front_open && !is_left_open && !is_right_open)
    {
      return TurnType::BLIND_TURN; // New enum value for when all directions are closed
    }

    // Decision tree for all possible turn types
    if (is_left_open && is_right_open && is_front_open)
    {
      return TurnType::CROSS; // All directions open = crossroad
    }
    else if (is_left_open && is_right_open && !is_front_open)
    {
      return TurnType::T_TURN; // Left and right open, front closed = T junction
    }
    else if (is_left_open && is_front_open && !is_right_open)
    {
      return TurnType::LEFT_FRONT; // Left and front open
    }
    else if (is_right_open && is_front_open && !is_left_open)
    {
      return TurnType::RIGHT_FRONT; // Right and front open
    }
    else if (is_left_open && !is_right_open && !is_front_open)
    {
      return TurnType::LEFT; // Only left open
    }
    else if (is_right_open && !is_left_open && !is_front_open)
    {
      return TurnType::RIGHT; // Only right open
    }

    // If we somehow get here (shouldn't happen with complete logic above)
    return TurnType::BLIND_TURN; // Default to blind turn as safest option
  }

  // Callback function processes each incoming LaserScan message.
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Define the desired angles for the readings:
    float front_angle = msg->angle_max;
    float left_angle = msg->angle_min / 1.5f;
    float right_angle = msg->angle_max / 1.5f;

    float hard_left = msg->angle_min / 2.0f;
    float hard_right = msg->angle_max / 2.0f;

    // float detection_left = msg->angle_min * (3.0f / 4.0f);
    // float detection_right = msg->angle_max * (3.0f / 4.0f);

    // Compute the average distances for each direction using the angular window.
    float front_avg = average_range_at_angle(front_angle, k_angle_window, msg);
    float left_avg = average_range_at_angle(left_angle, k_angle_window, msg);
    float right_avg = average_range_at_angle(right_angle, k_angle_window, msg);

    float hard_left_avg = average_range_at_angle(hard_left, k_angle_window, msg);
    float hard_right_avg = average_range_at_angle(hard_right, k_angle_window, msg);

    float detection_right_avg = right_avg;
    float detection_left_avg = left_avg;
    // float detection_right_avg = average_range_at_angle(detection_right, k_angle_window, msg);
    // float detection_left_avg = average_range_at_angle(detection_left, k_angle_window, msg);

    // Determine the turn type based on the measurements
    TurnType turn = get_turn(front_avg, hard_left_avg, hard_right_avg);

    // if both walls gone, mock walls
    if (left_avg > 0.45 && right_avg > 0.45)
    {
      left_avg = 0.2;
      right_avg = 0.2;
    }

    // if right wall is gone, mock right from left
    if (left_avg > 0.3)
    {
      left_avg = 0.43 - right_avg;
    }

    // if left wall is gone, mock left from left
    if (right_avg > 0.3)
    {
      right_avg = 0.43 - left_avg;
    }

    // Create a message to publish the averaged measurements: order: front, right, left.
    std_msgs::msg::Float32MultiArray avg_msg;
    avg_msg.data.resize(5);
    avg_msg.data[0] = front_avg;           // front
    avg_msg.data[1] = right_avg;           // right
    avg_msg.data[2] = left_avg;            // left
    avg_msg.data[3] = detection_right_avg; // single right
    avg_msg.data[4] = detection_left_avg;  // single left

    // Create a separate message for the turn type
    std_msgs::msg::Int8 turn_msg;
    turn_msg.data = static_cast<int8_t>(turn);

    // Publish both messages
    lidar_avg_publisher_->publish(avg_msg);
    lidar_turn_publisher_->publish(turn_msg);

    // Optionally log the detected turn
    // RCLCPP_INFO(node_->get_logger(), "Detected turn: %d (F: %g, L: %g, R: %g)", static_cast<int>(turn), front_avg, hard_left_avg, hard_right_avg);
  }
};