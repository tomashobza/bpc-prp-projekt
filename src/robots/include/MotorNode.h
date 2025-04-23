#pragma once

#include <iostream>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "algorithms/kinematics.hpp"  // Include the kinematics logic

class MotorNode
{
public:
  MotorNode(const rclcpp::Node::SharedPtr &node)
    : node_(node),
      // Initialize kinematics with wheel radius (m), wheel base (m), and ticks per rotation
      kinematics_(0.033, 0.16, 360),
      movement_enabled_(false)
  {
    const std::string motors_topic  = "/bpc_prp_robot/set_motor_speeds";
    const std::string lidar_topic   = "/bpc_prp_robot/lidar_avg";
    const std::string buttons_topic = "/bpc_prp_robot/buttons";

    // Set up publisher for motor commands
    motors_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(motors_topic, 1);

    // Subscribe to lidar averaged data
    lidar_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
      lidar_topic, 1, std::bind(&MotorNode::lidar_callback, this, std::placeholders::_1));

    // Subscribe to button inputs
    buttons_subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
      buttons_topic, 1, std::bind(&MotorNode::buttons_callback, this, std::placeholders::_1));

    // Create a timer to publish motor commands at 50 Hz (20 ms)
    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&MotorNode::timer_callback, this));

    RCLCPP_INFO(node_->get_logger(), "Simplified motor control node started with improved speed mapping!");
  }

private:
  // Convert wheel speed (m/s) to a motor command.
  // The conversion multiplies the wheel speed by a coefficient and adds 127 (the stop/neutral value).
  // Positive speeds will result in command values greater than 127, and negative speeds in values lower than 127.
  // Commands are clamped between 0 and 255.
  uint8_t convert_speed_to_command(float wheel_speed)
  {
    int command = 127 + static_cast<int>(std::round(speed_coefficient_ * wheel_speed));
    command = std::min(255, std::max(0, command));
    return static_cast<uint8_t>(command);
  }

  // Process lidar data and update wheel speeds accordingly.
  // The node checks for an obstacle in front before computing the speeds.
  // It computes an angular velocity using a steering gain multiplied by the difference
  // between right and left distances and sets a base linear velocity when movement is enabled.
  void lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() >= 3)
    {
      float front_dist = msg->data[0];
      float right_dist = msg->data[1];
      float left_dist  = msg->data[2];

      // Emergency stop if an obstacle is too close in front.
      if (front_dist < 0.2)
      {
        movement_enabled_ = false;
        RCLCPP_WARN(node_->get_logger(), "Emergency stop: obstacle too close (%.2f m)", front_dist);
        left_command_  = 127;
        right_command_ = 127;
        return;
      }

      if (last_left_dist != -1) {
        float dist_diff_left = std::fabs(left_dist - last_left_dist);
        float dist_diff_right = std::fabs(right_dist - last_right_dist);
        if (dist_diff_left > 0.2) {
          RCLCPP_INFO(node_->get_logger(), "ZATACKA VLEVO");
        }
        if (dist_diff_right > 0.2) {
          RCLCPP_INFO(node_->get_logger(), "ZATACKA VPRAVO");
        }
      }

      // Calculate the corridor offset: positive if closer to left wall, negative if closer to right wall.
      float corridor_offset = left_dist - right_dist;

      // Compute angular velocity with a global steering gain.
      // TODO: connect PID controller to the angular velocity
      float angular_velocity = global_steering_gain_ * corridor_offset;

      // Set the linear velocity if movement is enabled, otherwise use 0.
      float linear_velocity = base_linear_velocity_;

      // Calculate desired robot speed.
      algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);

      // RCLCPP_INFO(node_->get_logger(), "angular_speed: %g", angular_velocity);

      // Use inverse kinematics to compute wheel speeds (in m/s).
      algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);

      // RCLCPP_INFO(node_->get_logger(), "raw left: %f, raw right: %f", wheel_speeds.l, wheel_speeds.r);

      // Convert the wheel speeds to motor commands using the simplified approach.
      if (movement_enabled_) {
          left_command_  = convert_speed_to_command(wheel_speeds.l);
          right_command_ = convert_speed_to_command(wheel_speeds.r);
      } else {
        left_command_ = 127;
        right_command_ = 127;
      }

      // RCLCPP_INFO(node_->get_logger(), "mapped left: %d, mapped right: %d", left_command_, right_command_);

      // save lidar reading
      last_left_dist = left_dist;
      last_right_dist = right_dist;
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Unexpected lidar data format!");
    }
  }

  // Toggle movement enabled/disabled based on button press.
  void buttons_callback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    RCLCPP_INFO(node_->get_logger(), "Button pressed %d", msg->data);
    if (msg->data == 0)
    {
      movement_enabled_ = !movement_enabled_;
      RCLCPP_INFO(node_->get_logger(), "Movement %s", movement_enabled_ ? "enabled" : "disabled");
    }
  }

  // Publish the current motor commands at a fixed rate.
  void timer_callback()
  {
    std_msgs::msg::UInt8MultiArray msg_commands;
    msg_commands.data = { left_command_, right_command_ };
    motors_publisher_->publish(msg_commands);

    RCLCPP_DEBUG(node_->get_logger(), "Motors: left=%d, right=%d", left_command_, right_command_);
  }

  // ROS node handle, publishers, subscribers, and timer.
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_subscriber_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr buttons_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Kinematics object for inverse kinematics calculations.
  algorithms::Kinematics kinematics_;

  // Flag to determine if movement is enabled.
  bool movement_enabled_;

  // Motor command values (127 represents a stop).
  uint8_t left_command_  = 127;
  uint8_t right_command_ = 127;

  // Save last lidar reading
  float last_right_dist = -1;
  float last_left_dist = -1;

  // Coefficient to convert wheel speed (m/s) to a motor command increment.
  const float speed_coefficient_ = 10.0f;  // Adjust this value for the desired sensitivity

  // Base linear velocity (in m/s) when movement is enabled.
  const float base_linear_velocity_ = 0.02f;

  // Steering gain used to compute angular velocity from the corridor offset.
  const float global_steering_gain_ = 1.0f;
};
