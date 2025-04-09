#pragma once

#include <iostream>
#include <cmath>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "algorithms/pid.h"  // Include the PID logic
#include "algorithms/kinematics.hpp"  // Include the kinematics logic

class MotorNode
{
public:
    MotorNode(const rclcpp::Node::SharedPtr &node)
        : node_(node),
          start_time_(node_->now()),
          // Initialize PID controller with the given gains (kp, ki, kd)
          pid_corridor_(2.0f, 0.1f, 1.0f),
          // Initialize kinematics with wheel radius (m), wheel base (m), and ticks per rotation
          kinematics_(0.033, 0.16, 360)
    {
        const std::string motors_topic = "/bpc_prp_robot/set_motor_speeds";
        const std::string lidar_topic = "/bpc_prp_robot/lidar_avg";

        // Initialize the motor speeds publisher.
        motors_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(motors_topic, 1);

        // Subscribe to the lidar averaged data.
        lidar_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            lidar_topic, 1, std::bind(&MotorNode::lidar_callback, this, std::placeholders::_1));

        // Create a timer for publishing motor speeds (50 Hz).
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MotorNode::timer_callback, this));

        RCLCPP_INFO(node_->get_logger(), "Motor control node started with corridor centering!");
    }

private:
    // Optional helper to map a wheel speed to motor command between 127 and 255.
    uint8_t map_speed_to_motor_cmd(float wheel_speed)
    {
        // Normalize the wheel speed to a motor command in [127, 255]
        // Assuming wheel_speed range is [-max_wheel_speed, max_wheel_speed]
        float normalized_value = (wheel_speed + max_wheel_speed) / (2 * max_wheel_speed);
        int mapped_value = 127 + static_cast<int>((255 - 127) * normalized_value);
        // Clamp the value between 127 and 255.
        return static_cast<uint8_t>(std::min(255, std::max(127, mapped_value)));
    }

    // Callback to update wheel speeds based on the lidar measurements.
    void lidar_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Ensure we have the expected data format (front, right, left)
        if (msg->data.size() >= 3)
        {
            float front_dist = msg->data[0];
            float right_dist = msg->data[1];
            float left_dist = msg->data[2];

            // Calculate corridor offset (positive if closer to left wall, negative if closer to right wall)
            float corridor_offset = right_dist - left_dist;
            
            // Calculate desired corridor center (adjust if we want to bias toward one side)
            // If corridor_offset is 0, we're in the center
            float desired_offset = 0.0f;  // We want to be in the center
            float error = desired_offset - corridor_offset;

            // Compute the PID output to get angular velocity
            float angular_velocity = pid_corridor_.compute(error);
            
            // Limit the maximum angular velocity
            angular_velocity = std::min(std::max(angular_velocity, -max_angular_velocity), max_angular_velocity);

            // Set linear velocity 
            float linear_velocity = base_linear_velocity;
            
            // Slow down if approaching an obstacle in front
            if (front_dist < front_distance_threshold && front_dist > 0.0f)
            {
                linear_velocity *= std::min(1.0f, front_dist / front_distance_threshold);
            }

            // Create robot speed structure
            algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);
            
            // Use inverse kinematics to get wheel speeds
            algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);
            
            // Map wheel speeds to motor commands and store them
            left_speed_ = map_speed_to_motor_cmd(wheel_speeds.l);
            right_speed_ = map_speed_to_motor_cmd(wheel_speeds.r);

            RCLCPP_INFO(node_->get_logger(), 
                        "Corridor: L=%.2f, R=%.2f, Offset=%.2f, Angular=%.2f, Linear=%.2f", 
                        left_dist, right_dist, corridor_offset, angular_velocity, linear_velocity);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "Unexpected lidar data format!");
        }
    }

    // Timer callback to publish the current motor speeds.
    void timer_callback()
    {
        auto msg_speeds = std_msgs::msg::UInt8MultiArray();
        msg_speeds.data = { static_cast<uint8_t>(left_speed_),
                            static_cast<uint8_t>(right_speed_) };
        motors_publisher_->publish(msg_speeds);

        RCLCPP_DEBUG(node_->get_logger(),
                     "Motors: L=%d, R=%d",
                     static_cast<int>(msg_speeds.data[0]),
                     static_cast<int>(msg_speeds.data[1]));
    }

    // ROS node handle.
    rclcpp::Node::SharedPtr node_;
    // Publishers and subscribers.
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation.
    rclcpp::Time start_time_;

    // PID controller for corridor centering.
    PIDController pid_corridor_;
    
    // Kinematics calculator
    algorithms::Kinematics kinematics_;

    // Configuration parameters.
    const float base_linear_velocity = 0.15f;      // Base forward speed in m/s
    const float max_angular_velocity = 1.5f;       // Maximum angular velocity in rad/s
    const float max_wheel_speed = 10.0f;           // Maximum wheel speed in rad/s for mapping
    const float front_distance_threshold = 0.5f;   // Distance threshold for slowing down (m)

    // Current motor speeds.
    uint8_t left_speed_ = 130;
    uint8_t right_speed_ = 130;
};