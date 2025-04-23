#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include "algorithms/kinematics.hpp"  // Include the kinematics logic

enum class RobotState {
    FOLLOWING_CORRIDOR,
    ALIGN_TURN
};

class ControlNode {
public:
    ControlNode(const rclcpp::Node::SharedPtr &node)
        : node_(node),
          kinematics_(0.033, 0.16, 360),  // Same parameters as MotorNode
          current_state_(RobotState::FOLLOWING_CORRIDOR),
          end_of_corridor_detected_(false)
    {
        RCLCPP_INFO(node_->get_logger(), "Control node started!");

        // Subscribe to averaged LIDAR data
        lidar_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/bpc_prp_robot/lidar_avg", 1,
            std::bind(&ControlNode::on_lidar_msg, this, std::placeholders::_1));

        // Publisher for motor commands
        motors_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds", 1);

        // Create timer for state machine updates (50 Hz)
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControlNode::state_machine_update, this));
    }

private:
    // ROS node handle, publishers, subscribers, and timer
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Kinematics object for inverse kinematics calculations
    algorithms::Kinematics kinematics_;

    // State machine variables
    RobotState current_state_;
    bool end_of_corridor_detected_;

    // LIDAR data storage
    float front_dist_{0.0f};
    float right_dist_{0.0f};
    float left_dist_{0.0f};
    float last_right_dist_{-1.0f};
    float last_left_dist_{-1.0f};

    // PID Controller variables
    float previous_error_{0.0f};
    float integral_{0.0f};
    const float kp_{1.0f};    // Proportional gain
    const float ki_{0.05f};   // Integral gain
    const float kd_{0.1f};    // Derivative gain

    // Constants
    const float corner_detection_threshold_{0.2f};  // Same as in MotorNode
    const float speed_coefficient_{10.0f};          // Same as in MotorNode
    const float base_linear_velocity_{0.02f};       // Same as in MotorNode

    void on_lidar_msg(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            front_dist_ = msg->data[0];
            right_dist_ = msg->data[1];
            left_dist_ = msg->data[2];

            // Check for end of corridor
            if (last_left_dist_ != -1) {
                float dist_diff_left = std::fabs(left_dist_ - last_left_dist_);
                float dist_diff_right = std::fabs(right_dist_ - last_right_dist_);
                
                if (dist_diff_left > corner_detection_threshold_ || 
                    dist_diff_right > corner_detection_threshold_) {
                    end_of_corridor_detected_ = true;
                    RCLCPP_INFO(node_->get_logger(), "End of corridor detected!");
                }
            }

            last_left_dist_ = left_dist_;
            last_right_dist_ = right_dist_;
        }
    }

    float calculate_pid_angular_velocity() {
        float corridor_offset = left_dist_ - right_dist_;  // Positive if closer to left wall
        
        // PID calculations
        float error = corridor_offset;
        integral_ += error * 0.02f;  // dt = 20ms = 0.02s
        float derivative = (error - previous_error_) / 0.02f;
        
        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        
        previous_error_ = error;
        return output;
    }

    uint8_t convert_speed_to_command(float wheel_speed) {
        int command = 127 + static_cast<int>(std::round(speed_coefficient_ * wheel_speed));
        command = std::min(255, std::max(0, command));
        return static_cast<uint8_t>(command);
    }

    void state_machine_update() {
        std_msgs::msg::UInt8MultiArray motor_command;
        motor_command.data = {127, 127};  // Default to stopped

        switch (current_state_) {
            case RobotState::FOLLOWING_CORRIDOR: {
                if (end_of_corridor_detected_) {
                    // Transition to ALIGN_TURN state
                    current_state_ = RobotState::ALIGN_TURN;
                    end_of_corridor_detected_ = false;  // Reset the flag
                    RCLCPP_INFO(node_->get_logger(), "Transitioning to ALIGN_TURN state");
                } else if (front_dist_ > 0.2f) {  // Only move if no front obstacle
                    // Calculate angular velocity using PID
                    float angular_velocity = calculate_pid_angular_velocity();
                    
                    // Set linear velocity
                    float linear_velocity = base_linear_velocity_;
                    
                    // Calculate robot speed
                    algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);
                    
                    // Use inverse kinematics to compute wheel speeds
                    algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);
                    
                    // Convert wheel speeds to motor commands
                    motor_command.data = {
                        convert_speed_to_command(wheel_speeds.l),
                        convert_speed_to_command(wheel_speeds.r)
                    };
                }
                break;
            }
            case RobotState::ALIGN_TURN: {
                // Currently does nothing - will be implemented later
                // Motors already set to 127, 127 by default
                break;
            }
        }

        // Publish motor commands
        motors_publisher_->publish(motor_command);
    }
};