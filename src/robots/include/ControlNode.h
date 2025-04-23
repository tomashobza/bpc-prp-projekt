#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "algorithms/kinematics.hpp"
#include "algorithms/turns.hpp"

enum class RobotState {
    IDLE,
    FOLLOWING_CORRIDOR,
    ALIGN_TURN,
    TURN,
    EMERGENCY_STOP
};

class ControlNode {
public:
    ControlNode(const rclcpp::Node::SharedPtr &node)
        : node_(node),
          kinematics_(0.033, 0.16, 360),
          current_state_(RobotState::IDLE),
          end_of_corridor_detected_(false),
          last_button_pressed_(-1),
          ALIGN_TO_CENTER_DISTANCE(0.3f) // 30 centimeters in meters
    {
        RCLCPP_INFO(node_->get_logger(), "Control node started!");

        lidar_subscriber_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/bpc_prp_robot/lidar_avg", 1,
            std::bind(&ControlNode::on_lidar_msg, this, std::placeholders::_1));
            
        turn_subscriber_ = node_->create_subscription<std_msgs::msg::Int8>(
            "/bpc_prp_robot/detected_turn", 1,
            std::bind(&ControlNode::on_turn_msg, this, std::placeholders::_1));

        buttons_subscriber_ = node_->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/buttons", 1,
            std::bind(&ControlNode::on_button_msg, this, std::placeholders::_1));

        pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::Pose2D>(
            "/robot_pose", 1,
            std::bind(&ControlNode::on_pose_msg, this, std::placeholders::_1));

        motors_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds", 1);
        
        integrated_yaw_subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
            "/integrated_yaw", 1,
            std::bind(&ControlNode::on_yaw_msg, this, std::placeholders::_1));

        // New publisher for RGB LEDs
        leds_publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/rgb_leds", 1);

        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ControlNode::state_machine_update, this));
            
        RCLCPP_INFO(node_->get_logger(), "Starting in IDLE state. Press button 0 to start corridor following.");
        
        // Set initial LED color
        update_led_color(current_state_);
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr turn_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr buttons_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr leds_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr integrated_yaw_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    algorithms::Kinematics kinematics_;
    RobotState current_state_;
    bool end_of_corridor_detected_;
    int last_button_pressed_;
    
    // Rotation tracking
    float current_yaw_{0.0f};
    float start_yaw_{0.0f};

    // Position tracking
    double current_x_{0.0};
    double current_y_{0.0};
    double current_theta_{0.0};
    double align_start_x_{0.0};
    double align_start_y_{0.0};
    const float ALIGN_TO_CENTER_DISTANCE;

    // LIDAR data storage
    float front_dist_{-1.0f};
    float right_dist_{-1.0f};
    float left_dist_{-1.0f};
    float last_right_dist_{-1.0f};
    float last_left_dist_{-1.0f};

    // PID Controller variables
    float previous_error_{0.0f};
    float integral_{0.0f};
    const float kp_{1.0f};
    const float ki_{0.05f};
    const float kd_{0.1f};

    // Constants
    const float corner_detection_threshold_{0.2f};
    const float speed_coefficient_{10.0f};
    const float base_linear_velocity_{0.02f};
    const float emergency_stop_threshold_{0.15f};

    void update_led_color(RobotState state) {
        std_msgs::msg::UInt8MultiArray led_msg;
        led_msg.data.resize(3);  // RGB values

        switch (state) {
            case RobotState::IDLE:
                // Blue
                led_msg.data = {0, 0, 255};
                break;
            case RobotState::FOLLOWING_CORRIDOR:
                // Green
                led_msg.data = {0, 255, 0};
                break;
            case RobotState::ALIGN_TURN:
                // Yellow
                led_msg.data = {128, 0, 128};
                break;
            case RobotState::TURN:
                // Yellow
                led_msg.data = {255, 255, 255};
                break;
            case RobotState::EMERGENCY_STOP:
                // Red
                led_msg.data = {255, 0, 0};
                break;
        }

        leds_publisher_->publish(led_msg);
    }

    void on_turn_msg(const std_msgs::msg::Int8::SharedPtr msg) {
        // RCLCPP_INFO(node_->get_logger(), "TURN: %d", msg->data);
    }

    void on_pose_msg(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_theta_ = msg->theta;
    }

    bool has_moved_required_distance() {
        double dx = current_x_ - align_start_x_;
        double dy = current_y_ - align_start_y_;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        RCLCPP_INFO(node_->get_logger(), "Distance moved: %.3f m", distance);
        return distance >= ALIGN_TO_CENTER_DISTANCE;
    }

    void on_button_msg(const std_msgs::msg::UInt8::SharedPtr msg) {
        last_button_pressed_ = static_cast<int>(msg->data);
        RCLCPP_INFO(node_->get_logger(), "Button pressed: %d", last_button_pressed_);
    }

    void on_lidar_msg(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            front_dist_ = msg->data[0];
            right_dist_ = msg->data[1];
            left_dist_ = msg->data[2];

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

    void on_yaw_msg(const std_msgs::msg::Float32::SharedPtr msg) {
        current_yaw_ = msg->data;
    }

    float calculate_pid_angular_velocity() {
        float corridor_offset = left_dist_ - right_dist_;
        
        float error = corridor_offset;
        integral_ += error * 0.02f;
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

        update_led_color(current_state_);

        if (front_dist_ == -1 || right_dist_ == -1 || left_dist_ == -1) {
            RCLCPP_INFO(node_->get_logger(), "Waiting for lidar data");
            return;
        }

        // Check for emergency stop condition in any state except EMERGENCY_STOP
        if (current_state_ != RobotState::EMERGENCY_STOP && current_state_ != RobotState::TURN && front_dist_ < emergency_stop_threshold_) {
            RobotState previous_state = current_state_;
            current_state_ = RobotState::EMERGENCY_STOP;
            RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP: Obstacle too close (%.2f m)", front_dist_);
        }

        switch (current_state_) {
            case RobotState::IDLE: {
                if (last_button_pressed_ == 0) {
                    current_state_ = RobotState::FOLLOWING_CORRIDOR;
                    last_button_pressed_ = -1;
                    RCLCPP_INFO(node_->get_logger(), "Transitioning from IDLE to FOLLOWING_CORRIDOR state");
                }
                else if (last_button_pressed_ == 1) {
                    current_state_ = RobotState::ALIGN_TURN;
                    align_start_x_ = current_x_;
                    align_start_y_ = current_y_;
                    last_button_pressed_ = -1;
                    RCLCPP_INFO(node_->get_logger(), "Transitioning from IDLE to ALIGN_TURN state");
                } else if (last_button_pressed_ == 2) {
                    current_state_ = RobotState::TURN;
                    start_yaw_ = current_yaw_;
                    last_button_pressed_ = -1;
                    RCLCPP_INFO(node_->get_logger(), "Transitioning from IDLE to TURN state");
                }
                break;
            }
            case RobotState::FOLLOWING_CORRIDOR: {
                if (end_of_corridor_detected_) {
                    current_state_ = RobotState::ALIGN_TURN;
                    align_start_x_ = current_x_;
                    align_start_y_ = current_y_;
                    end_of_corridor_detected_ = false;
                    RCLCPP_INFO(node_->get_logger(), "Transitioning to ALIGN_TURN state");
                } else if (front_dist_ > emergency_stop_threshold_) {
                    float angular_velocity = calculate_pid_angular_velocity();
                    float linear_velocity = base_linear_velocity_;
                    
                    algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);
                    algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);
                    
                    motor_command.data = {
                        convert_speed_to_command(wheel_speeds.l),
                        convert_speed_to_command(wheel_speeds.r)
                    };
                }
                break;
            }
            case RobotState::ALIGN_TURN: {
                if (has_moved_required_distance()) {
                    // TODO: read the type of turn and proceed to turn that way
                    current_state_ = RobotState::TURN;
                    start_yaw_ = current_yaw_;
                    RCLCPP_INFO(node_->get_logger(), "Alignment complete, transitioning to TURN");
                } else if (front_dist_ > emergency_stop_threshold_) {
                    algorithms::RobotSpeed robot_speed(base_linear_velocity_, 0.0);
                    algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);
                    
                    motor_command.data = {
                        convert_speed_to_command(wheel_speeds.l),
                        convert_speed_to_command(wheel_speeds.r)
                    };
                }
                break;
            }
            case RobotState::TURN: {
                float angle_turned = current_yaw_ - start_yaw_;
                // Normalize angle to [-π, π]
                while (angle_turned > M_PI) angle_turned -= 2*M_PI;
                while (angle_turned < -M_PI) angle_turned += 2*M_PI;
                
                // Target is 90 degrees = π/2 radians
                if (std::abs(angle_turned) >= M_PI/2.0f) {
                    current_state_ = RobotState::IDLE;
                    RCLCPP_INFO(node_->get_logger(), "Turn complete (%.2f rad), transitioning to IDLE", angle_turned);
                } else {
                    // Turn with constant angular velocity (positive = counter-clockwise)
                    float angular_velocity = 0.5f;  // You might need to tune this value
                    algorithms::RobotSpeed robot_speed(0.0f, angular_velocity);
                    algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);
                    
                    motor_command.data = {
                        convert_speed_to_command(wheel_speeds.l),
                        convert_speed_to_command(wheel_speeds.r)
                    };
                    
                    RCLCPP_INFO(node_->get_logger(), "Current angle: %.2f rad", angle_turned);
                }
                break;
            }
            case RobotState::EMERGENCY_STOP: {
                // Stay in emergency stop until front distance is safe AND a button is pressed
                if (front_dist_ > emergency_stop_threshold_ && last_button_pressed_ >= 0) {
                    current_state_ = RobotState::IDLE;
                    last_button_pressed_ = -1;
                    RCLCPP_INFO(node_->get_logger(), "Emergency condition cleared, transitioning to IDLE");
                }
                break;
            }
        }

        // Publish motor commands
        motors_publisher_->publish(motor_command);
    }
};