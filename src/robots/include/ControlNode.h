#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include "algorithms/kinematics.hpp"
#include "algorithms/turns.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"  // For marker IDs
#include "geometry_msgs/msg/point32.hpp"       // For marker corners

enum class RobotState {
    IDLE,
    FOLLOWING_CORRIDOR,
    ALIGN_TURN,
    TURN,
    POST_ALIGN_TURN,
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
          ALIGN_TO_CENTER_DISTANCE(0.25f) // 30 centimeters in meters
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

        aruco_subscriber_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/bpc_prp_robot/detected_markers/ids", 1,
            std::bind(&ControlNode::on_aruco_msg, this, std::placeholders::_1));

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
    // ArUco marker tracking
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr aruco_subscriber_;
    std::unordered_set<int> detected_markers_;  // Just store unique marker IDs
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr turn_subscriber_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr buttons_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr leds_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr integrated_yaw_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    TurnType current_turn_type_{TurnType::RIGHT};  // Default value
    float target_turn_angle_{0.0f};
    
    algorithms::Kinematics kinematics_;
    RobotState current_state_;
    bool end_of_corridor_detected_;
    int last_button_pressed_;
    
    // Rotation tracking
    float current_yaw_{0.0f};
    float start_yaw_{0.0f};

    // Turn PID Controller variables
    float turn_previous_error_{0.0f};
    float turn_integral_{0.0f};
    const float turn_kp_{0.2f};    // Tune these values
    const float turn_ki_{0.1f};
    const float turn_kd_{0.0f};
    

    // Position tracking
    double current_x_{0.0};
    double current_y_{0.0};
    double current_theta_{0.0};
    double align_start_x_{0.0};
    double align_start_y_{0.0};

    // LIDAR data storage
    float front_dist_{-1.0f};
    float right_dist_{-1.0f};
    float left_dist_{-1.0f};
    float single_right_{-1.0f};
    float single_left_{-1.0f};
    float last_right_dist_{-1.0f};
    float last_left_dist_{-1.0f};

    // PID Controller variables
    float previous_error_{0.0f};
    float integral_{0.0f};
    const float kp_{1.5f};
    const float ki_{0.0f};
    const float kd_{0.15f};

    // PID Controller variables for straight-line maintenance
    float straight_previous_error_{0.0f};
    float straight_integral_{0.0f};
    const float straight_kp_{0.05f};    // Even more conservative
    const float straight_ki_{0.0f};     // Remove integral term completely
    const float straight_kd_{0.05f};     // Keep damping
    const float max_straight_correction_{0.2f};  // Limit maximum correction

    // Constants
    const float corner_detection_threshold_{0.3f};
    const float speed_coefficient_{10.0f};
    const float base_linear_velocity_{0.04f};
    const float emergency_stop_threshold_{0.2f};

    const float ALIGN_TO_CENTER_DISTANCE = 0.25f;      // For initial alignment and crossroads
    const float POST_ALIGN_SHORT_DISTANCE = 0.15f;   // Shorter distance for post-turn alignment
    bool is_crossroad_{false};                       // Track if we're handling a crossroad

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
            case RobotState::POST_ALIGN_TURN:
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
        current_turn_type_ = static_cast<TurnType>(msg->data);
    }

    void on_pose_msg(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        current_x_ = msg->x;
        current_y_ = msg->y;
        current_theta_ = msg->theta;
    }

    void on_aruco_msg(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        for (const auto& marker_id : msg->data) {
            if (detected_markers_.insert(marker_id).second) {  // If insertion was successful (new marker)
                RCLCPP_INFO(node_->get_logger(), "New ArUco marker detected: %d", marker_id);
            }
        }
    }

    bool has_moved_required_distance() {
        double dx = current_x_ - align_start_x_;
        double dy = current_y_ - align_start_y_;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Use different distances based on state and situation
        float required_distance = ALIGN_TO_CENTER_DISTANCE;
        if (current_state_ == RobotState::POST_ALIGN_TURN && !is_crossroad_) {
            required_distance = POST_ALIGN_SHORT_DISTANCE;
        }
        
        return distance >= required_distance;
    }

    bool is_yaw_aligned() {
        float yaw_diff = current_yaw_ - start_yaw_;
        while (yaw_diff > M_PI) yaw_diff -= 2*M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2*M_PI;
        
        return std::abs(yaw_diff) < 0.1f;  // About 5.5 degrees tolerance
    }

    void on_button_msg(const std_msgs::msg::UInt8::SharedPtr msg) {
        last_button_pressed_ = static_cast<int>(msg->data);
        RCLCPP_INFO(node_->get_logger(), "Button pressed: %d", last_button_pressed_);
    }

    void on_lidar_msg(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 5) {
            front_dist_ = msg->data[0];
            right_dist_ = msg->data[1];
            left_dist_ = msg->data[2];
            single_right_ = msg->data[3];
            single_left_ = msg->data[4];

            // if (last_left_dist_ != -1) {
            //     float dist_diff_right = std::fabs(single_right_ - last_right_dist_);
            //     float dist_diff_left = std::fabs(single_left_ - last_left_dist_);
                
            //     if (dist_diff_left > corner_detection_threshold_ || 
            //         dist_diff_right > corner_detection_threshold_) {
            //         end_of_corridor_detected_ = true;
            //         // left_dist_ = last_left_dist_;
            //         // right_dist_ = last_right_dist_;
            //         RCLCPP_INFO(node_->get_logger(), "End of corridor detected!");
            //     }
            // }

            // If at least one of the walls is substituted, the robot is currently in a corner
            if (std::fabs(single_right_ - right_dist_) > corner_detection_threshold_ || std::fabs(single_left_ - left_dist_) > corner_detection_threshold_ && !end_of_corridor_detected_) {
                RCLCPP_INFO(node_->get_logger(), "END OF CORRIDOR DETECTED!");
                end_of_corridor_detected_ = true;
            }

            last_right_dist_ = single_right_;
            last_left_dist_ = single_left_;
        }
    }

    void on_yaw_msg(const std_msgs::msg::Float32::SharedPtr msg) {
        current_yaw_ = msg->data;
    }

    float calculate_pid_angular_velocity() {
        // Filter out small differences in raw measurements
        float left = left_dist_;
        float right = right_dist_;
        const float distance_deadband = 0.02f;  // 2cm deadband for raw distances
        
        if (std::abs(left - right) < distance_deadband) {
            // If difference is smaller than deadband, consider them equal
            left = (left + right) / 2.0f;
            right = left;
        }
        
        float corridor_offset = left - right;
        
        float error = corridor_offset;
        integral_ += error * 0.02f;
        float derivative = (error - previous_error_) / 0.02f;
        
        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        
        previous_error_ = error;
        return output;
    }

    float calculate_straight_pid_angular_velocity() {
        // Calculate yaw difference
        float yaw_diff = current_yaw_ - start_yaw_;
        // Normalize to [-π, π]
        while (yaw_diff > M_PI) yaw_diff -= 2*M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2*M_PI;
        
        // Apply deadband to ignore tiny errors
        if (std::abs(yaw_diff) < 0.015f) {  // Ignore errors less than ~0.86 degrees
            straight_previous_error_ = 0.0f;  // Reset derivative term when in deadband
            return 0.0f;
        }
        
        // Use yaw difference as error
        float error = yaw_diff;
        
        // Calculate derivative with low-pass filter
        float derivative = (error - straight_previous_error_) / 0.02f;
        // Simple low-pass filter on derivative term
        derivative = 0.7f * derivative;  // Dampen the derivative response
        
        float output = straight_kp_ * error + straight_kd_ * derivative;
        
        // Limit maximum correction
        output = std::min(std::max(-max_straight_correction_, output), max_straight_correction_);
        
        straight_previous_error_ = error;
        return -output;  // Negative because positive yaw diff needs negative angular velocity to correct
    }

    float calculate_turn_pid_angular_velocity(float target_angle) {
        float angle_error = target_angle - (current_yaw_ - start_yaw_);
        
        // Normalize error to [-π, π]
        while (angle_error > M_PI) angle_error -= 2*M_PI;
        while (angle_error < -M_PI) angle_error += 2*M_PI;
        
        // PID calculations
        turn_integral_ += angle_error * 0.02f;  // dt = 20ms
        float derivative = (angle_error - turn_previous_error_) / 0.02f;
        
        float output = turn_kp_ * angle_error + 
                      turn_ki_ * turn_integral_ + 
                      turn_kd_ * derivative;
        
        turn_previous_error_ = angle_error;
        return output;
    }

    uint8_t convert_speed_to_command(float wheel_speed) {
        int command = 127 + static_cast<int>(std::round(speed_coefficient_ * wheel_speed));
        command = std::min(255, std::max(0, command));
        return static_cast<uint8_t>(command);
    }

    // Add this as a private method in the ControlNode class
    void handle_turn_transition() {
        start_yaw_ = current_yaw_;  // Fresh yaw reference for turn
        turn_integral_ = 0.0f;      // Reset turn PID
        turn_previous_error_ = 0.0f;

        // TODO: figure out where to turn based on the found markers

        // Log and clear detected markers before the turn
        if (!detected_markers_.empty()) {
            RCLCPP_INFO(node_->get_logger(), "Markers detected before turn:");
            for (const auto& id : detected_markers_) {
                RCLCPP_INFO(node_->get_logger(), "Marker ID: %d", id);
            }
            detected_markers_.clear(); // clear the set of markers for the next stint
        }
        
        // Determine turn angle based on turn type
        switch (current_turn_type_) {
            case TurnType::LEFT:
                target_turn_angle_ = M_PI/2.0f;
                is_crossroad_ = false;
                current_state_ = RobotState::TURN;
                RCLCPP_INFO(node_->get_logger(), "Starting left turn");
                break;
            case TurnType::RIGHT:
                target_turn_angle_ = -M_PI/2.0f;
                is_crossroad_ = false;
                current_state_ = RobotState::TURN;
                RCLCPP_INFO(node_->get_logger(), "Starting right turn");
                break;
            case TurnType::CROSS:
                current_state_ = RobotState::POST_ALIGN_TURN;
                align_start_x_ = current_x_;
                align_start_y_ = current_y_;
                is_crossroad_ = true;
                RCLCPP_INFO(node_->get_logger(), "Detected crossing, transition to POST_ALIGN_TURN");
                break;
            case TurnType::LEFT_FRONT:
            case TurnType::RIGHT_FRONT:
                target_turn_angle_ = 0.0f;
                is_crossroad_ = true;
                current_state_ = RobotState::TURN;
                RCLCPP_INFO(node_->get_logger(), "Going straight through left-front or right-front");
                break;
            case TurnType::T_TURN:
                target_turn_angle_ = M_PI/2.0f;
                is_crossroad_ = false;
                current_state_ = RobotState::TURN;
                RCLCPP_INFO(node_->get_logger(), "Going left on a T_TURN");
                break;
            case TurnType::BLIND_TURN:
                target_turn_angle_ = M_PI;
                is_crossroad_ = false;
                current_state_ = RobotState::TURN;
                RCLCPP_INFO(node_->get_logger(), "Turning around in a blind turn");
                break;
        }
    }

    void state_machine_update() {
        std_msgs::msg::UInt8MultiArray motor_command;
        motor_command.data = {127, 127};  // Default to stopped

        update_led_color(current_state_);

        // Check if button 1 was pressed - reset to IDLE from any state
        if (last_button_pressed_ == 1) {
            current_state_ = RobotState::IDLE;
            end_of_corridor_detected_ = false;  // Reset corridor detection
            last_button_pressed_ = -1;  // Reset button press
            // Reset PID controllers
            integral_ = 0.0f;
            turn_integral_ = 0.0f;
            previous_error_ = 0.0f;
            turn_previous_error_ = 0.0f;
            RCLCPP_INFO(node_->get_logger(), "Button 1 pressed: Resetting to IDLE state");
            motors_publisher_->publish(motor_command);  // Stop motors
            return;  // Exit early after reset
        }

        if (front_dist_ == -1 || right_dist_ == -1 || left_dist_ == -1) {
            RCLCPP_INFO(node_->get_logger(), "Waiting for lidar data");
            return;
        }

        // // Check for emergency stop condition in any state except EMERGENCY_STOP
        // if (current_state_ != RobotState::EMERGENCY_STOP && current_state_ != RobotState::TURN && front_dist_ < emergency_stop_threshold_) {
        //     RobotState previous_state = current_state_;
        //     current_state_ = RobotState::EMERGENCY_STOP;
        //     RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP: Obstacle too close (%.2f m)", front_dist_);
        // }

        switch (current_state_) {
            case RobotState::IDLE: {
                if (last_button_pressed_ == 0) {
                    current_state_ = RobotState::FOLLOWING_CORRIDOR;
                    last_button_pressed_ = -1;
                    RCLCPP_INFO(node_->get_logger(), "Transitioning from IDLE to FOLLOWING_CORRIDOR state");
                }
                // else if (last_button_pressed_ == 1) {
                //     current_state_ = RobotState::ALIGN_TURN;
                //     align_start_x_ = current_x_;
                //     align_start_y_ = current_y_;
                //     last_button_pressed_ = -1;
                //     RCLCPP_INFO(node_->get_logger(), "Transitioning from IDLE to ALIGN_TURN state");
                // } else if (last_button_pressed_ == 2) {
                //     current_state_ = RobotState::TURN;
                //     start_yaw_ = current_yaw_;
                //     last_button_pressed_ = -1;
                //     RCLCPP_INFO(node_->get_logger(), "Transitioning from IDLE to TURN state");
                // }
                break;
            }
            case RobotState::FOLLOWING_CORRIDOR: {
                if (end_of_corridor_detected_) {
                    // First stop the robot briefly to ensure stable yaw reading
                    // motor_command.data = {127, 127};
                    // motors_publisher_->publish(motor_command);
                    
                    // Store current position and yaw
                    align_start_x_ = current_x_;
                    align_start_y_ = current_y_;
                    start_yaw_ = current_yaw_;
                    
                    // Reset all PID variables for straight line control
                    straight_integral_ = 0.0f;
                    straight_previous_error_ = 0.0f;
                    
                    // Reset flags and change state
                    end_of_corridor_detected_ = false;
                    current_state_ = RobotState::ALIGN_TURN;
                    
                    RCLCPP_INFO(node_->get_logger(), "Transitioning to ALIGN_TURN state. Initial yaw: %.3f", start_yaw_);
                } else if (front_dist_ < emergency_stop_threshold_) {
                    handle_turn_transition();
                } else {
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
                    handle_turn_transition();
                    RCLCPP_INFO(node_->get_logger(), "Alignment complete, transitioning to TURN");
                } else if (front_dist_ < emergency_stop_threshold_) {
                    handle_turn_transition();
                } else {
                    // Use corridor wall-following PID instead of straight line PID
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
            case RobotState::TURN: {
                float angle_turned = current_yaw_ - start_yaw_;
                // Normalize angle to [-π, π]
                while (angle_turned > M_PI) angle_turned -= 2*M_PI;
                while (angle_turned < -M_PI) angle_turned += 2*M_PI;
                
                float angle_tolerance = 0.07f;
                
                // Target reached within tolerance
                if (std::abs(target_turn_angle_ - angle_turned) < angle_tolerance) {
                    current_state_ = RobotState::POST_ALIGN_TURN;
                    align_start_x_ = current_x_;
                    align_start_y_ = current_y_;
                    start_yaw_ = current_yaw_;  // Fresh yaw reference for post-align
                    straight_integral_ = 0.0f;
                    straight_previous_error_ = 0.0f;
                    RCLCPP_INFO(node_->get_logger(), "Turn complete, transitioning to POST_ALIGN_TURN");
                } else {
                    // Calculate angular velocity using PID
                    float angular_velocity = calculate_turn_pid_angular_velocity(target_turn_angle_);
                    
                    // If going straight, add forward velocity
                    float linear_velocity = (target_turn_angle_ == 0.0f) ? base_linear_velocity_ : 0.0f;
                    
                    // Create robot speed command
                    algorithms::RobotSpeed robot_speed(linear_velocity, angular_velocity);
                    algorithms::WheelSpeed wheel_speeds = kinematics_.inverse(robot_speed);
                    
                    motor_command.data = {
                        convert_speed_to_command(wheel_speeds.l),
                        convert_speed_to_command(wheel_speeds.r)
                    };
                }
                break;
            }
            case RobotState::POST_ALIGN_TURN: {
                if (has_moved_required_distance()) {
                    end_of_corridor_detected_ = false;
                    current_state_ = RobotState::FOLLOWING_CORRIDOR;
                    is_crossroad_ = false;  // Reset crossroad flag
                    // Reset corridor PID for new corridor
                    integral_ = 0.0f;
                    previous_error_ = 0.0f;
                    RCLCPP_INFO(node_->get_logger(), "Alignment complete, transitioning to FOLLOWING_CORRIDOR");
                } else if (front_dist_ < emergency_stop_threshold_) {
                    handle_turn_transition();
                } else {
                    // Use corridor wall-following PID instead of straight line PID
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