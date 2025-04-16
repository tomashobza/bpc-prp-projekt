#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <algorithm>
#include "algorithms/planar_imu_integrator.hpp"

enum class ImuNodeMode {
    CALIBRATE,
    INTEGRATE,
};

class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("imu_node"), 
                planar_integrator_(),
                calibration_duration_(5.0)  // 5 seconds calibration
    {
        // Subscribe to IMU data
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bpc_prp_robot/imu", 10, 
            std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1)
        );

        // Publisher for integrated yaw
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/integrated_yaw", 1
        );

        // Set up publisher for motor commands
        motors_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "/bpc_prp_robot/set_motor_speeds", 1
        );

        // Set up timer for motor control (20ms = 50Hz)
        motor_control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ImuNode::control_loop, this)
        );

        // Start in calibration mode
        mode = ImuNodeMode::CALIBRATE;
        calibration_start_time_ = this->get_clock()->now();
        last_msg_time_ = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "IMU Node initialized. Starting calibration...");
    }

    ~ImuNode() override = default;

private:
    void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
        if (mode == ImuNodeMode::CALIBRATE) {
            handle_calibration(msg);
        } else {
            handle_integration(msg);
        }
    }

    void handle_calibration(const sensor_msgs::msg::Imu::SharedPtr msg) {
        gyro_calibration_samples_.push_back(msg->angular_velocity.z);
        
        // Check if calibration time is complete
        auto current_time = this->get_clock()->now();
        if ((current_time - calibration_start_time_).seconds() >= calibration_duration_) {
            // Set calibration and switch to integration mode
            planar_integrator_.setCalibration(gyro_calibration_samples_);
            reference_yaw_ = planar_integrator_.getYaw();
            mode = ImuNodeMode::INTEGRATE;
            
            RCLCPP_INFO(this->get_logger(), 
                "Calibration complete. Collected %zu samples.",
                gyro_calibration_samples_.size());
        }
    }

    void handle_integration(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Use message timestamps for dt calculation
        rclcpp::Time current_msg_time(msg->header.stamp);
        double dt = (current_msg_time - last_msg_time_).seconds();
        last_msg_time_ = current_msg_time;
    
        // Update yaw estimation
        planar_integrator_.update(msg->angular_velocity.z, dt);
    
        // Publish integrated yaw
        auto yaw_msg = std_msgs::msg::Float32();
        yaw_msg.data = planar_integrator_.getYaw();
        yaw_publisher_->publish(yaw_msg);
    }

    void control_loop() {
        if (mode != ImuNodeMode::INTEGRATE) {
            return;  // Don't run control loop during calibration
        }

        // Calculate yaw error and correction
        float current_yaw = planar_integrator_.getYaw();
        float yaw_error = reference_yaw_ - current_yaw;
        
        // Calculate correction, keeping it as float until the last moment
        float correction = Kp_ * yaw_error;
        
        // Convert to motor commands, clamping values properly
        int base_speed = 127;
        int left_speed = base_speed - static_cast<int>(correction);
        int right_speed = base_speed + static_cast<int>(correction);
        
        // Clamp values to valid range before converting to uint8_t
        left_speed = std::clamp(left_speed, 0, 255);
        right_speed = std::clamp(right_speed, 0, 255);
    
        // Now safely convert to uint8_t
        uint8_t left_motor_command = static_cast<uint8_t>(left_speed);
        uint8_t right_motor_command = static_cast<uint8_t>(right_speed);
    
        // Publish motor command
        auto msg_commands = std_msgs::msg::UInt8MultiArray();
        msg_commands.data = {left_motor_command, right_motor_command};
        motors_publisher_->publish(msg_commands);
    
        RCLCPP_INFO(this->get_logger(), 
            "Yaw error: %f, Correction: %f, Motors L/R: %d/%d", 
            yaw_error, correction, left_speed, right_speed);
    }

    ImuNodeMode mode;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motors_publisher_;
    rclcpp::TimerBase::SharedPtr motor_control_timer_;
    algorithms::PlanarImuIntegrator planar_integrator_;
    std::vector<float> gyro_calibration_samples_;
    rclcpp::Time last_msg_time_;
    rclcpp::Time calibration_start_time_;
    double calibration_duration_;
    float reference_yaw_;
    float Kp_ = 50.0;
};