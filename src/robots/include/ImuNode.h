#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include "algorithms/planar_imu_integrator.hpp"

enum class ImuNodeMode {
    CALIBRATE,
    INTEGRATE,
};

class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("imu_node"), 
                planar_integrator_(),
                calibration_duration_(5.0)  // 3 seconds calibration
    {
        // Subscribe to IMU data
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bpc_prp_robot/imu", 10, 
            std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1)
        );

        // Publisher for integrated yaw
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float32>(
            "/integrated_yaw", 10
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

    ImuNodeMode mode;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_publisher_;
    algorithms::PlanarImuIntegrator planar_integrator_;
    std::vector<float> gyro_calibration_samples_;
    rclcpp::Time last_msg_time_;
    rclcpp::Time calibration_start_time_;
    double calibration_duration_;
};
