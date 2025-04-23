#include <rclcpp/rclcpp.hpp>
#include "EncoderNode.h"
#include "MotorNode.h"
#include "EncoderNode.h"
#include "LineNode.h"
#include "LidarNode.h"
#include "ControlNode.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    auto _encoder_node = std::make_shared<rclcpp::Node>("encoder_node");
    auto _motor_node = std::make_shared<rclcpp::Node>("motor_node");
    auto _line_node = std::make_shared<rclcpp::Node>("line_node");
    auto _lidar_node = std::make_shared<rclcpp::Node>("lidar_node");
    auto _control_node = std::make_shared<rclcpp::Node>("control_node");

    // Create instances of RosExampleClass using the existing nodes
    auto encoder_node = std::make_shared<EncoderNode>(_encoder_node);
    auto motor_node = std::make_shared<MotorNode>(_motor_node);
    auto line_node = std::make_shared<LineNode>(_line_node);
    auto lidar_node = std::make_shared<LidarNode>(_lidar_node);
    auto control_node = std::make_shared<ControlNode>(_control_node);

    // Add nodes to the executor
    
    executor->add_node(_encoder_node);
    // executor->add_node(_motor_node);
    // executor->add_node(_line_node);
    executor->add_node(_lidar_node);
    executor->add_node(_control_node);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
