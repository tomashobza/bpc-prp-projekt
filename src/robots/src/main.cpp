#include <rclcpp/rclcpp.hpp>
// #include "RosExampleClass.h"
#include "MotorNode.h"
#include "EncoderNode.h"
#include "LineNode.h"
#include "LidarNode.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    auto node1 = std::make_shared<rclcpp::Node>("lidar_node");
    //auto node2 = std::make_shared<rclcpp::Node>("line_node");

    // Create instances of RosExampleClass using the existing nodes
    auto lidar_node = std::make_shared<LidarNode>(node1);
    //auto line_node = std::make_shared<LineNode>(node2);

    // Add nodes to the executor
    executor->add_node(node1);
    //executor->add_node(node2);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
