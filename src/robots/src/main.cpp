#include <rclcpp/rclcpp.hpp>
// #include "RosExampleClass.h"
#include "MotorNode.h"
#include "EncoderNode.h"
#include "LineNode.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    auto node1 = std::make_shared<rclcpp::Node>("line_node");
    // auto node2 = std::make_shared<rclcpp::Node>("node2");

    // Create instances of RosExampleClass using the existing nodes
    // auto example_class1 = std::make_shared<RosExampleClass>(node1, "topic1", 1.0);
    // auto example_class2 = std::make_shared<RosExampleClass>(node2, "topic2", 2.0);
    // auto example_class2 = std::make_shared<MotorNode>(node1);
    // auto example_class2 = std::make_shared<EncoderNode>(node1);
    auto example_class2 = std::make_shared<LineNode>(node1);

    // Add nodes to the executor
    executor->add_node(node1);
    // executor->add_node(node2);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
