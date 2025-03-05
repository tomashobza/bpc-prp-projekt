#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"  // Note: updated include path to match previous implementation

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create an instance of IoNode
    auto io_node = std::make_shared<nodes::IoNode>();

    // Add node to the executor
    executor->add_node(io_node);

    // Run the executor (handles callbacks for the node)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}