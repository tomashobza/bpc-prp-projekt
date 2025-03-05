#include "nodes/io_node.hpp"

namespace nodes {  // Note: corrected from 'namespace {' to proper namespace syntax
    
    IoNode::IoNode() 
        : Node("io_node")  // Initialize the Node with a name
    {
        // Create subscriber for button press messages
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            "button_topic",  // Topic name
            10,             // Queue size
            std::bind(&IoNode::on_button_callback, this, std::placeholders::_1)
        );
        
        // Optional: Add logging to confirm node initialization
        RCLCPP_INFO_ONCE(this->get_logger(), "IO Node has been initialized");
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        // Store the received button value
        button_pressed_ = static_cast<int>(msg->data);
        
        // Optional: Add logging to track received messages
        RCLCPP_INFO_ONCE(this->get_logger(), "Received button press: %d", button_pressed_);
    }
}  // namespace nodes