#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, Float32, UInt32MultiArray


class MotorNode(Node):
    def __init__(self):
        super().__init__("motor_node")

        publish_topic = "/bpc_prp_robot/set_motor_speeds"
        subscribe_topic = "/bpc_prp_robot/encoders"
        timer_period = 0.1  # seconds

        self.publisher_ = self.create_publisher(UInt8MultiArray, publish_topic, 10)
        self.subscription = self.create_subscription(
            UInt32MultiArray, subscribe_topic, self.listener_callback, 10
        )
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now()

        self.get_logger().info("A jedeeeem!")

    def timer_callback(self):
        self.get_logger().info("Timer triggered. Publishing uptime...")
        current_time = self.get_clock().now()
        uptime = (current_time - self.start_time).nanoseconds / 1e9
        self.publish_message(uptime)

    def listener_callback(self, msg):
        self.get_logger().info("Received: %d" % msg.data[0])

    def publish_message(self, value_to_publish):
        msg = UInt8MultiArray()
        msg.data = [100, 100]  # Example fixed data
        self.publisher_.publish(msg)
        self.get_logger().info("Published: %s" % str(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
