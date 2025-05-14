#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H

#include <memory>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "std_msgs/msg/int32_multi_array.hpp" // For marker IDs
#include "geometry_msgs/msg/point32.hpp"      // For marker corners

class CameraNode : public rclcpp::Node
{
public:
    // Constants
    static constexpr bool PUBLISH_ANNOTATED_IMAGES = false; // Toggle for publishing marked images

    // Represents one detected marker
    struct Aruco
    {
        int id;
        std::vector<cv::Point2f> corners;
    };

    CameraNode() : Node("camera_node")
    {
        // Initialize ArUco dictionary
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // Subscribe to compressed image topic
        compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/bpc_prp_robot/camera/compressed", 1,
            std::bind(&CameraNode::compressed_image_callback, this, std::placeholders::_1));

        // Publishers for ArUco markers
        marker_ids_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
            "/bpc_prp_robot/detected_markers/ids", 1);

        // Publisher for annotated images (if enabled)
        if (PUBLISH_ANNOTATED_IMAGES)
        {
            annotated_image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
                "/bpc_prp_robot/camera/annotated/compressed", 1);
        }

        RCLCPP_INFO(get_logger(), "Camera node initialized");
    }

    // Process the latest frame and detect markers
    void process_frame()
    {
        if (last_frame_.empty())
        {
            RCLCPP_WARN(get_logger(), "No frame available for processing");
            return;
        }

        // Detect ArUco markers
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(last_frame_, dictionary_, marker_corners, marker_ids);

        // Store detection results and publish
        last_detections_.clear();

        // Create annotated frame if publishing is enabled
        cv::Mat annotated_frame;
        if (PUBLISH_ANNOTATED_IMAGES)
        {
            annotated_frame = last_frame_.clone();
        }

        if (!marker_ids.empty())
        {
            // Prepare message for marker IDs
            auto ids_msg = std::make_unique<std_msgs::msg::Int32MultiArray>();
            ids_msg->data.clear();

            for (size_t i = 0; i < marker_ids.size(); i++)
            {
                ids_msg->data.push_back(marker_ids[i]);

                Aruco aruco;
                aruco.id = marker_ids[i];
                aruco.corners = marker_corners[i];
                last_detections_.push_back(aruco);
            }

            // Publish marker IDs
            marker_ids_pub_->publish(std::move(ids_msg));

            // Draw markers if publishing is enabled
            if (PUBLISH_ANNOTATED_IMAGES)
            {
                cv::aruco::drawDetectedMarkers(annotated_frame, marker_corners, marker_ids);
            }
        }

        // Publish annotated image if enabled, regardless of marker detection
        if (PUBLISH_ANNOTATED_IMAGES)
        {
            // Convert to compressed image message
            std::vector<uchar> buffer;
            cv::imencode(".jpg", annotated_frame, buffer);

            auto img_msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            img_msg->format = "jpeg";
            img_msg->data = buffer;

            annotated_image_pub_->publish(std::move(img_msg));
        }
    }

private:
    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            // Decode compressed image
            std::vector<uchar> data(msg->data.begin(), msg->data.end());
            last_frame_ = cv::imdecode(data, cv::IMREAD_COLOR);

            // Check if the image is valid
            if (last_frame_.empty())
            {
                RCLCPP_ERROR(get_logger(), "Failed to decode compressed image");
                return;
            }

            // Process the frame immediately
            process_frame();
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "CV exception: %s", e.what());
        }
    }

    // ArUco dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    // Subscription to compressed image
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr marker_ids_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr annotated_image_pub_;

    // Store the latest frame and detections
    cv::Mat last_frame_;
    std::vector<Aruco> last_detections_;

public:
    // Getters to access the stored data
    cv::Mat get_last_frame() const { return last_frame_; }
    std::vector<Aruco> get_last_detections() const { return last_detections_; }
};

#endif // CAMERA_NODE_H