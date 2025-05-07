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

class CameraNode : public rclcpp::Node {
public:
    // Represents one detected marker
    struct Aruco {
        int id;
        std::vector<cv::Point2f> corners;
    };

    CameraNode() : Node("camera_node") {
        // Initialize ArUco dictionary
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

        // Subscribe to compressed image topic
        compressed_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            "/bpc_prp_robot/camera/compressed", 10,
            std::bind(&CameraNode::compressed_image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Camera node initialized");
    }

    // Process the latest frame and detect markers
    void process_frame() {
        if (last_frame_.empty()) {
            RCLCPP_WARN(get_logger(), "No frame available for processing");
            return;
        }

        // Detect ArUco markers
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        
        cv::aruco::detectMarkers(last_frame_, dictionary_, marker_corners, marker_ids);
        
        // Store detection results
        last_detections_.clear();
        if (!marker_ids.empty()) {
            std::cout << "ArUcos found: ";
            for (size_t i = 0; i < marker_ids.size(); i++) {
                std::cout << marker_ids[i] << " ";
                
                Aruco aruco;
                aruco.id = marker_ids[i];
                aruco.corners = marker_corners[i];
                last_detections_.push_back(aruco);
            }
            std::cout << std::endl;
            
            // Draw markers on a copy of the image for visualization (if needed)
            cv::Mat annotated_frame = last_frame_.clone();
            cv::aruco::drawDetectedMarkers(annotated_frame, marker_corners, marker_ids);
            
            // Display the annotated image (optional)
            cv::imshow("ArUco Detections", annotated_frame);
            cv::waitKey(1);  // Allow OpenCV to process window events
        }
    }

private:
    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            // Decode compressed image
            std::vector<uchar> data(msg->data.begin(), msg->data.end());
            last_frame_ = cv::imdecode(data, cv::IMREAD_COLOR);
            
            // Check if the image is valid
            if (last_frame_.empty()) {
                RCLCPP_ERROR(get_logger(), "Failed to decode compressed image");
                return;
            }

            // Process the frame immediately
            process_frame();

        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(get_logger(), "CV exception: %s", e.what());
        }
    }

    // ArUco dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

    // Subscription to compressed image
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;

    // Store the latest frame and detections
    cv::Mat last_frame_;
    std::vector<Aruco> last_detections_;
    
public:
    // Getters to access the stored data
    cv::Mat get_last_frame() const { return last_frame_; }
    std::vector<Aruco> get_last_detections() const { return last_detections_; }
};

#endif // CAMERA_NODE_H