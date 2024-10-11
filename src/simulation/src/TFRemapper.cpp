/**
 * @file TFRemapper.cpp
 * @brief This file contains the implementation of the TFRemapper class.
 * 
 * See TFRemapper.h for more details.
 */

#include "TFRemapper.h"

TFRemapper::TFRemapper(const std::string node_name, const std::string ns)
                : Node(node_name, ns) {

    // Declare parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Frame of the camera";
    this->declare_parameter("target_frame", "target_frame", param_desc);
    param_desc.description = "Frame of the locobot tag";
    this->declare_parameter("source_frame", "source_frame", param_desc);
    param_desc.description = "Debug mode. Print errors in case of failure";
    this->declare_parameter("debug", false, param_desc);
    param_desc.description = "The frequency to look at and publish the TFs";
    this->declare_parameter("publish_frequency", 1.0, param_desc);
    param_desc.description = "The name of the publishing topic";
    this->declare_parameter("publish_topic", "tf_locobot", param_desc);

    // Save parameters
    source_frame_ = this->get_parameter("source_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    debug_ = this->get_parameter("debug").as_bool();

    // Initialize the tf buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Initialize the transform broadcaster
    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(this->get_parameter("publish_topic").as_string(), 10);

    // Create a timer to look for new the apriltag tf
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / this->get_parameter("publish_frequency").as_double()),
      std::bind(&TFRemapper::TfCallback, this));
}


void TFRemapper::TfCallback() {
    // Save the last tf from the locobot marker frame to the camera frame
    try {
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                                                target_frame_, source_frame_,
                                                tf2::TimePointZero);
        // Change timestamp to now
        t.header.stamp = this->get_clock()->now();
        // Convert TransformStamped to TFMessage
        tf2_msgs::msg::TFMessage tf_message;
        tf_message.transforms.push_back(t);
        // Publish the TFMessage
        tf_publisher_->publish(tf_message);
    } catch (const tf2::TransformException & ex) {
        if (debug_) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                source_frame_.c_str(), target_frame_.c_str(), ex.what());
                return;
        }
    }    
}


// MAIN
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFRemapper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}