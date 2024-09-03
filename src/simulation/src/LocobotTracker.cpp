/**
 * @file LocobotTracker.cpp
 * @brief This file contains the implementation of the LocobotTracker class.
 * 
 * See LocobotTracker.h for more details.
 */

#include "LocobotTracker.h"

LocobotTracker::LocobotTracker(const string node_name, const string ns)
                : Node(node_name, ns) {

    // Declare parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Frame of the camera";
    this->declare_parameter("camera_frame", "env_camera_frame", param_desc);
    param_desc.description = "Frame of the map tag";
    this->declare_parameter("map_tag_frame", "map_tag", param_desc);
    param_desc.description = "Frame of the locobot tag";
    this->declare_parameter("locobot_tag_frame", "locobot_tag", param_desc);
    param_desc.description = "Timer period [s] to look for new the apriltag tf";
    this->declare_parameter("timer_period", 0.1, param_desc);
    param_desc.description = "Debug mode. Print errors in case of failure";
    this->declare_parameter("debug", false, param_desc);
    param_desc.description = "Look for the map_tag->camera tf";
    this->declare_parameter("look_for_map_tag", false, param_desc);

    // Save parameters
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    map_tag_frame_ = this->get_parameter("map_tag_frame").as_string();
    locobot_tag_frame_ = this->get_parameter("locobot_tag_frame").as_string();
    timer_period_ = this->get_parameter("timer_period").as_double();
    debug_ = this->get_parameter("debug").as_bool();
    look_for_map_tag_ = this->get_parameter("look_for_map_tag").as_bool();

    // Initialize the tf buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // Initialize the transform broadcaster
    tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf_locobot", 10);

    // Create a timer to look for new the apriltag tf
    timer_ = this->create_wall_timer(
      0.005s, [this]() {return this->TfCallback();});
}


void LocobotTracker::TfCallback() {
    // Save the last tf from the locobot marker frame to the camera frame
    try {
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                                                locobot_tag_frame_, camera_frame_,
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
                locobot_tag_frame_.c_str(), camera_frame_.c_str(), ex.what());
                return;
        }
    }

    // Republish the tf from the camera frame to the map frame to /tf_locobot topic
    if (look_for_map_tag_) {
        try {
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                                                    camera_frame_, map_tag_frame_,
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
                    camera_frame_.c_str(), map_tag_frame_.c_str(), ex.what());
                    return;
            }
        }
    }
    
}


// MAIN
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocobotTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}