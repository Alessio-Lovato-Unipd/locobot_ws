/**
 * @file TFPublisher.cpp
 * 
 * @brief Contains a ROS 2 node that publishes the tf set as parameter
 */

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include <chrono>


class TFPublisher : public rclcpp::Node {
public:
    /**
     * @brief Construct a new TFPublisher object
     * 
     * @param name The name of the node
     * @param ns The namespace of the node
     * @param options The node options
     *
     */
    TFPublisher(const std::string name = "tf_publisher", const std::string ns ="",
                const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
        : Node(name, ns, options) {
        
        // Declare parameters
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "The frame to publish from";
        this->declare_parameter("frame_id", "map", param_desc);
        param_desc.description = "The frame to publish to";
        this->declare_parameter("child_frame_id", "robot", param_desc);
        param_desc.description = "The frequency to publish the TF";
        this->declare_parameter("publish_frequency", 1.0, param_desc);
        param_desc.description = "The x position of the TF";
        this->declare_parameter("x", 0.0, param_desc);
        param_desc.description = "The y position of the TF";
        this->declare_parameter("y", 0.0, param_desc);
        param_desc.description = "The z position of the TF";
        this->declare_parameter("z", 0.0, param_desc);
        param_desc.description = "The roll of the TF";
        this->declare_parameter("roll", 0.0, param_desc);
        param_desc.description = "The pitch of the TF";
        this->declare_parameter("pitch", 0.0, param_desc);
        param_desc.description = "The yaw of the TF";
        this->declare_parameter("yaw", 0.0, param_desc);
        param_desc.description = "The topic to publish the TF";
        this->declare_parameter("topic", "tf", param_desc);

        // Create the tf
        tf2::Quaternion q;
        q.setRPY(this->get_parameter("roll").as_double(),
                 this->get_parameter("pitch").as_double(),
                 this->get_parameter("yaw").as_double());

        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();

        tf_.header.frame_id = this->get_parameter("frame_id").as_string();
        tf_.child_frame_id = this->get_parameter("child_frame_id").as_string();
        tf_.transform.translation.x = this->get_parameter("x").as_double();
        tf_.transform.translation.y = this->get_parameter("y").as_double();
        tf_.transform.translation.z = this->get_parameter("z").as_double();
        tf_.transform.rotation = q_msg;

        // Create the publisher
        tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>(this->get_parameter("topic").as_string(), 10);

        // Create the timer
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/this->get_parameter("publish_frequency").as_double()),
            std::bind(&TFPublisher::PublishTF, this));
    }

private:
    // tf message to publish
    geometry_msgs::msg::TransformStamped tf_{geometry_msgs::msg::TransformStamped()};
    // Publisher for the TF message
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
    // Timer to publish the TF message
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief Publish the TF message
     * 
     */
    void PublishTF() {
        // Change timestamp to now
        tf_.header.stamp = this->get_clock()->now();
        // Publish the TF message
        tf2_msgs::msg::TFMessage tf_message_;
        tf_message_.transforms.push_back(tf_);
        tf_publisher_->publish(tf_message_);
    };
};


// Main function
int main(int argc, char * argv[]) {
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);
    // Create the node
    auto node = std::make_shared<TFPublisher>();
    // Spin the node
    rclcpp::spin(node);
    // Shutdown the node
    rclcpp::shutdown();
    return 0;
}