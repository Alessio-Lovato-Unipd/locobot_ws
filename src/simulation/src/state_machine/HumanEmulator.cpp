/**
 * @file HumanEmulator.cpp
 * 
 * @brief Implements a class that emulates the human movement.
 * 
 * @details This class is used to emulate the human movement (publishes a transform of human position). It is used to test the state machine.
 */
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_msgs/msg/tf_message.hpp>

class HumanEmulator : public rclcpp::Node
{
public:
    /**
     * @brief Constructor of the HumanEmulator class.
     * 
     * @param node_name The name of the node.
     */
    HumanEmulator(const std::string &node_name, const geometry_msgs::msg::Pose &human_pose,
                    const bool static_human = false, const std::string &human_frame = "human_tag", 
                    const std::string &map_frame = "map")
        : Node(node_name), static_human_(static_human), human_frame_(human_frame), 
            map_frame_(map_frame) {
        
        // Set the human pose
        human_pose_.transform.translation.x = human_pose.position.x;
        human_pose_.transform.translation.y = human_pose.position.y;
        human_pose_.transform.translation.z = human_pose.position.z;
        human_pose_.transform.rotation = human_pose.orientation;

        // Initialize the transform broadcaster
        tf_publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 10);

        // Set the timer to publish the human transform
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&HumanEmulator::publishHumanTransform, this));
        step_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&HumanEmulator::Step, this));
    }

private:
    // Variables
    geometry_msgs::msg::TransformStamped human_pose_{geometry_msgs::msg::TransformStamped()}; // Transform of the human
    bool static_human_; // Flag to indicate if the human is static
    std::string human_frame_; // Frame of the human
    std::string map_frame_; // Frame of the map
    uint8_t step_number{0}; // Number of steps of the human
    float step_size{1.0}; // Size of the step of the human [m]
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_; // Transform broadcaster
    rclcpp::TimerBase::SharedPtr timer_; // Timer to publish the human transform
    rclcpp::TimerBase::SharedPtr step_timer_; // Timer to update the human position

    /**
     * @brief Function to publish the transform of the human.
     */
    void publishHumanTransform() {

        // Set the header
        human_pose_.header.stamp = this->now();
        human_pose_.header.frame_id = map_frame_;
        human_pose_.child_frame_id = human_frame_;

        // Publish the human transform
        // Convert TransformStamped to TFMessage
        tf2_msgs::msg::TFMessage tf_message;
        tf_message.transforms.push_back(human_pose_);
        // Publish the TFMessage
        tf_publisher_->publish(tf_message);
    }
    
    /**
     * @brief Function to calculate the human position.
     */
    void calculateHumanPosition() {
        // Update the human position
        switch (step_number) {
            case 7:
            case 8:
                human_pose_.transform.translation.x -= step_size;
                break;
            case 9:
            case 10:
            case 11:
                human_pose_.transform.translation.y += step_size;
                break;
            case 12:
                human_pose_.transform.translation.x += step_size;
                break;
            default:
                break;
        }
    }

    void Step() {
        // Update the human position
        if (!static_human_)
            calculateHumanPosition();

        step_number++;
    }
};

int main(int argc, char **argv) {
    // Initialize the ROS node
    rclcpp::init(argc, argv);

    // Create the human emulator
    geometry_msgs::msg::Pose human_pose;
    human_pose.position.x = 0.0;
    human_pose.position.y = 1.5;
    human_pose.position.z = 0.0;
    human_pose.orientation.x = 0.0;
    human_pose.orientation.y = 0.0;
    human_pose.orientation.z = 1.0;
    human_pose.orientation.w = 0;
    auto human_emulator = std::make_shared<HumanEmulator>("human_emulator", human_pose);

    // Spin the node
    rclcpp::spin(human_emulator);

    // Shutdown the ROS node
    rclcpp::shutdown();

    return 0;
}