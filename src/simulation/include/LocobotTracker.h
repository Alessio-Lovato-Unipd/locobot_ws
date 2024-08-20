
/**
 * @file LocobotTracker.h
 * @brief LocobotTracker class
 * 
 * This file contains the declaration of the LocobotTracker class, which is a class that 
 * uses the tf published of some Apriltags to track the position of the robot in the map.
 */

#ifndef LOCOBOT_TRACKER_H
#define LOCOBOT_TRACKER_H

//Libraries
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS 2
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using std::string;
using namespace std::chrono_literals;

class LocobotTracker : public rclcpp::Node {
    public:

        /**
         * @brief Construct a new Locobot Tracker object
         * 
         * @return LocobotTracker object
         */
        LocobotTracker(const string node_name = "locobot_tracker", const string ns = "env_camera");
    

    private:
        // Broadcaster to the locobot tf relative to the map frame
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
        // Listener to the tf
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        // Buffer for the tf
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        // Timer to listen to the tf
        rclcpp::TimerBase::SharedPtr timer_{nullptr};

        //Node variables
        string camera_frame_; // Frame of the map tag
        string map_tag_frame_; // Frame of the map tag
        string locobot_tag_frame_; // Frame of the locobot tag
        float timer_period_; // Timer period [s] to look for new the apriltag tf

        // Callback functions
        void TfCallback();
};


#endif // LOCOBOT_TRACKER_H