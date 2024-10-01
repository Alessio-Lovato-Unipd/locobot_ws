
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
         * @param node_name Name of the node
         * @param ns Namespace of the node
         * 
         * @return LocobotTracker object
         */
        LocobotTracker(const string node_name = "locobot_tracker", const string ns = "env_camera");
    

    private:
        // Broadcaster to the locobot tf relative to the map frame
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
        // Listener to the tf
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        // Buffer for the tf
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        // Timer to listen to the tf
        rclcpp::TimerBase::SharedPtr timer_;

        //Node variables
        string camera_frame_; // Frame of the map tag
        string map_tag_frame_; // Frame of the map tag
        string locobot_tag_frame_; // Frame of the locobot tag
        float timer_period_; // Timer period [s] to look for new the apriltag tf
        bool debug_; // Debug mode. Print errors in case of failure
        bool look_for_map_tag_; // Look for the tf from the 'camera frame' to the 'map tag frame'

        // Callback functions

        /**
         * @brief Callback function to look for new the tf after a certain period of time.
         * Il will look for the tf from the 'locobot tag frame' to the 'camera frame' and for the tf from the 'camera frame'
         *  to the 'map tag frame'.
         * If the tf is found, it will publish the tf in the other direction on the topic 'tf_locobot'.
         * If 'debug' parameter is true, it will print the error message in case the tf is not found.
         * 
         * @return void
         */
        void TfCallback();
};


#endif // LOCOBOT_TRACKER_H