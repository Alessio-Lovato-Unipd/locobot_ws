
/**
 * @file TFRemapper.h
 * @brief TFRemapper class
 * 
 * This file contains the declaration of the TFRemapper class, which is a class that 
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


using namespace std::chrono_literals;


/**
 * @brief TFRemapper class to remap the TFs.
 * 
 * @details The class looks for the TFs between the source and target frames in the /tf topic and publishes 
 * it in the topic passed as 'publish_topic' parameter. This allows to republish the TFs to another topic 
 * changing the relationship of the TFs.
 * 
 * @param publish_topic The topic to publish the TFs.
 * @param source_frame The source frame to look for the TFs.
 * @param target_frame The target frame to look for the TFs.
 * @param debug Debug mode. Print errors in case of failure.
 * @param publish_frequency The frequency to look at and publish the TFs.
 * 
 */
class TFRemapper : public rclcpp::Node {
    public:

        /**
         * @brief Construct a new TF Remapper object
         * 
         * @param node_name Name of the node
         * @param ns Namespace of the node
         * 
         * @return TFRemapper object
         */
        TFRemapper(const std::string node_name = "tf_remapper", const std::string ns = "");
    

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
        std::string source_frame_; // Source frame
        std::string target_frame_; // Target frame
        bool debug_; // Debug mode. Print errors in case of failure

        // Callback functions

        /**
         * @brief Callback function to look for the TFs and publish them.
         * 
         * @details The function looks for the TFs between the source and target frames and publishes them.
         * This allows to republish the TFs to another topic changing the relationship of the TFs.
         * 
         * @return void
         */
        void TfCallback();
};


#endif // LOCOBOT_TRACKER_H