/**
 * @file Locobot_control.h
 * @brief This file contains the declaration of the LocobotControl class.
 * 
 * 
 * LocobotControl is a ROS2 node that controls the Locobot arm, gripper, and base. 
 * The class is responsible for moving the Locobot arm and gripper to a specified pose and position using MoveIt2.
 * The class also provides methods to move the Locobot base (Kobuki version) to a specified point in the map using the navigation2 stack.
 * The position of the arm and gripper can be set to predefined poses by enum classes ArmPose and GripperState or to a custom pose by geometry_msgs::msg::Pose.
*/

#ifndef LOCOBOT_CONTROL_H
#define LOCOBOT_CONTROL_H

// Libraries
#include <memory>
#include <string>
#include <future>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <moveit/move_group_interface/move_group_interface.h>


using std::string;
using moveit::planning_interface::MoveGroupInterface;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

/**
 * @brief Enum class to represent the different arm poses defined in the MoveIt configuration.
 */
enum class ArmPose {
    HOME,
    SLEEP,
    UPRIGHT
};

/**
 * @brief Enum class to represent the different gripper states defined in the MoveIt configuration.
 */
enum class GripperState {
    HOME,
    RELEASED,
    GRASPING
};

/**
 * @brief LocobotControl class performs the control of the Locobot arm, gripper and base.
 * 
 * The class is responsible for moving the Locobot arm and gripper to a specified pose and position 
 * using MoveIt2. The class also provides methods to move the Locobot base (Kobuki version) to a 
 * specified point in the map using the navigation2 stack.
 */
class LocobotControl : public rclcpp::Node {

public:

    /**
     * @brief Construct a new Locobot Control object.
     * 
     * The constructor initializes the action client for the navigation stack and the node.
     * 
     * @param name The name of the node. Default is 'locobot_controller'.
     * @param ns The namespace of the node. Default is 'locobot'.
     * @param options The node options. Default is automatically declare parameters from overrides.
     * 
     * @return A new LocobotControl object.
     */
    explicit LocobotControl(const string name = "locobot_controller", const string ns = "", 
                            const rclcpp::NodeOptions & options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    
    
    // Functions
    /**
     * @brief Send a goal to the navigation stack to move the robot to a specified pose.
     * 
     * @param pose The absolute pose relative to frame id where to move the robot to.
     * @param timeout The time to wait for the action server to be available. Default is 2 seconds.
     */
    void MoveBaseTo(const geometry_msgs::msg::PoseStamped &pose, const uint timeout = 2);

    /**
     * @brief Move the robot arm to a specified pose.
     * 
     * @param pose The predefined pose to move the robot arm to.
     * 
     * @return True if the arm was moved successfully, false otherwise.
     */
    bool SetArmPose(const ArmPose pose);

    /**
     * @brief Move the gripper to a specified pose.
     * 
     * @param pose The predefined pose to move the gripper to.
     * 
     * @return True if the gripper was moved successfully, false otherwise.
     */
    bool SetGripper(const GripperState state);


private:
    // Action client for the navigation stack
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

    // Callbacks for the client
    void goal_response_callback(GoalHandle::SharedPtr goal_handle);
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const GoalHandle::WrappedResult &result);

    // Functions
    /**
     * @brief Return the string representation of the ArmPose enum.
     * 
     * The method returns the string representation of the ArmPose enum, based on the 
     * defined poses in the MoveIt configuration.
     */
    string ArmPose_to_string(const ArmPose pose);

    /**
     * @brief Return the string representation of the GripperState enum.
     * 
     * The method returns the string representation of the GripperState enum, based on the 
     * defined gripper states in the MoveIt configuration.
     */
    string GripperState_to_string(const GripperState state);

    /**
     * @brief Execute the MoveIt plan.
     * 
     * The method executes the MoveIt plan using the MoveGroupInterface.
     * 
     * @param move_group_interface The MoveGroupInterface to execute the plan.
     * 
     * @return True if the plan was executed successfully, false otherwise.
     */
    bool ExecutePlan(moveit::planning_interface::MoveGroupInterface &move_group_interface);

};

/**
 * @brief Convert coordinates to a PoseStamped message.
 * 
 * @param x The x coordinate.
 * @param y The y coordinate.
 * @param z The z coordinate.
 * @param qx The x component of the quaternion.
 * @param qy The y component of the quaternion.
 * @param qz The z component of the quaternion.
 * @param qw The w component of the quaternion.
 * @param frame_id The frame id of the pose. Default is 'map'.
 * 
 * @return The PoseStamped message.
 */
geometry_msgs::msg::PoseStamped coordinates_to_pose(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id = "map");


#endif // LOCOBOT_CONTROL_H