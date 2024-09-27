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
#include <thread>
#include <vector>
#include <utility>

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
    UPRIGHT,
    UNKNOWN
};

/**
 * @brief Enum class to represent the different gripper states defined in the MoveIt configuration.
 */
enum class GripperState {
    HOME,
    RELEASED,
    GRASPING,
    UNKNOWN
};

/**
 * @brief Class to store the status of the arm.
 */
class ArmStatus {
    public:
        /**
         * @brief Construct a new arm_status object.
         * 
         * The constructor initializes the arm pose to the specified pose.
         * 
         * @param pose The arm pose. Default is ArmPose::SLEEP.
         */
        ArmStatus(const ArmPose pose = ArmPose::UNKNOWN) : pose_(pose) {};

        // Functions
        /**
         * @brief Save the arm pose and fripper state.
         * 
         * @param pose The arm pose to save.
         * @param state The gripper state to save. 
         */
        void save(const ArmPose pose, const GripperState state) {pose_ = pose; gripper_ = state;};

        /**
         * @brief Set the arm in movement or planning status.
         * 
         * @param moving True if the arm is moving or planning, false otherwise.
         */
        void in_motion(bool moving) {in_motion_ = moving;};

        /**
         * @brief Set the arm in error status.
         * 
         * @param error True if the arm is in error, false otherwise.
         */
        void in_error(bool error) {in_error_ = error;};

        // Access functions

        // Return true if the arm is moving or planning, false otherwise
        bool is_moving() const {return in_motion_;}; 
        // Return true if the arm is in error (planning or execution failed), false otherwise
        bool is_error() const {return in_error_;};
        // Return the current arm pose
        ArmPose get_arm_pose() const {return pose_;};
        // Return the current gripper state
        GripperState get_gripper_state() const {return gripper_;};

    private:
        ArmPose pose_;             // The current arm pose
        GripperState gripper_;     // The current gripper state
        bool in_motion_{false};    // True if the arm is moving, false otherwise
        bool in_error_{false};     // True if the arm is in error, false otherwise
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
                            const rclcpp::NodeOptions & options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true),
                            const ArmPose arm_pose = ArmPose::UNKNOWN);
    /**
     * @brief Destructor
     * 
     * Waits for the arm to stop moving before destroying the object.
     */
    ~LocobotControl() {
        while (arm_status_.is_moving()) {
           // Wait for the active threads to finish
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    };
    
    // Functions
    /**
     * @brief Send a goal to the navigation stack to move the robot to a specified pose.
     * 
     * @param pose The absolute pose relative to frame id where to move the robot to.
     * @param timeout The time to wait for the action server to be available. Default is 2 seconds.
     */
    void MoveBaseTo(const geometry_msgs::msg::PoseStamped &pose, const uint timeout = 2);

    /**
     * @brief Return the distance remaining to the goal.
     */
    float NavigationDistanceRemaining() const {return remaining_distance_;};

    /**
     * @brief Move the robot arm to a specified pose. Updates the arm status to moving.
     * 
     * @note This function invokes a detached thread to execute the plan to move 
     * the arm to the specified pose. Please make sure the condition 
     * isArmMoving() is false before performing a rclcpp::shutdown(), otherwise 
     * the execution will be interrupted due to a bad ROS2 context.
     * 
     * @param pose The predefined pose to move the robot arm to.
     * 
     * @param interface_name The name of the MoveGroupInterface to use. Default is 'interbotix_arm'.
     * 
     * @return True if the arm movement can be sent to the planning pipeline, false otherwise.
     */
    bool SetArmPose(const ArmPose pose, const string interface_name = "interbotix_arm");

    /**
     * @brief Move the gripper to a specified pose. Updates the arm status to moving.
     * 
     * @note This function invokes a detached thread to execute the plan to move 
     * the arm to the specified pose. Please make sure the condition 
     * isArmMoving() is false before performing a rclcpp::shutdown(), otherwise 
     * the execution will be interrupted due to a bad ROS2 context.
     * 
     * @param pose The predefined pose to move the gripper to.
     * 
     * @param interface_name The name of the MoveGroupInterface to use. Default is 'interbotix_gripper'.
     * 
     * @return True if the gripper movement can be send to the planning pipeline, false otherwise.
     */
    bool SetGripper(const GripperState state, const string interface_name = "interbotix_gripper");

    /**
     * @brief Stop the arm movement if it is in progress.
     * 
     * @param arm_interface_name The name of the MoveGroupInterface for the arm. Default is 'interbotix_arm'.
     * @param gripper_interface_name The name of the MoveGroupInterface for the gripper. Default is 'interbotix_gripper'.
     */
    void StopArm(const string arm_interface_name = "interbotix_arm", 
                    const string gripper_interface_name = "interbotix_gripper");

    /**
     * @brief Reset the arm status to not moving and not in error.
     */
    void ResetArmStatus() {arm_status_.in_motion(false); arm_status_.in_error(false);};


    // Return true if the arm is moving or planning, false otherwise
    bool isArmMoving() const {return arm_status_.is_moving();};
    // Return true if the arm is in error (planning or execution failed), false otherwise
    bool isArmInError() const {return arm_status_.is_error();};
    // Return the current arm pose
    ArmPose ArmCurrentPose() const {return arm_status_.get_arm_pose();};
    GripperState GripperCurrentState() const {return arm_status_.get_gripper_state();};


private:
    // Action client for the navigation stack
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    

    // Callbacks for the action client
    void goal_response_callback(GoalHandle::SharedPtr goal_handle);
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const GoalHandle::WrappedResult &result);

    // Variables
    float remaining_distance_{0.0}; // Distance remaining to the navigation goal
    ArmStatus arm_status_;          // Status of the arm

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
     * @param next_pose The next pose to move the arm to.
     * @param next_gripper_state The next gripper state to move the gripper to.
     */
    void ExecutePlan(const string interface_name, const ArmPose next_pose, 
                                    const GripperState next_gripper_state);

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