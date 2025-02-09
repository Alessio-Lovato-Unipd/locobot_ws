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
#include <optional>
#include <variant>
#include <chrono>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/duration.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <moveit/move_group_interface/move_group_interface.h>


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

using PoseOrState = std::variant<ArmPose, GripperState>;

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
         * @param arm_pose The arm pose. Default is ArmPose::SLEEP.
         * @param gripper The gripper state. Default is GripperState::HOME.
         */
        ArmStatus(const ArmPose pose = ArmPose::SLEEP, const GripperState gripper = GripperState::HOME) 
                : pose_(pose), gripper_(gripper) {};

        // Functions
        /**
         * @brief Save the arm pose and fripper state.
         * 
         * @param pose The arm pose to save.
         * @param state The gripper state to save. 
         */
        void updateStatus(const ArmPose pose, const GripperState gripper) {pose_ = pose; gripper_ = gripper;};

        /**
         * @brief Set the arm in movement or planning status.
         * 
         * @param moving True if the arm is moving or planning, false otherwise.
         */
        void in_motion(bool moving) {in_motion_ = moving;};

        /**
         * @brief Set the arm in error status.
         * 
         * @param in_error True if the arm is in error, false otherwise.
         */
        void errorState(bool in_error) {in_error_ = in_error;};

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


class NavigationStatus {
    public:
        // Constructor
        NavigationStatus() {};

        // Functions

        /**
         * @brief Save the distance remaining to the goal and the estimated time of arrival.
         * 
         * @param distance The distance remaining to the goal.
         * @param ETA The estimated time of arrival.
         */
        void updateStatus(const float distance, const double ETA) {remaining_distance_ = distance;
                                                                    remaining_time_ = ETA;};

        /**
         * @brief Set the navigation in progress status.
         * 
         * @param state True if the navigation is in progress, false otherwise.
         */
        void startNavigation() {in_progress_ = true;};

        /**
         * @brief End the navigation status.
         * 
         * @param state The result from the navigation action server.
         */
        void stopNavigation(const rclcpp_action::ResultCode state) {
            in_progress_ = false; 
            navigation_result_ = state;
            state == rclcpp_action::ResultCode::SUCCEEDED ? in_error_ = false : in_error_ = true;
        };

        /**
         * @brief Clear the navigation error status.
         */
        void clearError() {in_error_ = false;};

        // Access functions

        // Return the distance remaining to the navigation goal
        float getDistance() const {return remaining_distance_;};
        // Return the estimated time of arrival
        double getETA() const {return remaining_time_;};
        // Return true if the navigation is in progress, false otherwise
        bool isMoving() const {return in_progress_;};
        // Return true if the navigation is in error, false otherwise
        bool isError() const {return in_error_;};
        // Return the result of the navigation action server
        rclcpp_action::ResultCode getResult() const {return navigation_result_;};

    private:

        // Variables

        float remaining_distance_{0.0}; // Distance remaining to the navigation goal
        double remaining_time_{0.0}; // Time remaining to the navigation goal
        bool in_progress_{false}; // True if the navigation is in progress, false otherwise
        bool in_error_{false}; // True if the navigation is in error, false otherwise
        rclcpp_action::ResultCode navigation_result_{rclcpp_action::ResultCode::UNKNOWN}; // Result of the navigation action server
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
    explicit LocobotControl(const std::string name = "locobot_controller", const std::string ns = "", 
                            const rclcpp::NodeOptions & options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true),
                            const ArmPose arm_pose = ArmPose::UNKNOWN);
    /**
     * @brief Destructor
     * 
     * Waits for the arm to stop moving before destroying the object.
     */
    ~LocobotControl() {
        while (arm_status_.is_moving() or navigation_status_.isMoving()) {
           // Wait for the active threads to finish
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    };
    
    // Functions
    /**
     * @brief Send a goal to the navigation stack to move the robot to a specified pose.
     * 
     * @param pose The absolute pose relative to frame id where to move the robot to.
     * @param timeout The time to wait for the action server to be available. Default 
     * is set as the timeout defined in the parameters.
     */
    void MoveBaseTo(const geometry_msgs::msg::PoseStamped &pose, std::optional<double> timeout = std::nullopt);

    /**
     * @brief Return the distance remaining to the goal.
     */
    float NavigationDistanceRemaining() const {return navigation_status_.getDistance();};

    /**
     * @brief Move the robot arm to a specified pose. Updates the arm status to moving.
     * 
     * @note This function invokes a detached thread to execute the plan to move 
     * the arm to the specified pose. Please make sure the condition 
     * isArmMoving() is false before performing a rclcpp::shutdown(), otherwise 
     * the execution will be interrupted due to a bad ROS2 context.
     * 
     * @param pose The predefined pose to move the robot arm to.
     * @param interface_name The name of the MoveGroupInterface to use. Default is set as the arm interface name
     * defined in the parameters.
     * 
     * @return True if the arm movement can be sent to the planning pipeline, false otherwise.
     */
    bool SetArmPose(const ArmPose pose, std::optional<std::string> interface_name = std::nullopt);

    /**
     * @brief Move the gripper to a specified pose. Updates the arm status to moving.
     * 
     * @note This function invokes a detached thread to execute the plan to move 
     * the arm to the specified pose. Please make sure the condition 
     * isArmMoving() is false before performing a rclcpp::shutdown(), otherwise 
     * the execution will be interrupted due to a bad ROS2 context.
     * 
     * @param pose The predefined pose to move the gripper to.
     * @param interface_name The name of the MoveGroupInterface to use. Default is set as the gripper interface name 
     * defined in the parameters.
     * 
     * @return True if the gripper movement can be send to the planning pipeline, false otherwise.
     */
    bool SetGripper(const GripperState state, std::optional<std::string> interface_name = std::nullopt);

    /**
     * @brief Stop the arm movement if it is in progress.
     * 
     */
    void StopArm();

    /**
     * @brief Cancel all the goals sent to the navigation server.
     * 
     * @details Does not wait for the action server to be available and the answer to the cancel request.
     * 
     * @param async_cancellation True if want to wait the response of the cancel request, false otherwise.
     * 
     * @return True if is not asynchronous or if the request has succeeded, false otherwise.
     * 
     * @note The function send a cancel request to the navigation action server to stop the robot.
     */
    bool cancelNavigationGoal(const bool async_cancellation = false);

    // Return true if the navigation action server is available, false otherwise. Timeout is set to 1 second.
    bool isNavigationServerAvailable() const {return this->nav_client_ptr_->wait_for_action_server(std::chrono::seconds(1));};
    // Return true if the arm is moving or planning, false otherwise
    bool isArmMoving() const {return arm_status_.is_moving();};
    // Return true if the arm is in error (planning or execution failed), false otherwise
    bool isArmInError() const {return arm_status_.is_error();};
    // Clear the arm error status
    void clearArmError() {arm_status_.errorState(false); arm_status_.in_motion(false);};
    // Return true if the navigation is in progress, false otherwise
    bool isNavigating() const {return navigation_status_.isMoving();};
    // Return true if the navigation is in error, false otherwise
    bool isNavigationInError() const {return navigation_status_.isError();};
    // Clear the navigation error status
    void clearNavigationError() {navigation_status_.clearError();};
    // Return the result of the navigation action server
    rclcpp_action::ResultCode NavigationResult() const {return navigation_status_.getResult();};
    // Return the estimated time of arrival
    double NavigationETA() const {return navigation_status_.getETA();};
    // Return the remaining distance to the navigation goal
    float NavigationDistance() const {return navigation_status_.getDistance();};
    // Return the current arm pose
    ArmPose ArmCurrentPose() const {return arm_status_.get_arm_pose();};
    // Return the current gripper state
    GripperState GripperCurrentState() const {return arm_status_.get_gripper_state();};
    // Return the string of the MoveIt arm interface name
    std::string ArmInterfaceName() const {return arm_interface_name_;};
    // Return the string of the MoveIt gripper interface name
    std::string GripperInterfaceName() const {return gripper_interface_name_;};


private:
    // Action client for the navigation stack
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_ptr_;

    

    // Callbacks for the action client
    void goal_response_callback(GoalHandle::SharedPtr goal_handle);
    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const GoalHandle::WrappedResult &result);

    // Variables
    ArmStatus arm_status_;          // Status of the arm
    NavigationStatus navigation_status_; // Status of the navigation
    std::string navigation_server_name_; // Name of the navigation action server
    std::string arm_interface_name_; // Name of the MoveGroupInterface for the arm
    std::string gripper_interface_name_; // Name of the MoveGroupInterface for the gripper
    double timeout_; // Timeout for the navigation action server waiting time in seconds

    // Functions
    /**
     * @brief Return the string representation of the ArmPose enum.
     * 
     * The method returns the string representation of the ArmPose enum, based on the 
     * defined poses in the MoveIt configuration.
     */
    std::string ArmPose_to_string(const ArmPose pose);

    /**
     * @brief Return the string representation of the GripperState enum.
     * 
     * The method returns the string representation of the GripperState enum, based on the 
     * defined gripper states in the MoveIt configuration.
     */
    std::string GripperState_to_string(const GripperState state);

    /**
     * @brief Plan and execute the MoveIt plan.
     * 
     * The method plan and executes the movement using the MoveGroupInterface.
     * 
     * @param move_group_interface The MoveGroupInterface to execute the plan.
     * @param target The target pose or state to move the arm to.
     * 
     * @return True if the plan is executed successfully, false otherwise.
     */
    bool ExecutePlan(const std::string interface_name, const PoseOrState target);

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