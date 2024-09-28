/**
 * @file states.h
 * @brief This file contains the declaration of the StateMachine ROS2 node.
 * 
 * The node is responsible for controlling the behavior of the robot using an Action server.
 * 
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "LocobotControl.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


// Services and actions
#include "simulation_interfaces/srv/last_error.hpp"
#include "simulation_interfaces/srv/clear_error.hpp"
#include "simulation_interfaces/action/state_machine.hpp"

#include <string>
#include <thread>
#include <cmath>



/**
 * @brief Enum class to represent the different states of the state machine.
 * 
 * The state machine is used to control the different states of the behavior of the robot.
 */
enum class States : uint8_t {
    ERROR,                  // Error state
    IDLE,                   // Wait for a new goal (triggered by human distance more than 1m)
    CLEAR,                  // Clear the error state
    SECURE_ARM,             // Secure the arm to avoid collision in navigation
    WAIT_ARM_SECURING,      // Wait for the arm to be secured
    SEND_NAV_GOAL,          // Send the navigation goal to the navigation stack
    WAIT_NAVIGATION,        // Wait for the navigation to reach the goal
    NAVIGATION_COMPLETED,   // Extend the arm to deliver the object
    WAIT_ARM_EXTENDING,     // Wait for the arm to be extended
    ARM_EXTENDED,           // Open the gripper to release the object upon command
    WAIT_GRIPPER_OPENING,   // Wait for the gripper to open
    GRIPPER_OPENED,         // Wait for the human to take the object (wait 5 seconds)
    GRIPPER_CLOSING,        // Close the gripper
    WAIT_GRIPPER_CLOSING,   // Wait for the gripper to close
    GRIPPER_CLOSED,         // Retract the arm
    WAIT_ARM_RETRACTING,    // Wait for the arm to retract
    COMPLETED,              // The task has been completed
    ABORTING,               // The task is being aborted
    ABORTED                 // The task has been aborted
};

/**
 * @brief Enum class to represent the different results of the state machine.
 * 
 * The result of the state machine is used to determine the behavior of the main loop.
 */
enum class Result : uint8_t {
    SUCCESS,    // The machine has completed the task successfully
    FAILURE,    // The machine has failed to complete the task
    RUNNING,    // The machine is still running
    ABORTING,   // The machine is aborting
    ABORTED,    // The machine has been aborted
    INITIALIZED // The machine has been initialized
};


/**
 * @brief StateMachine class to control the different states of the robot behavior.
 * 
 * The state machine is used to control the different states of the behavior of the robot.
 * The output of the state machine is expressed as a Result enum class.
 */
class StateMachine : public LocobotControl {
public:

    using StateMachineAction = simulation_interfaces::action::StateMachine;
    using GoalHandleStateMachine = rclcpp_action::ServerGoalHandle<StateMachineAction>;
    /**
     * @brief Construct a new State Machine object
     * 
     * The constructor initializes the state machine in the IDLE state and the result in INITIALIZED.
     */
    explicit StateMachine(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Functions
    void nextState(); // Operate the state machine (send it to the next state)
    string getLastError() const {return errorMsg_;} // Get the last error message
    bool clearError(); // Clear the error message
    States getMachineState() const {return state_;} // Get the current state of the machine


private:

    // Action server

    rclcpp_action::Server<StateMachineAction>::SharedPtr action_server_; // Action server to control the state machine
    
    /**
     * @brief Handle the goal request.
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                            std::shared_ptr<const StateMachineAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received execution for State Machine");
        if (requestedAbort_) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected: machine is aborted. Please clear the ABORTED state.");
            return rclcpp_action::GoalResponse::REJECT;
        } else {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
    }

    /**
     *  @brief Handle the cancel request.
     */
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleStateMachine> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        requestedAbort_ = true;
        errorMsg_ = "Goal canceled by user";
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief Handle the accepted goal.
     */
    void handle_accepted(const std::shared_ptr<GoalHandleStateMachine> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&StateMachine::spinMachine, this, _1), goal_handle}.detach();
    }


    // Last Error Service Server
    
    // Service to request the last error message
    rclcpp::Service<simulation_interfaces::srv::LastError>::SharedPtr get_last_error_; 

    /**
     * @brief Service to return the last error message and state of the state machine.
     */
    void return_last_error(const std::shared_ptr<simulation_interfaces::srv::LastError::Request> request,
                           std::shared_ptr<simulation_interfaces::srv::LastError::Response> response) {

        response->error_message = errorMsg_;
        response->last_state = state_to_string(state_);
    }

    // Clear Error Service Server

    // Service to clear error or abort state
    rclcpp::Service<simulation_interfaces::srv::ClearError>::SharedPtr clear_error_service_; 

    /**
     * @brief Service to clear the error (or aborted) state and message error.
     * 
     * @note The machine will enter the IDLE state if possible.
     */
    void clear_error_callback(const std::shared_ptr<simulation_interfaces::srv::ClearError::Request> request,
                           std::shared_ptr<simulation_interfaces::srv::ClearError::Response> response) {

        response->successful = clear_error();
    }
    

    // Variables

    States state_{States::IDLE};  // Current state of the machine
    Result result_{Result::INITIALIZED}; // Result of the machine
    std::string errorMsg_{""}; // Error message
    bool requestedAbort_{false}; // Request to abort the machine
    std::string robot_frame_{""}; // Frame of the robot tag
    std::string map_frame_{""}; // Frame of the map tag
    std::string human_frame_{""}; // Frame of the human tag
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};// Listener to the tf
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;// Buffer for the tf
    

    // Functions
    void onError(); // Perform actions when in error state
    void machineError(const std::string &msg); // Set the machine in error state and save the error message

    /**
     * @brief Function to operate the state machine.
     * 
     * The function is a loop that terminates when the machine is not in the RUNNING 
     *  or ABORTING Result state. Otherwise, the machine will terminate when rclcpp is shutdown.
     */
    void spinMachine(const std::shared_ptr<GoalHandleStateMachine> goal_handle); // Spin the machine control object

    /**
     * @brief Function to clear the error message and set the machine in the IDLE state.
     * 
     * @return True if the error can be cleared, false otherwise.
     */
    bool clear_error();

    /**
     * @brief Function to determine if the TF of the human and robot are available.
     * 
     * @return True if the TF of the human and robot are available, false otherwise.
     */
    bool tf_available();

    /**
     * @brief Function to determine if the requested TF is available.
     * 
     * @param to_frame The frame to look to.
     * @param from_frame The frame to look from.
     * 
     * @return True if the requested TF is available, false otherwise.
     */
    bool lookup_tf(const string &to_frame, const string &from_frame);

    /**
     * @brief Function to calculate the planar Euclidean distance between the robot and the human.
     * 
     * @note Map tag frame is used as the origin of the map.
     * 
     * @return The absolute distance between the robot and the human in meters. If an error 
     * occurs, the function returns -1.0.
     */
    double distance_to_human() const;

    /**
     * @brief Obtains the pose of the human relative to the map frame.
     * 
     * @return The pose of the human relative to the map frame.
     */
    geometry_msgs::msg::PoseStamped GetHumanPose() const;

    /**
     * @brief Convert the state enum to a string.
     * 
     * @param state The state to convert.
     * 
     * @return The string representation of the state.
     */
    std::string state_to_string(const States state) const;
};


RCLCPP_COMPONENTS_REGISTER_NODE(StateMachine)

#endif  // STATE_MACHINE_H
