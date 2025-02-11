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
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


// Services and actions
#include "locobot_control_interfaces/srv/last_error.hpp"
#include "locobot_control_interfaces/srv/clear_error.hpp"
#include "locobot_control_interfaces/srv/control_states.hpp"

#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <mutex>



/**
 * @brief StateMachine class to control the different states of the robot behavior.
 * 
 * The state machine is used to control the different states of the behavior of the robot.
 * The output of the state machine is expressed as a Result enum class.
 */
class StateMachine : public LocobotControl {
public:

    /**
     * @brief Construct a new State Machine object
     * 
     * The constructor initializes the state machine in the IDLE state and the result in INITIALIZED.
     */
    explicit StateMachine(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    /**
     * @brief Destructor
     */
    ~StateMachine();



    // Internal classes
    /**
     * @brief Enum class to represent the different states of the state machine.
     * 
     * The state machine is used to control the different states of the behavior of the robot.
     */
    enum class States : uint8_t {
        IDLE,                   // Waits for a new command
        SECURE_ARM,             // Secure the arm to avoid collision in navigation
        WAIT_ARM_SECURING,      // Wait for the arm to be secured
        SEND_NAV_GOAL,          // Send the navigation goal to the navigation stack
        WAIT_NAVIGATION,        // Wait for the navigation to reach the goal
        WAIT_ARM_EXTENDING,     // Wait for the arm to be extended
        ARM_EXTENDED,           // Open the gripper to release the object upon command
        WAIT_GRIPPER,           // Wait for the gripper to open
        WAIT_ARM_RETRACTING,    // Wait for the arm to retract
        ERROR,                  // Error state
        ABORT,                  // Abort the machine
        STOPPING                // Stop the machine
    };


    /**
     * @brief Enum class to represent the different results of the state machine.
     * 
     * The result of the state machine is used to determine the behavior of the main loop.
     */
    enum class Result : uint8_t {
        SUCCESS,    // The machine has completed the task successfully and the main loop terminates
        FAILURE,    // The machine has failed to complete the task
        RUNNING,    // The machine is still running
        INITIALIZED // The machine has been initialized
    };
    
    
    
    // Functions
    void nextState(); // Operate the state machine (send it to the next state)
    std::string getLastError() const {return errorMsg_;}; // Get the last error message
    States getMachineInternalState() const {return state_;}; // Get the current state of the machine
    Result getMachineExternalState() const {return result_;}; // Get the external state of the machine
    
    
    /**
     * @brief Convert the state enum to a string.
     * 
     * @param state The state to convert.
     * 
     * @return The string representation of the state.
     */
    std::string state_to_string(const States state) const;

    /**
     * @brief Convert the result enum to a string.
     * 
     * @param result The result to convert.
     * 
     * @return The string representation of the result.
     */
    std::string result_to_string(const Result result) const;


private:

    // Last Error Service Server
    
    // Service to request the last error message
    rclcpp::Service<locobot_control_interfaces::srv::LastError>::SharedPtr get_last_error_; 

    /**
     * @brief Service callback to return the last error message and state of the state machine.
     */
    void return_last_error(const std::shared_ptr<locobot_control_interfaces::srv::LastError::Request>,
                           std::shared_ptr<locobot_control_interfaces::srv::LastError::Response> response) {

        response->error_message = errorMsg_;
        response->last_state = state_to_string(state_);
    }

    // Clear Error Service Server

    // Service to clear error or abort state
    rclcpp::Service<locobot_control_interfaces::srv::ClearError>::SharedPtr clear_error_service_; 

    /**
     * @brief Service callback to clear the error (or aborted) state and message error.
     * 
     * @note The machine will enter the IDLE state if possible.
     */
    void clear_error_callback(const std::shared_ptr<locobot_control_interfaces::srv::ClearError::Request>,
                           std::shared_ptr<locobot_control_interfaces::srv::ClearError::Response> response) {

        response->successful = clear_error();
    }

    // Gripper control service server

    // Service to control the state machine state changes
    rclcpp::Service<locobot_control_interfaces::srv::ControlStates>::SharedPtr control_state_service_; 

    /**
     * @brief Service callback to control the state machine state's changes.
     * 
     * @note The service allows to change the state of the machine to IDLE, NAVIGATION, INTERACTION, 
     * or ABORT.
     */
    void change_state_callback(const std::shared_ptr<locobot_control_interfaces::srv::ControlStates::Request> request,
                           std::shared_ptr<locobot_control_interfaces::srv::ControlStates::Response> response) {
    
        // N.B requestedInteraction_ and requestedNavigation_ must be mutually exclusive
        
        // Lock the mutex to protect the state machine from concurrent access at the request variables
        std::lock_guard<std::mutex> lock(request_mutex_);
        
        switch (request->state) {
            case locobot_control_interfaces::srv::ControlStates::Request::IDLE:
                    requestedInteraction_ = false;
                    requestedNavigation_ = false;
                    response->successful_request = true;
                break;

            case locobot_control_interfaces::srv::ControlStates::Request::NAVIGATION:
                if (state_ == States::IDLE) {
                    requestedNavigation_ = true;
                    requestedInteraction_ = false;
                    response->successful_request = true;
                } else {
                    response->successful_request = false;
                }
                break;

            case locobot_control_interfaces::srv::ControlStates::Request::INTERACTION:
                if (state_ == States::IDLE) {
                    requestedNavigation_ = false;
                    requestedInteraction_ = true;
                    response->successful_request = true;
                } else {
                    response->successful_request = false;
                }
                break;

            case locobot_control_interfaces::srv::ControlStates::Request::OPEN_GRIPPER:
                if (state_ == States::ARM_EXTENDED) {
                    requestedGripperMovement_ = GripperState::RELEASED;
                    response->successful_request = true;
                } else {
                    response->successful_request = false;
                }
                break;

            case locobot_control_interfaces::srv::ControlStates::Request::CLOSE_GRIPPER:
                if (state_ == States::ARM_EXTENDED) {
                    /**
                    * NOTE: The gripper is set to HOME to allow the grasping of the object.
                    *       GripperState::GRASPING is not used to avoid the gripper to close too much.
                    */
                    requestedGripperMovement_ = GripperState::HOME;  
                    response->successful_request = true;
                } else {
                    response->successful_request = false;
                }
                break;

            case locobot_control_interfaces::srv::ControlStates::Request::ABORT:
                requestedAbort_ = true;
                requestedInteraction_ = false;
                requestedNavigation_ = false;
                response->successful_request = true;
                break;
        
            default:  
                response->successful_request = false;
                break;
        }
    }



    // Variables

    States state_{States::ERROR};  // Current state of the machine
    Result result_{Result::INITIALIZED}; // Result of the machine
    std::string errorMsg_{""}; // Error message
    bool requestedAbort_{false}; // Request to abort the machine
    bool requestedNavigation_{false}; // Request to navigate to the human
    bool requestedInteraction_{false}; // Request to interact with the human
    GripperState requestedGripperMovement_{GripperState::UNKNOWN}; // Requested gripper movement
    std::mutex request_mutex_; // Mutex to protect the state machine
    std::mutex next_state_mutex_; // Mutex to protect the next state
    std::string robot_frame_; // Frame of the robot tag
    std::string map_frame_; // Frame of the map tag
    std::string human_frame_; // Frame of the human tag
    bool follow_human_; // Follow the human or not after the first position
    int sleep_time_; // Sleep time in milliseconds for the state machine cycle
    bool debug_; // Debug flag
    int tf_tolerance_; // Tolerance in seconds for the TF

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};// Listener to the tf
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};// Buffer for the tf
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_updater_; // Publisher to update the navigation goal
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    std::thread machine_thread_; // Thread to operate the state machine

    // Functions
    void machineError(const std::string &msg); // Set the machine in error state and save the error message

    /**
     * @brief Function to operate the state machine.
     * 
     * The function is a loop that operates the state machine. The loop is executed until the
     * result state is different from RUNNING or INITIALIZED.
     */
    void spinMachine(); // Spin the machine control object

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
    bool checkTf(const std::string &to_frame, const std::string &from_frame);

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
    geometry_msgs::msg::PoseStamped GetHumanPose();

};


RCLCPP_COMPONENTS_REGISTER_NODE(StateMachine)

#endif  // STATE_MACHINE_H
