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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"



// Services and actions
#include "simulation_interfaces/srv/last_error.hpp"
#include "simulation_interfaces/srv/clear_error.hpp"
#include "simulation_interfaces/srv/control_states.hpp"

#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <mutex>
#include <stdexcept>



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
    COMPLETED               // The state machine has completed the task
};


/**
 * @brief StateMachine class to control the different states of the robot behavior.
 * 
 * The state machine is used to control the different states of the behavior of the robot.
 * The output of the state machine is expressed as a Result enum class.
 */
class StateMachine : public rclcpp_lifecycle::LifecycleNode {
public:
    /**
     * @brief Construct a new State Machine object
     */
    explicit StateMachine(
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions().use_intra_process_comms(true))
        : rclcpp_lifecycle::LifecycleNode("state_machine", "", options) {};

    // Functions
    void nextState(); // Operate the state machine (sends it to the next state)
    std::string getLastError() const {return errorMsg_;} // Get the last error message
    States getMachineState() const {return state_;} // Get the current state of the machine

    // Callbacks
    /**
     * @brief Callback to handle the configuration of the node.
     * @return Success if the configuration is successful, Failure otherwise.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & state);

    /**
     * @brief Callback to handle the activation of the node.
     * @return Success if the activation is successful, Failure otherwise.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & state);

    /**
     * @brief Callback to handle the deactivation of the node.
     * @return Success if the deactivation is successful, Failure otherwise.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & state);

    /**
     * @brief Callback to handle the errors of the node.
     * @return Success if the error is handled, Failure otherwise.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
        const rclcpp_lifecycle::State & state);

    /**
     * @brief Callback to handle the shutdown of the node.
     * @return Success if the shutdown is successful, Failure otherwise.
     */
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State & state);

private:

    // Locobot Node
    std::shared_ptr<LocobotControl> locobot_{nullptr}; // Node to handle the locobot control node

    // Last Error Service Server
    
    // Service to request the last error message
    rclcpp::Service<simulation_interfaces::srv::LastError>::SharedPtr get_last_error_; 

    /**
     * @brief Service callback to return the last error message and state of the state machine.
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
     * @brief Service callback to clear the error state and message error.
     * 
     * @note The machine will enter the IDLE state if possible.
     */
    void clear_error_callback(const std::shared_ptr<simulation_interfaces::srv::ClearError::Request> request,
                            std::shared_ptr<simulation_interfaces::srv::ClearError::Response> response) {

        // Trigger the on_activate callback
        result = this->activate();
        if (result.label() != "Active") {
            response->successful = false;
            return;
        }

        // If both transitions are successful
        response->successful = true;
    }

    // Status control service server

    // Service to control the state machine state changes
    rclcpp::Service<simulation_interfaces::srv::ControlStates>::SharedPtr control_state_service_; 

    /**
     * @brief Service callback to control the state machine state's changes.
     * 
     * @note The service allows to change the state of the machine to IDLE, NAVIGATION, INTERACTION and 
     * send gripper commands.
     */
    void change_state_callback(const std::shared_ptr<simulation_interfaces::srv::ControlStates::Request> request,
                           std::shared_ptr<simulation_interfaces::srv::ControlStates::Response> response) {
    
        // N.B requestedInteraction_ and requestedNavigation_ must be mutually exclusive
        
        // Lock the mutex to protect the state machine from concurrent access at the request variables
        std::lock_guard<std::mutex> lock(request_mutex_);
       
        switch (request->state) {
            case simulation_interfaces::srv::ControlStates::Request::IDLE:
                requestedInteraction_ = false;
                requestedNavigation_ = false;
                response->successful_request = true;
                break;

            case simulation_interfaces::srv::ControlStates::Request::NAVIGATION:
                requestedNavigation_ = true;
                requestedInteraction_ = false;
                response->successful_request = true;
                break;

            case simulation_interfaces::srv::ControlStates::Request::INTERACTION:
                requestedNavigation_ = false;
                requestedInteraction_ = true;
                response->successful_request = true;
                break;

            case simulation_interfaces::srv::ControlStates::Request::OPEN_GRIPPER:
                requestedGripperMovement_ = GripperState::RELEASED;
                response->successful_request = true;
                break;

            case simulation_interfaces::srv::ControlStates::Request::CLOSE_GRIPPER:
                requestedGripperMovement_ = GripperState::GRASPING;
                response->successful_request = true;
                break;
        
            default:
                response->successful_request = false;
                break;
        }
    }



    // Private Variables

    States state_{States::IDLE};  // Current state of the machine
    std::string errorMsg_{""}; // Error message
    bool requestedAbort_{false}; // Request to abort the machine
    bool requestedNavigation_{false}; // Request to navigate to the human
    bool requestedInteraction_{false}; // Request to interact with the human
    GripperState requestedGripperMovement_{GripperState::UNKNOWN}; // Requested gripper movement
    std::mutex request_mutex_; // Mutex to protect the state machine
    std::string robot_frame_; // Frame of the robot tag
    std::string map_frame_; // Frame of the map tag
    std::string human_frame_; // Frame of the human tag
    std::string goal_update_topic_; // Topic to update the navigation goal
    bool follow_human_; // Follow the human or not after the first position
    int sleep_time_; // Sleep time in milliseconds for the state machine cycle
    geometry_msgs::msg::PoseStamped last_human_pose_; // Pose of the human

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};// Listener to the tf
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_{nullptr};// Buffer for the tf
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_updater_{nullptr}; // Publisher to update the navigation goal
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr info_publisher_{nullptr}; // Publisher to publish the state info

    // Main thread
    std::thread main_thread_; // Main thread to operate the state machine
    bool pause_main_thread_{true}; // Flag to stop the main thread
    std::mutex main_thread_mutex_; // Mutex to protect the main thread

    // Locobot thread
    std::thread locobot_thread_; // Thread to operate the locobot control node
    bool pause_locobot_thread_{true}; // Flag to stop the locobot thread
    std::mutex locobot_thread_mutex_; // Mutex to protect the locobot thread
    bool first_time_{true}; // Flag to indicate if the configuration is performed for the first time

    // Private Functions

    /**
     * @brief Save the error message and trigger the error transition.
     */
    void machineError(const std::string &msg);

    /**
     * @brief Main loop of the state machine.
     */
    void mainLoop();

    /**
     * @brief Locobot control loop.
     */
    void locobotLoop(std::shared_ptr<LocobotControl> locobot);

    /**
     * @brief Function to determine if the TF of the human and robot are available.
     * 
     * @return True if the TF of the human and robot are available, false otherwise.
     */
    bool tf_available() const;

    /**
     * @brief Function to determine if the requested TF is available.
     * 
     * @param to_frame The frame to look to.
     * @param from_frame The frame to look from.
     * 
     * @return True if the requested TF is available, false otherwise.
     */
    bool lookup_tf(const std::string &to_frame, const std::string &from_frame) const;

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
