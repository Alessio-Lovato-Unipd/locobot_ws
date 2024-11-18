/**
 * @file StateMachine.cpp
 * 
 * @brief This file contains the implementation of the StateMachine class.
 */

#include "state_machine/StateMachine.h"


void StateMachine::machineError(const std::string &msg) {
    state_ = States::STOPPING;
    errorMsg_ = msg;
}

void StateMachine::nextState() {

    // Mutex to protect the state machine and read the requested states
    std::unique_lock<std::mutex> lock(request_mutex_);
    bool requestedAbort = requestedAbort_;
    bool requestedInteraction = requestedInteraction_;
    GripperState requestedGripperMovement = requestedGripperMovement_;
    lock.unlock();

    if (requestedAbort)
        state_ = States::ABORT;

    switch (state_) {
        case States::IDLE: // Initial state, wait for a new command
            if (requestedInteraction) {
                SetArmPose(ArmPose::HOME);
                state_ = States::WAIT_ARM_EXTENDING;
            }
            break;

        case States::WAIT_ARM_EXTENDING:  // Wait for the arm to be extended
            if (!isArmMoving()) {
                state_ = States::ARM_EXTENDED;
                if (isArmInError())
                    machineError("Arm extending failed");
            }
            break;


        case States::ARM_EXTENDED:  // Open the gripper to release the object upon command
            // Open the gripper if allowed
            if (requestedGripperMovement != GripperCurrentState() 
                        && requestedGripperMovement != GripperState::UNKNOWN) {
                state_ = States::WAIT_GRIPPER;
                SetGripper(requestedGripperMovement);
                requestedGripperMovement = GripperState::UNKNOWN;
            } else if (!requestedInteraction) {
                SetArmPose(ArmPose::SLEEP);
                state_ = States::WAIT_ARM_RETRACTING;
            }
            break;


        case States::WAIT_GRIPPER:  // Wait for the gripper to open
            if (!isArmMoving()) {
                state_ = States::ARM_EXTENDED;
                if (isArmInError())
                    machineError("Gripper movement failed");
            }
            break;


        case States::WAIT_ARM_RETRACTING:  // Wait for the arm to retract
            if (!isArmMoving()) {
                state_ = States::IDLE;
                if (isArmInError())
                    machineError("Arm retracting failed");
            }
            break;

        case States::STOPPING:
            StopArm();
            cancelNavigationGoal();
            state_ = States::ERROR;
            break;

        case States::ERROR:  // The machine is in error
            break;

        case States::ABORT:  // The machine is aborted
            StopArm(); // Stop the arm movement
            cancelNavigationGoal(); // Cancel the navigation goal
            clear_error();
            break;
    }
}


StateMachine::StateMachine (const rclcpp::NodeOptions & options) 
    : LocobotControl("StateMachine", "", options)  {

    // Declare parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Time [ms] to sleep between cycles of the state machine";
    this->declare_parameter("sleep_time", 100, param_desc);
    param_desc.description = "Topic where the state of the state machine is published";
    this->declare_parameter("state_topic", "machine_state", param_desc);

    // Save parameters
    sleep_time_ = this->get_parameter("sleep_time").as_int();

    if (this->get_parameter("sleep_time").as_int() <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Sleep time must be greater than 0");
        throw std::invalid_argument("Sleep time must be greater than 0");
    }else if (this->get_parameter("state_topic").as_string() == "") {
        RCLCPP_ERROR(this->get_logger(), "State topic was an empty string");
        throw std::invalid_argument("State topic was an empty string");
    }

    // Initialize the tf buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create the action server
    using namespace std::placeholders;

    // Create the last error service
    get_last_error_ = this->create_service<simulation_interfaces::srv::LastError>(
        "get_last_error", std::bind(&StateMachine::return_last_error, this, _1, _2));
    
    // Create the clear error service
    clear_error_service_ = this->create_service<simulation_interfaces::srv::ClearError>(
        "clear_error_state", std::bind(&StateMachine::clear_error_callback, this, _1, _2));

    // Create the control gripper service
    control_state_service_ = this->create_service<simulation_interfaces::srv::ControlStates>(
        "state_control", std::bind(&StateMachine::change_state_callback, this, _1, _2));

    // Create the state publisher
    state_publisher_ = this->create_publisher<std_msgs::msg::String>(this->get_parameter("state_topic").as_string(), 10);

    // Create the thread to spin the machine
    machine_thread_ = std::thread(&StateMachine::spinMachine, this);
}


void StateMachine::spinMachine() {
    // Initialize the feedback message
    States last_status = state_;
    // Main loop
    while (rclcpp::ok()) {
        if (result_ == Result::SUCCESS)
            break;

        // Update the feedback message
        std_msgs::msg::String status;
        status.data = state_to_string(state_);
        state_publisher_->publish(status);

        if (last_status != state_) {
            last_status = state_;
            RCLCPP_INFO(this->get_logger(), "Status: %s", state_to_string(state_).c_str());
        }
        nextState();
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_));
    }
    
}  

bool StateMachine::clear_error() {
    // Clear the error message and set the machine in the IDLE state
    if (state_ != States::ERROR && state_ != States::ABORT) 
        return false;

    errorMsg_ = "";
    std::unique_lock<std::mutex> lock(request_mutex_);
    requestedNavigation_ = false;
    requestedInteraction_ = false;
    requestedGripperMovement_ = GripperState::UNKNOWN;
    requestedAbort_ = false;
    while (isArmMoving()) {
        // Wait for the arm to stop
    }
    clearArmError();
    while(isNavigating()) {
        // Wait for the server to cancel the goal
    }
    clearNavigationError();
    result_ = Result::INITIALIZED;
    state_ = States::IDLE;
    lock.unlock();
    RCLCPP_INFO(this->get_logger(), "Error cleared. Machine in IDLE state");
    return true;
}


std::string StateMachine::state_to_string(const States state) const {
    // Return the string representation of the States enum
    switch (state) {
        case States::IDLE:
            return "IDLE";
        case States::WAIT_ARM_EXTENDING:
            return "WAIT_ARM_EXTENDING";
        case States::ARM_EXTENDED:
            return "ARM_EXTENDED";
        case States::WAIT_GRIPPER:
            return "WAIT_GRIPPER";
        case States::WAIT_ARM_RETRACTING:
            return "WAIT_ARM_RETRACTING";
        case States::ERROR:
            return "ERROR";
        case States::ABORT:
            return "ABORT";
        case States::STOPPING:
            return "STOPPING";
        default:
            return "UNKNOWN";
    }
}