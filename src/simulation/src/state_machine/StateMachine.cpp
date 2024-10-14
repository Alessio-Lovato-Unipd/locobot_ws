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

    // Check if Tf of both human and robot are available
    if (state_ != States::ERROR && !tf_available()) {
        machineError("TF not available");
    }
    // Mutex to protect the state machine and read the requested states
    std::unique_lock<std::mutex> lock(request_mutex_);
    bool requestedAbort = requestedAbort_;
    bool requestedNavigation = requestedNavigation_;
    bool requestedInteraction = requestedInteraction_;
    GripperState requestedGripperMovement = requestedGripperMovement_;
    lock.unlock();

    if (requestedAbort)
        state_ = States::ABORT;

    switch (state_) {
        case States::IDLE: // Initial state, wait for a new command
            if (requestedNavigation) {
                state_ = States::SECURE_ARM;
            } else if (requestedInteraction) {
                SetArmPose(ArmPose::HOME);
                state_ = States::WAIT_ARM_EXTENDING;
            }
            break;


        case States::SECURE_ARM:  // Send the arm to the secure position
            if (ArmCurrentPose() != ArmPose::SLEEP) {
                SetArmPose(ArmPose::SLEEP);
                state_ = States::WAIT_ARM_SECURING;
            } else {
                state_ = States::SEND_NAV_GOAL;
            }
            break;


        case States::WAIT_ARM_SECURING:  // Wait for the arm to be secured
            if (!isArmMoving()) {
                state_ = States::SEND_NAV_GOAL;
                if (isArmInError())
                    machineError("Arm securing failed");
            }
            break;


        case States::SEND_NAV_GOAL:  // Send the navigation goal to the navigation stack
            last_human_pose_ = GetHumanPose();
            MoveBaseTo(last_human_pose_);
            state_ = States::WAIT_NAVIGATION;
            break;
        

        case States::WAIT_NAVIGATION:  // Wait for the navigation to reach the goal
            // Check if the human has moved
            if (follow_human_)
                nav_goal_updater_->publish(GetHumanPose());

            // Check if the navigation has completed
            if (!requestedNavigation) { // Human has stopped navigation    
                if(isNavigating()) {
                    cancelNavigationGoal();
                }
                if (requestedInteraction) { // Human has requested interaction
                    SetArmPose(ArmPose::HOME);
                    state_ = States::WAIT_ARM_EXTENDING;
                } else {
                    state_ = States::IDLE;
                }
            } else if (!isNavigating()) { // Goal has been reached but human is still moving
                state_ = States::SEND_NAV_GOAL;  
            }
            if (isNavigationInError()) { // Navigation has failed
                machineError("Navigation failed");
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
    param_desc.description = "Frame of the robot tag";
    this->declare_parameter("robot_tag_frame", "locobot_tag", param_desc);
    param_desc.description = "Frame of the map tag (origin of the map)";
    this->declare_parameter("map_frame", "map", param_desc);
    param_desc.description = "Frame of the human tag";
    this->declare_parameter("human_tag_frame", "human_tag", param_desc);
    param_desc.description = "Indicates if the human should be followed after the first position";
    this->declare_parameter("follow_human", true, param_desc);
    param_desc.description = "Topic to update the navigation goal";
    this->declare_parameter("goal_update_topic", "goal_update", param_desc);
    param_desc.description = "Time [ms] to sleep between cycles of the state machine";
    this->declare_parameter("sleep_time", 100, param_desc);
    param_desc.description = "Topic where the state of the state machine is published";
    this->declare_parameter("state_topic", "machine_state", param_desc);

    // Save parameters
    robot_frame_ = this->get_parameter("robot_tag_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    human_frame_ = this->get_parameter("human_tag_frame").as_string();
    follow_human_ = this->get_parameter("follow_human").as_bool();
    sleep_time_ = this->get_parameter("sleep_time").as_int();

    if (robot_frame_ == "") {
        RCLCPP_ERROR(this->get_logger(), "Robot frame was an empty string");
        throw std::invalid_argument("Robot frame was an empty string");
    } else if (map_frame_ == "") {
        RCLCPP_ERROR(this->get_logger(), "Map frame was an empty string");
        throw std::invalid_argument("Map frame was an empty string");
    } else if (human_frame_ == "") {
        RCLCPP_ERROR(this->get_logger(), "Human frame was an empty string");
        throw std::invalid_argument("Human frame was an empty string");
    } else if (this->get_parameter("goal_update_topic").as_string() == "") {
        RCLCPP_ERROR(this->get_logger(), "Goal update topic was an empty string");
        throw std::invalid_argument("Goal update topic was an empty string");
    } else if (this->get_parameter("sleep_time").as_int() <= 0) {
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

    // Create the update goal publisher
    nav_goal_updater_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(this->get_parameter("goal_update_topic").as_string(), 10);

    // Create the state publisher
    state_publisher_ = this->create_publisher<std_msgs::msg::String>(this->get_parameter("state_topic").as_string(), 10);

    // Create the thread to spin the machine
    machine_thread_ = std::thread(&StateMachine::spinMachine, this);
}


StateMachine::~StateMachine() {
    // Stop the arm
    StopArm();
    // Cancel the navigation goal
    cancelNavigationGoal();
    // Set the result to completed
    result_ = Result::SUCCESS;
    //wait the thread to finish
    if (machine_thread_.joinable()) {
        machine_thread_.join();
    }
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


bool StateMachine::lookup_tf(const std::string &to_frame, const std::string &from_frame) {
    // Check if the requested TF is available
    try {
        // TODO: Perform the lookupTransform with a timeout to ensure the TF is available
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                                                from_frame, to_frame,
                                                now,
                                                std::chrono::milliseconds(200));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "TF %s -> %s not available", from_frame.c_str(), to_frame.c_str());
        return false;
    }
    return true;
}

bool StateMachine::tf_available(){
    // Check if the TF of the human and robot are available
    return (lookup_tf(robot_frame_, map_frame_) && lookup_tf(human_frame_, map_frame_));
}

double StateMachine::distance_to_human() const {  
    try {
        // Get pose of the human and robot
        geometry_msgs::msg::TransformStamped human_tf, robot_tf;
        // TODO: Perform the lookupTransform with a timeout to ensure the TF is available
        human_tf = tf_buffer_->lookupTransform(human_frame_, map_frame_, tf2::TimePointZero);
        robot_tf = tf_buffer_->lookupTransform(robot_frame_, map_frame_, tf2::TimePointZero);

        // Extract x and y positions from the two transforms
        double human_x = human_tf.transform.translation.x;
        double human_y = human_tf.transform.translation.y;
        double robot_x = robot_tf.transform.translation.x;
        double robot_y = robot_tf.transform.translation.y;

        return std::sqrt(std::pow(human_x - robot_x, 2) + std::pow(human_y - robot_y, 2));

    } catch (const tf2::TransformException & ex) {
        return -1.0;
    }
}


geometry_msgs::msg::PoseStamped StateMachine::GetHumanPose() {
    // Get the pose of the human relative to the map frame
    geometry_msgs::msg::TransformStamped human_tf;
    human_tf = tf_buffer_->lookupTransform(map_frame_, human_frame_, tf2::TimePointZero); // Get last pose, no need for check
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = map_frame_;
    pose.pose.position.x = human_tf.transform.translation.x;
    pose.pose.position.y = human_tf.transform.translation.y;
    pose.pose.position.z = 0.0; // Navigation is 2D
    pose.pose.orientation = human_tf.transform.rotation;
    return pose;
}




std::string StateMachine::state_to_string(const States state) const {
    // Return the string representation of the States enum
    switch (state) {
        case States::IDLE:
            return "IDLE";
        case States::SECURE_ARM:
            return "SECURE_ARM";
        case States::WAIT_ARM_SECURING:
            return "WAIT_ARM_SECURING";
        case States::SEND_NAV_GOAL:
            return "SEND_NAV_GOAL";
        case States::WAIT_NAVIGATION:
            return "WAIT_NAVIGATION";
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