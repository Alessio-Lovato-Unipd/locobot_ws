/**
 * @file StateMachine.cpp
 * 
 * @brief This file contains the implementation of the StateMachine class.
 */

#include "state_machine/StateMachine.h"


void StateMachine::machineError(const std::string &msg) {
    state_ = States::ERROR;
    errorMsg_ = msg;
}

void StateMachine::nextState() {

    // Check if Tf of both human and robot are available
    if (!tf_available()) {
        machineError("TF not available");
    } else if (robot_frame_ == "" || map_frame_ == "" || human_frame_ == "") { // Check if the robot frame and map tag frame are available
        machineError("Robot frame or map tag frame not passed as parameters");
    }
    // Mutex to protect the state machine
    std::unique_lock<std::mutex> lock(request_mutex_);

    if (requestedAbort_ && state_!= States::ABORTED) // Check if an abort has been requested
        state_ = States::ABORTING;
    
    lock.unlock();

    switch (state_) {
        case States::IDLE: // Initial state, wait for a new command
            lock.lock();
            if (requestedAbort_) {
                lock.unlock();
                state_ = States::ABORTING;
            } else if (requestedNavigation_) {
                lock.unlock();
                state_ = States::SECURE_ARM;
            } else if (requestedInteraction_) {
                lock.unlock();
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
            lock.lock();
            if (!requestedNavigation_) { // Human has stopped navigation    
                lock.unlock();
                if(isNavigating()) {
                    cancelNavigationGoal();
                }
                lock.lock();
                if (requestedInteraction_) { // Human has requested interaction
                    lock.unlock();
                    SetArmPose(ArmPose::HOME);
                    state_ = States::WAIT_ARM_EXTENDING;
                } else {
                    lock.unlock();
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
            lock.lock();
            if (requestedGripperMovement_ != GripperCurrentState() 
                        && requestedGripperMovement_ != GripperState::UNKNOWN) {
                lock.unlock();
                SetGripper(requestedGripperMovement_);
                requestedGripperMovement_ = GripperState::UNKNOWN;
                state_ = States::WAIT_GRIPPER;
            } else if (!requestedInteraction_) {
                lock.unlock();
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


        case States::ERROR:  // The machine is in error
            StopArm(); // Stop the arm movement
            cancelNavigationGoal(); // Cancel the navigation goal
            result_ =  Result::FAILURE;
            break;


        case States::ABORTING:  // The machine is aborting
            result_ = Result::ABORTING;
            state_ = States::ABORTED;
            StopArm(); // Stop the arm movement

            // N.B. Arm will be in error state due to abortion of ExecutePlan
            //      The error state is cleared in the clear_error() function

            // Stop the navigation
            if (!cancelNavigationGoal())
                    machineError("Navigation cancel failed");
    
            break;


        case States::ABORTED:  // The machine is aborted
            result_ = Result::ABORTED;
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

    // Save parameters
    robot_frame_ = this->get_parameter("robot_tag_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    human_frame_ = this->get_parameter("human_tag_frame").as_string();
    follow_human_ = this->get_parameter("follow_human").as_bool();
    goal_update_topic_ = this->get_parameter("goal_update_topic").as_string();
    sleep_time_ = this->get_parameter("sleep_time").as_int();

    // Initialize the tf buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create the action server
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<StateMachineAction>(
      this,
      "state_machine_server",
      std::bind(&StateMachine::handle_goal, this, _1, _2),
      std::bind(&StateMachine::handle_cancel, this, _1),
      std::bind(&StateMachine::handle_accepted, this, _1));

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
    nav_goal_updater_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_update_topic_, 10);

}


void StateMachine::spinMachine(const std::shared_ptr<GoalHandleStateMachine> goal_handle) {
    // Define feedback message and result
    auto feedback = std::make_shared<simulation_interfaces::action::StateMachine::Feedback>();
    uint8_t &status = feedback->current_state;
    auto result = std::make_shared<simulation_interfaces::action::StateMachine::Result>();
    status = static_cast<uint8_t>(state_);
    goal_handle->publish_feedback(feedback);
    States last_status = state_;
    // Main loop
    while (rclcpp::ok()) {
        if (result_ != Result::RUNNING && result_ != Result::ABORTING && result_ != Result::INITIALIZED) {
            break;
        }
        if (last_status != state_) {
            last_status = state_;
            RCLCPP_INFO(this->get_logger(), "Status: %s", state_to_string(state_).c_str());
        }
        // Check if the goal has been canceled

        nextState();

        // Update the feedback message
        status = static_cast<uint8_t>(state_);
        goal_handle->publish_feedback(feedback);
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_));
    }

    if (requestedAbort_) {
        goal_handle->canceled(result);
    } else if (result_ == Result::SUCCESS) {
        goal_handle->succeed(result);
    } else {
        goal_handle->abort(result);
    }

}  

bool StateMachine::clear_error() {
    // Clear the error message and set the machine in the IDLE state
    if (state_ == States::ERROR || state_ == States::ABORTED) {
        errorMsg_ = "";
        requestedNavigation_ = false;
        requestedInteraction_ = false;
        requestedGripperMovement_ = GripperState::UNKNOWN;
        // If an aborted request has been made, Moveit throws an error but the machine is not in error
        if (requestedAbort_){
            requestedAbort_ = false;
            ResetArmStatus();
        }
        RCLCPP_INFO(this->get_logger(), "Error cleared. Machine in IDLE state");
        state_ = States::IDLE;
        result_ = Result::INITIALIZED;
        return true;
    }
    return false;
}


bool StateMachine::lookup_tf(const std::string &to_frame, const std::string &from_frame) const {
    // Check if the requested TF is available
    try {
        // TODO: Perform the lookupTransform with a timeout to ensure the TF is available
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                                                from_frame, to_frame,
                                                tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "TF %s -> %s not available", from_frame.c_str(), to_frame.c_str());
        return false;
    }
    return true;
}

bool StateMachine::tf_available() const{
    // Check if the TF of the human and robot are available
    return lookup_tf(human_frame_, map_frame_) && lookup_tf(robot_frame_, map_frame_);
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
    human_tf = tf_buffer_->lookupTransform(map_frame_, human_frame_, tf2::TimePointZero);
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
        case States::ABORTING:
            return "ABORTING";
        case States::ABORTED:
            return "ABORTED";
        default:
            return "UNKNOWN";
    }
}