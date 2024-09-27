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
    } else if (requestedAbort_ && state_!= States::ABORTED) { // Check if an abort has been requested
        state_ = States::ABORTING;
    }

    // Switch statement to determine the next state
    switch (state_) {
        case States::IDLE:  // Initial state, wait for the human to be further than 1m from the robot
            if (distance_to_human() > 1.0) {
                state_ = States::SECURE_ARM;
                result_ =  Result::RUNNING;
            }
            break;


        case States::SECURE_ARM:  // Send the arm to the secure position
            SetArmPose(ArmPose::SLEEP);
            state_ = States::WAIT_ARM_SECURING;
            break;


        case States::WAIT_ARM_SECURING:  // Wait for the arm to be secured
            if (!isArmMoving()) {
                state_ = States::COMPLETED;
            } else if (isArmInError()) {
                machineError("Arm securing failed");
            }
            break;

        case States::COMPLETED:  // The task has been completed
            //state_ = States::IDLE;
            result_ =  Result::SUCCESS;
            break;


        case States::ERROR:  // The machine is in error
            //StopArm(); // Stop the arm movement
            result_ =  Result::FAILURE;
            break;


        case States::ABORTING:  // The machine is aborting
            result_ =  Result::ABORTING;
            StopArm(); // Stop the arm movement
            state_ = States::ABORTED;
            break;

        case States::ABORTED:  // The machine is aborted
            result_ = Result::ABORTED;
            break;
    }

    // Spin the robot control object
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


    // Save parameters
    robot_frame_ = this->get_parameter("robot_tag_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    human_frame_ = this->get_parameter("human_tag_frame").as_string();

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

}


void StateMachine::spinMachine(const std::shared_ptr<GoalHandleStateMachine> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Inizio spinMachine");
    // Define feedback message and result
    auto feedback = std::make_shared<simulation_interfaces::action::StateMachine::Feedback>();
    uint8_t &status = feedback->current_state;
    auto result = std::make_shared<simulation_interfaces::action::StateMachine::Result>();
    status = static_cast<uint8_t>(state_);
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Inizio loop");
    // Main loop
    while (rclcpp::ok()) {
        if (result_ != Result::RUNNING && result_ != Result::ABORTING && result_ != Result::INITIALIZED) {
            break;
        }
        RCLCPP_INFO(this->get_logger(), "Status: %s", state_to_string(state_).c_str());
        // Check if the goal has been canceled

        nextState();

        // Update the feedback message
        status = static_cast<uint8_t>(state_);
        goal_handle->publish_feedback(feedback);
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
        state_ = States::IDLE;
        result_ = Result::INITIALIZED;
        errorMsg_ = "";
        // If an aborted request has benn made, Moveit throws an error but the machine is not in error
        if (requestedAbort_){
            requestedAbort_ = false;
            ResetArmStatus();
        }
        RCLCPP_INFO(this->get_logger(), "Error cleared. Machine in IDLE state");
        return true;
    }
    return false;
}


bool StateMachine::lookup_tf(const string &to_frame, const string &from_frame) {
    // Check if the requested TF is available
    try {
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                                                from_frame, to_frame,
                                                tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(this->get_logger(), "TF %s -> %s not available", from_frame.c_str(), to_frame.c_str());
        return false;
    }
    return true;
}

bool StateMachine::tf_available() {
    // Check if the TF of the human and robot are available
    return lookup_tf(human_frame_, map_frame_) && lookup_tf(robot_frame_, map_frame_);
}

double StateMachine::distance_to_human() {  
    try {
        // Get pose of the human and robot
        geometry_msgs::msg::TransformStamped human_tf, robot_tf;
        human_tf = tf_buffer_->lookupTransform(human_frame_, map_frame_,  tf2::TimePointZero);
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


std::string StateMachine::state_to_string(const States state) const {
    // Return the string representation of the States enum
    switch (state) {
        case States::IDLE:
            return "IDLE";
        case States::SECURE_ARM:
            return "SECURE_ARM";
        case States::WAIT_ARM_SECURING:
            return "WAIT_ARM_SECURING";
        case States::COMPLETED:
            return "COMPLETED";
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