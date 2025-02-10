#include "LocobotControl.h"

// NODE

LocobotControl::LocobotControl(const std::string name, const std::string ns, const rclcpp::NodeOptions &options,
                                const ArmPose arm_pose)
: Node(name, ns, options) {

    // Get parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Name of the navigation server";
    this->declare_parameter("navigation_server", "navigate_to_pose", param_desc);
    param_desc.description = "Name of the arm interface";
    this->declare_parameter("arm_interface", "interbotix_arm", param_desc);
    param_desc.description = "Name of the gripper interface";
    this->declare_parameter("gripper_interface", "interbotix_gripper", param_desc);
    param_desc.description = "Timeout for the navigation action server in seconds";
    this->declare_parameter("timeout", 2.0, param_desc);

    navigation_server_name_ = this->get_parameter("navigation_server").as_string();
    arm_interface_name_ = this->get_parameter("arm_interface").as_string();
    gripper_interface_name_ = this->get_parameter("gripper_interface").as_string();
    timeout_ = this->get_parameter("timeout").as_double();


    if (navigation_server_name_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Navigation server name is empty");
        rclcpp::shutdown();
        return;
    } else if (arm_interface_name_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Arm interface name is empty");
        rclcpp::shutdown();
        return;
    } else if (gripper_interface_name_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Gripper interface name is empty");
        rclcpp::shutdown();
        return;
    } else if (timeout_ <= 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Timeout must be greater than 0");
        rclcpp::shutdown();
        return;
    }

    this->nav_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    navigation_server_name_);
    arm_status_ = ArmStatus(arm_pose);
}


// BASE

void LocobotControl::MoveBaseTo(const geometry_msgs::msg::PoseStamped &pose,
                                std::optional<double> timeout) {

    if (!this->nav_client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    // Wait for the action server to be available
    std::chrono::seconds timeout_duration = std::chrono::seconds(static_cast<long>(timeout.value_or(timeout_)));
    if (!nav_client_ptr_->wait_for_action_server(timeout_duration)) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal position");
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&LocobotControl::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&LocobotControl::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&LocobotControl::result_callback, this, std::placeholders::_1);

    NavigateToPose::Goal goal;
    goal.pose = pose;
    navigation_status_.startNavigation();
    auto goal_handle_future = nav_client_ptr_->async_send_goal(goal, send_goal_options);
}



void LocobotControl::goal_response_callback(GoalHandle::SharedPtr goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        navigation_status_.stopNavigation(rclcpp_action::ResultCode::ABORTED);
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}



void LocobotControl::feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    // Update the navigation status
    rclcpp::Duration estimated_time_remaining{feedback->estimated_time_remaining};
    double seconds = estimated_time_remaining.seconds();
    navigation_status_.updateStatus(feedback->distance_remaining, seconds);
    //RCLCPP_INFO(this->get_logger(), "Remaining distance: %f m", feedback->distance_remaining);
}



void LocobotControl::result_callback(const GoalHandle::WrappedResult &result) {
    navigation_status_.stopNavigation(result.code);
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }

}



bool LocobotControl::cancelNavigationGoal(const bool async_cancellation) {
    auto future = nav_client_ptr_->async_cancel_all_goals();
    // If asynchronous wait for the response
    if (async_cancellation) {
        auto result = future.get();

        if (result->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
            return true;
        } else {
            return false;
        }
    }
    return true;
}




// ARM

bool LocobotControl::SetArmPose(const ArmPose pose, std::optional<std::string> interface_name) {
    // Verify the pose is not unknown
    if (pose == ArmPose::UNKNOWN) {
        RCLCPP_ERROR(get_logger(), "It is not possible to move the arm to an unknown pose");
        return false;
    }

    // Check if the arm is already in the target pose
    if (pose == arm_status_.get_arm_pose()) {
        RCLCPP_INFO(get_logger(), "The arm is already in the target pose");
        return true;
    }

    // Check if the pose can be converted to a string
    std::string target = ArmPose_to_string(pose);
    if (target.empty()) {
        RCLCPP_ERROR(get_logger(), "The target pose is not defined in function ArmPose_to_string()");
        return false;
    }  

    // Publish info message
    RCLCPP_INFO(get_logger(), "Planning arm movement to %s", ArmPose_to_string(pose).c_str());

    // Execute the plan in another thread
    arm_status_.in_motion(true);
    std::thread execution(&LocobotControl::ExecutePlan, this,
                          interface_name.value_or(arm_interface_name_), PoseOrState(pose));
    execution.detach(); // Detach the thread
    return true;
}



bool LocobotControl::ExecutePlan(const std::string interface_name, const PoseOrState target) {
    // Ensure the ROS2 context is valid
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "ROS2 context is not valid");
        return false;
    }

    // Create the MoveGroupInterface
    auto move_group_interface = MoveGroupInterface(shared_from_this(), interface_name);

    // Set the target
    if (std::holds_alternative<ArmPose>(target)) {
        ArmPose pose = std::get<ArmPose>(target);
        if (pose == ArmPose::UNKNOWN) {
            RCLCPP_ERROR(get_logger(), "Unknown arm target");
            return false;
        }
        move_group_interface.setNamedTarget(ArmPose_to_string(pose));
    } else if (std::holds_alternative<GripperState>(target)) {
        GripperState state = std::get<GripperState>(target);
        if (state == GripperState::UNKNOWN) {
            RCLCPP_ERROR(get_logger(), "Unknown gripper target");
            return false;
        }
        move_group_interface.setNamedTarget(GripperState_to_string(state));
    } else {
        RCLCPP_ERROR(get_logger(), "Unknown target type");
        return false;
    }

    // Create a plan
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        // Execute the plan
        if (move_group_interface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            // Update the arm status
            if (std::holds_alternative<ArmPose>(target)) {
                ArmPose next_pose = std::get<ArmPose>(target);
                arm_status_.updateStatus(next_pose, arm_status_.get_gripper_state());
            } else if (std::holds_alternative<GripperState>(target)) {
                GripperState next_state = std::get<GripperState>(target);
                arm_status_.updateStatus(arm_status_.get_arm_pose(), next_state);
            }
            arm_status_.in_motion(false);
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "Execution failed!");
        }
    } else {
        RCLCPP_ERROR(get_logger(), "Planning failed!");
    }
    arm_status_.in_motion(false);
    arm_status_.errorState(true);
    return false;
}




void LocobotControl::StopArm() {
    // Send a stop command to the arm and gripper

    /**
     * NOTE: The following code is commented out because the stop action has unexpected behavior in the
     * actual robot. When the stop is called, the motors of the arm goes to an unsafe position rather than
     * stopping in the current position. This is not the expected behavior and it is not clear why this
     * happens. This behavior is not present in the simulation.
     */
    
    // auto move_group_interface = MoveGroupInterface(shared_from_this(), arm_interface_name_);
    // move_group_interface.stop();
    // move_group_interface = MoveGroupInterface(shared_from_this(), gripper_interface_name_);
    //  move_group_interface.stop();

}




std::string LocobotControl::ArmPose_to_string(const ArmPose pose) {
    switch (pose) {
        case ArmPose::HOME:
            return "Home";
        case ArmPose::SLEEP:
            return "Sleep";
        case ArmPose::UPRIGHT:
            return "Upright";
        default:
            return "";
    }
}




// GRIPPER

bool LocobotControl::SetGripper(const GripperState state, std::optional<std::string> interface_name) {
    // Verify the state is not unknown
    if (state == GripperState::UNKNOWN) {
        RCLCPP_ERROR(get_logger(), "It is not possible to move the gripper to an unknown state");
        return false;
    } 

    // Check if the arm is already in the target state
    if (state == arm_status_.get_gripper_state()) {
        RCLCPP_INFO(get_logger(), "The gripper is already in the target state");
        return true;
    }

    // Check if the state can be converted to a string
    std::string target = GripperState_to_string(state);
    if (target.empty()) {
        RCLCPP_ERROR(get_logger(), "The target state is not defined in function ArmState_to_string()");
        return false;
    }  

    // Publish info message
    RCLCPP_INFO(get_logger(), "Planning arm movement to %s", GripperState_to_string(state).c_str());

    // Execute the plan in another thread
    arm_status_.in_motion(true);
    std::thread execution(&LocobotControl::ExecutePlan, this,
                          interface_name.value_or(gripper_interface_name_), PoseOrState(state));
    execution.detach(); // Detach the thread
    return true;
}




std::string LocobotControl::GripperState_to_string(const GripperState state) {
    switch (state) {
        case GripperState::HOME:
            return "Home";
        case GripperState::RELEASED:
            return "Released";
        case GripperState::GRASPING:
            return "Grasping";
        default:
            return "";
    }
}



// General functions

geometry_msgs::msg::PoseStamped coordinates_to_pose(double x, double y, double z, double qx,
                            double qy, double qz, double qw, std::string frame_id) {
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pose.pose.orientation.w = qw;
    return pose;
}