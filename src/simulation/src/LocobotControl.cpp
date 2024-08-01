#include "LocobotControl.h"

// NODE

LocobotControl::LocobotControl(const string name, const string ns, const rclcpp::NodeOptions &options)
: Node(name, ns, options) {
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_graph_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "navigate_to_pose"); // Action server name
}


bool LocobotControl::ExecutePlan(moveit::planning_interface::MoveGroupInterface &move_group_interface) {
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (static_cast<bool>(move_group_interface.plan(plan))) {
        // Execute the plan
        if (move_group_interface.execute(plan)) {
            return true;
        } else {
            RCLCPP_ERROR(get_logger(), "Execution failed!");
            return false;
        }
    } else {
        RCLCPP_ERROR(get_logger(), "Planning failed!");
        return false;
    }
}

// BASE

void LocobotControl::MoveBaseTo(const geometry_msgs::msg::PoseStamped &pose, const uint timeout) {

    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

    // Wait for the action server to be available
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(timeout))) {
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
    auto goal_handle_future = client_ptr_->async_send_goal(goal, send_goal_options);
}


void LocobotControl::goal_response_callback(GoalHandle::SharedPtr goal_handle) {
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        rclcpp::shutdown();
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}


void LocobotControl::feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Remaining distance: %f m", feedback->distance_remaining);
}


void LocobotControl::result_callback(const GoalHandle::WrappedResult &result) {
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

// ARM

bool LocobotControl::SetArmPose(const ArmPose pose) {
 
    // Set a target pose
    auto move_group_interface = MoveGroupInterface(shared_from_this(),"interbotix_arm");
    string target = ArmPose_to_string(pose);
    if (target.empty()) {
        RCLCPP_ERROR(get_logger(), "The target pose is not defined in function ArmPose_to_string()");
        return false;
    }  // The pose is not defined in the switch case in ArmPose_to_string
    move_group_interface.setNamedTarget(target);

    // Create a plan to that target pose
    RCLCPP_INFO(get_logger(), "Planning arm movement to %s", ArmPose_to_string(pose).c_str());
    return ExecutePlan(move_group_interface);
}


string LocobotControl::ArmPose_to_string(const ArmPose pose) {
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

bool LocobotControl::SetGripper(const GripperState state) {
    // Set a target pose
    auto move_group_interface = MoveGroupInterface(shared_from_this(),"interbotix_gripper");
    string target = GripperState_to_string(state);
    if (target.empty()) {
        RCLCPP_ERROR(get_logger(), "The target pose is not defined in function GripperState_to_string()");
        return false;
    }
    move_group_interface.setNamedTarget(target);

    // Create a plan to that target pose
    RCLCPP_INFO(get_logger(), "Planning gripper movement to %s", GripperState_to_string(state).c_str());
    return ExecutePlan(move_group_interface);
}


string LocobotControl::GripperState_to_string(const GripperState state) {
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