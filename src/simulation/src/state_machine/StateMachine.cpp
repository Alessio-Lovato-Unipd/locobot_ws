/**
 * @file StateMachine.cpp
 * 
 * @brief This file contains the implementation of the StateMachine class.
 */

#include "state_machine/StateMachine.h"


/***********************************************************************************************/
/***********************************************************************************************/
/***********************************   LIFECYCLE CALLBACKS   ***********************************/
/***********************************************************************************************/
/***********************************************************************************************/


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn StateMachine::on_configure(
        const rclcpp_lifecycle::State & state) {
    
    RCLCPP_INFO(this->get_logger(), "Configuring State Machine");

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
    param_desc.description = "Name of the locobot controller node";
    this->declare_parameter("controller_name", "locobot_controller", param_desc);
    param_desc.description = "Name of the locobot controller namespace";
    this->declare_parameter("controller_ns", "", param_desc);
    param_desc.description = "Name of the navigation server";
    this->declare_parameter("navigation_server", "navigate_to_pose", param_desc);
    param_desc.description = "Name of the arm interface";
    this->declare_parameter("arm_interface", "interbotix_arm", param_desc);
    param_desc.description = "Name of the gripper interface";
    this->declare_parameter("gripper_interface", "interbotix_gripper", param_desc);
    param_desc.description = "Timeout for the navigation action server in seconds";
    this->declare_parameter("timeout", 2.0, param_desc);
    param_desc.description = "Name of the topic to publish the state info";
    this->declare_parameter("info_topic", "state_info", param_desc);

    // Save parameters
    robot_frame_ = this->get_parameter("robot_tag_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    human_frame_ = this->get_parameter("human_tag_frame").as_string();
    follow_human_ = this->get_parameter("follow_human").as_bool();
    sleep_time_ = this->get_parameter("sleep_time").as_int();

    // Check if the parameters are valid
    if (robot_frame_ == "" || map_frame_ == "" || human_frame_ == "") {
        RCLCPP_ERROR(this->get_logger(), "Robot frame, Human frame or map frame was set to an empty string");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    if (sleep_time_ <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Sleep time must be greater than 0");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    if (this->get_parameter("goal_update_topic").as_string() == "") {
        RCLCPP_ERROR(this->get_logger(), "Goal update topic was set to an empty string");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    if (this->get_parameter("info_topic").as_string() == "") {
        RCLCPP_ERROR(this->get_logger(), "Info topic was set to an empty string");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
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

    // Create the info publisher
    info_publisher_ = this->create_publisher<std_msgs::msg::String>(this->get_parameter("info_topic").as_string(), 10);

    // Create the locobot controller
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"navigation_server", this->get_parameter("navigation_server").as_string()},
        {"arm_interface", this->get_parameter("arm_interface").as_string()},
        {"gripper_interface", this->get_parameter("gripper_interface").as_string()},
        {"timeout", this->get_parameter("timeout").as_double()}
    });
    locobot_ = std::make_shared<LocobotControl>(this->get_parameter("controller_name").as_string(),
                                                this->get_parameter("controller_ns").as_string(),
                                                options);

    // Check if the navigation server is available
    bool server_available = locobot_->isNavigationServerAvailable();
    if (!server_available) { // Try again in 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));
        server_available = locobot_->isNavigationServerAvailable();
    }
    if (!server_available) {
        RCLCPP_ERROR(this->get_logger(), "Navigation server not available");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // Create the threads for locobot and main loop
    main_thread_ = std::thread(&StateMachine::mainLoop, this);
    locobot_thread_ = std::thread(&StateMachine::locobotLoop, this, std::ref(locobot_));

    RCLCPP_INFO(this->get_logger(), "State Machine configured");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn StateMachine::on_activate(
        const rclcpp_lifecycle::State & state) {
    
    RCLCPP_INFO(this->get_logger(), "Activating State Machine");

    // Activate the publishers
    nav_goal_updater_->on_activate();
    info_publisher_->on_activate();

    // Initialize the state machine
    state_ = States::IDLE;
    requestedAbort_ = false;
    requestedNavigation_ = false;
    requestedInteraction_ = false;
    requestedGripperMovement_ = GripperState::UNKNOWN;
    errorMsg_ = "";

    // Launch locobot controller
    pause_locobot_thread_ = false;

    // Launch the main thread of the state machine
    pause_main_thread_ = false;

    RCLCPP_INFO(this->get_logger(), "State Machine activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn StateMachine::on_deactivate(
        const rclcpp_lifecycle::State & state) {
    
    RCLCPP_INFO(this->get_logger(), "Deactivating State Machine");

    // Stop Arm and Navigation
    locobot_->cancelNavigationGoal();
    locobot_->StopArm();

    // Pause the main thread
    std::unique_lock<std::mutex> lock(main_thread_mutex_);
    pause_main_thread_ = true;
    lock.unlock();

    // Pause the locobot thread
    std::unique_lock<std::mutex> lock2(locobot_thread_mutex_);
    pause_locobot_thread_ = true;
    lock2.unlock();

    // Deactivate the publishers
    nav_goal_updater_->on_deactivate();
    info_publisher_->on_deactivate();

    RCLCPP_INFO(this->get_logger(), "State Machine deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn StateMachine::on_error(
        const rclcpp_lifecycle::State & state) {
    
    RCLCPP_ERROR(this->get_logger(), "Error in State Machine: %s", errorMsg_.c_str());

    // Stop the threads and join them
    state_ = States::ERROR;
    locobot_thread_.join();
    main_thread_.join();
        
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn StateMachine::on_shutdown(
        const rclcpp_lifecycle::State & state) {

    RCLCPP_INFO(this->get_logger(), "Shutting down State Machine");

    if (locobot_->isArmMoving()) {
        locobot_->StopArm();
    }
    if (locobot_->isNavigating()) {
        locobot_->cancelNavigationGoal();
    }
    while (locobot_->isArmMoving() || locobot_->isNavigating()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    state_ = States::COMPLETED;

    // Stop the threads and join them
    locobot_thread_.join();

    // Stop the main thread
    main_thread_.join();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


/***********************************************************************************************/
/***********************************************************************************************/
/*********************************   STATE MACHINE FUNCTIONS   *********************************/
/***********************************************************************************************/
/***********************************************************************************************/


void StateMachine::machineError(const std::string &msg) {
    errorMsg_ = msg;
    locobot_->StopArm();
    locobot_->cancelNavigationGoal();
    while (locobot_->isArmMoving() || locobot_->isNavigating()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    state_ = States::ERROR;
    this->deactivate();
}


void StateMachine::nextState() {

    // Check if Tf of both human and robot are available
    if (!tf_available()) {
        machineError("TF not available");
    }

    switch (state_) {
        case States::IDLE: { // Initial state, wait for a new command
            std::unique_lock<std::mutex> lock(request_mutex_);
            if (requestedNavigation_) {
                lock.unlock();
                state_ = States::SECURE_ARM;
            } else if (requestedInteraction_) {
                lock.unlock();
                locobot_->SetArmPose(ArmPose::HOME);
                state_ = States::WAIT_ARM_EXTENDING;
            }
            break;
        }

        case States::SECURE_ARM: { // Send the arm to the secure position
            if (locobot_->ArmCurrentPose() != ArmPose::SLEEP) {
                locobot_->SetArmPose(ArmPose::SLEEP);
                state_ = States::WAIT_ARM_SECURING;
            } else {
                state_ = States::SEND_NAV_GOAL;
            }
            break;
        }

        case States::WAIT_ARM_SECURING: { // Wait for the arm to be secured
            if (!locobot_->isArmMoving()) {
                state_ = States::SEND_NAV_GOAL;
                if (locobot_->isArmInError())
                    machineError("Arm securing failed");
            }
            break;
        }

        case States::SEND_NAV_GOAL: { // Send the navigation goal to the navigation stack
            last_human_pose_ = GetHumanPose();
            locobot_->MoveBaseTo(last_human_pose_);
            state_ = States::WAIT_NAVIGATION;
            break;
        }

        case States::WAIT_NAVIGATION: { // Wait for the navigation to reach the goal
            // Check if the human has moved
            if (follow_human_)
                nav_goal_updater_->publish(GetHumanPose());

            // Check if the navigation has completed
            std::unique_lock<std::mutex> lock(request_mutex_);
            if (!requestedNavigation_) { // Human has stopped navigation    
                lock.unlock();
                if(locobot_->isNavigating()) {
                    locobot_->cancelNavigationGoal();
                }
                lock.lock();
                if (requestedInteraction_) { // Human has requested interaction
                    lock.unlock();
                    locobot_->SetArmPose(ArmPose::HOME);
                    state_ = States::WAIT_ARM_EXTENDING;
                } else {
                    lock.unlock();
                    state_ = States::IDLE;
                }
            } else if (!locobot_->isNavigating()) { // Goal has been reached but human is still moving
                state_ = States::SEND_NAV_GOAL;  
            }
            if (locobot_->isNavigationInError()) { // Navigation has failed
                machineError("Navigation failed");
            }
            break;
        }

        case States::WAIT_ARM_EXTENDING: { // Wait for the arm to be extended
            if (!locobot_->isArmMoving()) {
                state_ = States::ARM_EXTENDED;
                if (locobot_->isArmInError())
                    machineError("Arm extending failed");
            }
            break;
        }

        case States::ARM_EXTENDED: { // Open the gripper to release the object upon command
            std::unique_lock<std::mutex> lock(request_mutex_);
            if (requestedGripperMovement_ != locobot_->GripperCurrentState() 
                        && requestedGripperMovement_ != GripperState::UNKNOWN) {
                lock.unlock();
                locobot_->SetGripper(requestedGripperMovement_);
                requestedGripperMovement_ = GripperState::UNKNOWN;
                state_ = States::WAIT_GRIPPER;
            } else if (!requestedInteraction_) {
                lock.unlock();
                locobot_->SetArmPose(ArmPose::SLEEP);
                state_ = States::WAIT_ARM_RETRACTING;
            }
            break;
        }

        case States::WAIT_GRIPPER: { // Wait for the gripper to open
            if (!locobot_->isArmMoving()) {
                state_ = States::ARM_EXTENDED;
                if (locobot_->isArmInError())
                    machineError("Gripper movement failed");
            }
            break;
        }

        case States::WAIT_ARM_RETRACTING: { // Wait for the arm to retract
            if (!locobot_->isArmMoving()) {
                state_ = States::IDLE;
                if (locobot_->isArmInError())
                    machineError("Arm retracting failed");
            }
            break;
        }

        case States::ERROR: { // The machine is in error
            locobot_->StopArm();
            locobot_->cancelNavigationGoal();
            state_ = States::COMPLETED;
            break;
        }

        case States::COMPLETED: { // The machine has completed the task
            break;
        }
    }
}

void StateMachine::mainLoop() {
    rclcpp::Rate rate(1000/sleep_time_); // 1000 Hz / sleep_time_ ms
    States last_status = state_;
    while (rclcpp::ok() && state_ != States::ERROR && state_ != States::COMPLETED) {
        std::unique_lock<std::mutex> lock(main_thread_mutex_);
        if (pause_main_thread_) { // Wait for the main thread to be activated
            lock.unlock();
        } else { // Advance the state machine
            lock.unlock();
            nextState();
        }
        // Print the changes in the state
        if (last_status != state_) {
            last_status = state_;
            RCLCPP_INFO(this->get_logger(), "Status: %s", state_to_string(state_).c_str());
        }
        // Publish the state info
        if (info_publisher_->is_activated()) {
            std_msgs::msg::String msg;
            msg.data = state_to_string(state_);
            info_publisher_->publish(msg);
        }
        rate.sleep();
    }
}

void StateMachine::locobotLoop(std::shared_ptr<LocobotControl> locobot) {
    while (rclcpp::ok() && state_ != States::ERROR && state_ != States::COMPLETED) {
        std::unique_lock<std::mutex> lock(locobot_thread_mutex_);
        if (pause_locobot_thread_) { // Pause the locobot thread
            lock.unlock();
            // Wait for the locobot thread to be activated
        } else {
            lock.unlock();
            rclcpp::spin_some(locobot);
        }
    }
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
        case States::COMPLETED:
            return "COMPLETED";
        default:
            return "UNKNOWN";
    }
}