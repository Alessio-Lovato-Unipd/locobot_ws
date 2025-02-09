# Locobot Simulation Package

## Overview

This package contains the code to run the LoCoBot WX200 both in simulation and real world. It includes functionalities for controlling the Locobot arm, gripper, and base using ROS 2, MoveIt2, and Nav2.

## Directory Structure

The package is organized into the following directories:

- `behavior_tree/`: Contains behavior tree XML files for navigation.
- `config/`: Contains configuration files for various components.
- `include/`: Contains header files for the package.
- `launch/`: Contains launch files for bringing up required nodes.
- `maps/`: Provides example maps to be loaded in Nav2. See launch files for usage.
- `media/`: Contains media files for Gazebo Classic.
- `models/`: Contains sdf models used in the package.
- `rviz/`: Contains RViz configuration files.
- `src/`: Contains source code files.
- `urdf/`: Contains URDF files for robot description.
- `worlds/`: Contains simulation world files for Gazebo Classic.

## Files and Folders

### config/

- `camera_calibration\`: Folder containing camera calibration files for the kinect network.
- `apriltag.yaml`: Configuration file for AprilTag detection.
- `navigation_mmpi.yaml`: Configuration file for the MPPI navigation controller.
- `navigation_rpp.yaml`: Configuration file for the RPP navigation controller.
- `rs_camera.yaml`: Configuration file for the Realsense camera.

### include/

- `state_machine/StateMachine.h`: Header file for the `StateMachine` class, which controls the different states of the robot behavior.
- `LocobotControl.h`: Header file for the `LocobotControl` class, defining its methods and member variables.

### launch/

- `camera_calibration\calib_master.launch.py`: Launch file for calibrating the camera network.
- `camera_calibration\k01calib.launch.py`: Launch file to open the camera 'k01' for calibration.
- `camera_calibration\k02calib.launch.py`: Launch file to open the camera 'k02' for calibration.
- `camera_calibration\k4a.launch.py`: Kinetic Azure default camera calibration launch file.
- `arm_to_sleep.launch.py`: Launch file for sending the robot's arm to the sleep position.
- `demo.launch.py`: Launch file for running the demo program.
- `include_camera.launch.py`: Launch file for including the camera in the simulation.
- `include_markers.launch.py`: Launch file for including the AprilTag markers in the simulation.
- `remote.launch.py`: Launch file for running Rviz2 interface, apriltag detection and kinect connection on a remote machine (not the NUC).
- `robot.launch.py`: Launch file to bring up MoveIt2, Nav2, and the Locobot control node on the NUC, as well as the realsense camera.
- `second_camera.launch.py`: Launch file to include marker detection from a second kinect camera in the camera network.
- `simulation.launch.py`: Launch file for running the simulation.
- `state_machine.launch.py`: Launch file for running the state machine node.


> **Note:** The `camera_calibration` folder contains launch files for calibrating the camera network using the `camera_calibration` package.

> **Note:** Read the launch file sections for more information on the arguments required for each launch file.

### src/

- `state_machine/HumanEmulator.cpp`: Implements the `HumanEmulator` class, a ROS 2 node that emulates human behavior for simulation purposes.
- `state_machine/StateMachine.cpp`: Implements the `StateMachine` class, responsible for controlling the different states of the robot behavior.
- `ArmSleepPosition.cpp`: A demo file for the `LocobotControl` class that sends the robot's arm to the sleep position.
- `Demo.cpp`: A demo file for the `LocobotControl` class that demonstrates moving the base, arm, and gripper to specified positions.
- `LocobotControl.cpp`: Implements the `LocobotControl` class, responsible for controlling the Locobot arm, gripper, and base.


## Classes

### LocobotControl Class

The `LocobotControl` class is a ROS 2 node responsible for controlling the Locobot arm, gripper, and base. It provides methods to move the Locobot arm and gripper to specified poses using MoveIt2 and to move the Locobot base to specified points using the Navigation2 stack.

#### Parameters

- `navigation_server`: Name of the navigation action server (default: `navigate_to_pose`).
- `arm_interface`: Name of the MoveIt arm interface (default: `interbotix_arm`).
- `gripper_interface`: Name of the MoveIt gripper interface (default: `interbotix_gripper`).
- `timeout`: Timeout for the navigation action server in seconds (default: `2.0`).

#### Key Components

- **Enums**
  - `ArmPose`: Represents predefined arm poses (HOME, SLEEP, UPRIGHT, UNKNOWN).
  - `GripperState`: Represents predefined gripper states (HOME, RELEASED, GRASPING, UNKNOWN).

- **Classes**
  - `ArmStatus`: Stores the status of the arm, including its pose, gripper state, motion status, and error status.
  - `NavigationStatus`: Stores the status of the navigation, including remaining distance, ETA, progress status, and error status.

#### Main Functions

- **Constructor**
  - Initializes the node, declares parameters, and sets up the action client for navigation.

- **MoveBaseTo**
  - Sends a goal to the navigation stack to move the robot to a specified pose.

- **SetArmPose**
  - Moves the robot arm to a specified pose.

- **SetGripper**
  - Moves the gripper to a specified state.

- **ExecutePlan**
  - Plans and executes the movement using MoveGroupInterface.

- **StopArm**
  - Stops the arm movement if it is in progress.

- **CancelNavigationGoal**
  - Cancels all goals sent to the navigation server.

- **General Functions**
  - `coordinates_to_pose`: Converts coordinates to a PoseStamped message.

#### Callbacks

- **goal_response_callback**: Handles the response from the action server when a goal is sent.
- **feedback_callback**: Handles feedback from the action server during goal execution.
- **result_callback**: Handles the result from the action server after goal execution.

### StateMachine Class

The `StateMachine` class is a ROS 2 node that controls the different states of the robot behavior. It uses a state machine to manage the robot's actions, such as navigating to a goal, interacting with objects, and handling errors. The state machine is designed to be used by another node to control the robot's behavior via the provided services. It inherits from the `LocobotControl` class to control the robot's arm, gripper, and base.

#### Parameters

- `robot_tag_frame`: Frame of the robot tag (default: `locobot_tag`).
- `map_frame`: Frame of the map tag (default: `map`).
- `human_tag_frame`: Frame of the human tag (default: `human_tag`).
- `follow_human`: Indicates if the human should be followed after the first position (default: `true`).
- `goal_update_topic`: Topic to update the navigation goal (default: `goal_update`).
- `sleep_time`: Time [ms] to sleep between cycles of the state machine (default: `100`).
- `state_topic`: Topic where the state of the state machine is published (default: `machine_state`).
- `debug`: Publish the internal state of the machine (default: `true`).
- `tf_tolerance`: Time tolerance [s] for missing TFs before triggering an error (default: `5.0`).

#### Services

The `StateMachine` node provides several services to control and monitor its behavior:

- **Control States Service**: This service allows changing the state of the machine to IDLE, NAVIGATION, INTERACTION, or ABORT.
  - Service name: `/state_control`
  - Service type: `simulation_interfaces/srv/ControlStates`

- **Clear Error Service**: This service clears the error or aborted state and message error. The machine will enter the IDLE state if possible.
  - Service name: `/clear_error_state`
  - Service type: `simulation_interfaces/srv/ClearError`

- **Last Error Service**: This service returns the last error message and state of the state machine.
  - Service name: `/get_last_error`
  - Service type: `simulation_interfaces/srv/LastError`

#### Internal and External States

The state machine operates with a set of internal and external states to manage the robot's behavior. The internal states represent the current state of the machine, while the external states represent the result of the machine's operation.

**Internal States**

The internal states of the state machine are as follows:

- `IDLE`: The machine is waiting for a new command.
- `SECURE_ARM`: The machine secures the arm to avoid collision during navigation.
- `WAIT_ARM_SECURING`: The machine waits for the arm to be secured.
- `SEND_NAV_GOAL`: The machine sends the navigation goal to the navigation stack.
- `WAIT_NAVIGATION`: The machine waits for the navigation to reach the goal.
- `WAIT_ARM_EXTENDING`: The machine waits for the arm to be extended.
- `ARM_EXTENDED`: The machine opens the gripper to release the object upon command.
- `WAIT_GRIPPER`: The machine waits for the gripper to open.
- `WAIT_ARM_RETRACTING`: The machine waits for the arm to retract.
- `ERROR`: The machine is in an error state.
- `ABORT`: The machine is aborted.
- `STOPPING`: The machine is stopping.

![Internal States](internal_states.svg)

**External States**

The external states of the state machine are as follows:

- `SUCCESS`: The machine has completed the task successfully.
- `FAILURE`: The machine has failed to complete the task.
- `RUNNING`: The machine is still running.
- `INITIALIZED`: The machine has been initialized.

![External States](external_states.svg)

#### Safety Precautions
The state machine start the robot in the ERROR state. This is to prevent the robot from moving unexpectedly. To start the robot, the state must be switched to the IDLE state using the `\clear_error_state` service:

```bash
ros2 service call /clear_error_state simulation_interfaces/srv/ClearError
```
Furthermore, the state machine will automatically switch to the ERROR state if the TF of robot or human is not available for a certain amount of time. This is to prevent the robot from moving when the TF is not available.

## Launch Files

### state_machine.launch.py
This launch file is used to launch the state machine node.

**Input Arguments:**
- `debug`: Flag to publish the internal state of the machine. Can be *true* or *false*. (default: *true*)
- `tf_tolerance`: Time tolerance [s] for missing TFs before triggering an error. (default: *5.0*)
- `human_following`: Flag to indicate if the human should be followed after the first position. Can be *true* or *false*. (default: *true*)

> **Note:** The `state_machine.launch.py` launch file can be modified to include the all the arguments required for the `StateMachine` class.

### simulation.launch.py

This launch file is used to launch a complete simulation of the Locobot using Gazebo Classic. To control the robot behavior, the state machine can be included by setting the `state_machine` argument to *true*.
Otherwise the *LocobotControl* class can be used to control the robot manually or creating another script.
If the `nav2_param_file` is not set, the default file for navigation will be selected basing on the `nav_controller` argument.

**Input Arguments:**
- `nav2_param_file`: The file path to the params YAML file of Nav2. (default: '')
- `external_urdf_loc`: The file path to the custom URDF file to include in the Interbotix robot. (default: *urdf/locobot_tag.urdf.xacro*)
- `camera_number`: Number of cameras to be included in the simulation. Can be *1* or *2*. (default: *1*)
- `spawn_obstacle`: Flag to spawn an obstacle in the simulation. Can be *true* or *false*. (default: *true*)
- `container`: Name of the container where to load the components. Default is *nav2_container*.
- `nav_controller`: The Nav2 controller plugin to use. Can be *mppi* or *rpp*. (default: *mppi*)
- `state_machine`: Flag to include the state machine. Can be *true* or *false*. (default: *false*)

### robot.launch.py

This launch file is used to bring up the required packages in the NUC of the Locobot. To control the robot behavior, the state machine can be included by setting the `state_machine` argument to *true*. Otherwise the *LocobotControl* class can be used to control the robot manually or creating another script.
If the `nav2_param_file` is not set, the default file for navigation will be selected basing on the `nav_controller` argument.

**Input Arguments:**
- `external_urdf_loc`: The file path to the custom URDF file to include in the Interbotix robot. (default: *urdf/locobot_tag.urdf.xacro*)
- `nav2_param_file`: The file path to the params YAML file. (default: ``)
- `rs_camera_param`: The file path to the Realsense camera configuration file. (default: *rs_camera.yaml*)
- `container`: Name of an existing node container to load launched nodes into. If unset, a new container will be created for the apriltag detections.
- `nav_controller`: The controller plugin to be used in Nav2. Can be *mppi* or *rpp*. (default: *mppi*)
- `state_machine`: Flag to include the state machine. Can be *true* or *false*. (default: *false*)

### remote.launch.py

This launch file is used to launch the simulation environment with the Azure Kinect camera and the apriltag node. Not to be used on the NUC but on a remote machine connected to the kinect camera network.

**Input Arguments:**
- `container`: Name of an existing node container to load launched nodes into. If unset, a new container will be created for the apriltag detections.
- `only_camera`: If set to `true`, only the camera and apriltag node will be launched.
- `apriltag_config_file`: Full path to the apriltag configuration file to use.
- `kinect_config_file`: Full path to the kinect configuration file to use.

### second_kinect.launch.py

This launch file is used to launch the apriltag detection node with a second kinect camera in the camera network.

**Input Arguments:**
- `container`: Name of an existing node container to load launched nodes into. If unset, a new container will be created.
- `apriltag_config_file`: Full path to the apriltag configuration file to use.
- `kinect_config_file`: Full path to the kinect configuration file to use.

### include_markers.launch.py

This launch file is used to include the fiducial marker and camera entities in Gazebo simulation. It incorporates the `include_camera.launch.py` launch file.

**Input Arguments:**
- `camera_number`: Number of cameras to be included in the simulation. Can be *1* or *2*. (default: *1*)
- `use_gazebo`: Flag to include the Gazebo Classic simulation. Can be *true* or *false*. (default: *false*)
- `container`: Name of an existing node container to load launched nodes into. If unset, a new container will be created for the apriltag detections.

### include_camera.launch.py

This launch file is used to include the camera entities in Gazebo simulation.

**Input Arguments:**
- `camera_number`: Number of cameras to be included in the simulation. Can be *1* or *2*. (default: *1*)
- `use_gazebo`: Flag to include the Gazebo Classic simulation. Can be *true* or *false*. (default: *false*)

### demo.launch.py

This launch file is used to run a demo program for the Locobot.

**Input Arguments:**
- None

### arm_to_sleep.launch.py

This launch file is used to send the robot's arm to the sleep position.

**Input Arguments:**
- None

## Executables available
The following executables are available in the package, but it is recommended to use the launch files to start the nodes (see *Warning* section below).
- `demo`: A demo program for the Locobot.
- `arm_sleep_position`: Sends the robot's arm to the sleep position.
- `state_machine`: Controls the different states of the robot behavior. Note that the state machine is also registered as a component (*state_machine_component*).

## Warning

### Topic Remapping

To use the `LocobotControl` and `StateMachine` class, it is necessary to remap the topics as shown in the [demo_no_lidar.launch.py](launch/demo_no_lidar.launch.py) file. This is because Interbotix publishes the robot descriptions on specific topics.

### Stop Interface
The stop interface for MoveIt2 in the state machine has been commented out in the code because it exhibits unexpected behavior when used with the real robot. An issue should be opened in the Interbotix package repository to address this problem.

## License

This project is licensed under the Apache-2.0 License. See the `LICENSE` file for details.