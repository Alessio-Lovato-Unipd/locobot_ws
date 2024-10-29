# Gesture Recognition Package

This package contains the gesture recognition node that recognizes gestures using the MediaPipe library and sends the corresponding commands to change state to the robot. The package includes a launch file to start the gesture recognition node and optionally launch the RealSense camera.<br>
Documentation of the Mediapipe library is available [here](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer).

## Package Overview

- **gesture_recognition**: The main package containing the gesture recognition node.
- **launch**: Contains the launch file to start the gesture recognition node and optionally launch the RealSense camera.
- **gesture_recognizer_model**: Contains the gesture recognizer model used by the node.

## Launch File

The launch file `gesture_recognizer.launch.py` is used to start the gesture recognition node. It can optionally launch the RealSense camera based on the `simulation` argument.

### Usage
```sh
ros2 launch gesture_recognition gesture_recognizer.launch.py [simulation:=true|false]
```
### Arguments

- `simulation` (default: `false`): Whether to launch the RealSense camera. Possible values are `true` or `false`.

### Example

To launch the gesture recognition node without the RealSense camera:
```sh
ros2 launch gesture_recognition gesture_recognizer.launch.py simulation:=false
```
To launch the gesture recognition node with the RealSense camera:
```sh
ros2 launch gesture_recognition gesture_recognizer.launch.py simulation:=true
```
## Gesture Recognition Node

The gesture recognition node (`gesture_recognizer_node`) recognizes gestures using the MediaPipe library and sends the corresponding state to the robot. The node subscribes to the camera image topic, processes the images to recognize gestures, and sends the recognized gestures as commands to the robot.<br>
>**NOTE**<br>
>The commands are sent once only when a new gesture is detected, with exception for `Closed_Fist` and `Victory`. This is because if those commands are sent in the wrong state, they will be ignored by the state machine.

### Node Details

- **Node Name**: `gesture_recognizer_node`
- **Subscribed Topics**:
  - `/camera/image_raw` (default): The topic of the camera image. This can be changed using the `camera_topic` parameter.
- **Parameters**:
  - `camera_topic` (default: `/camera/image_raw`): The topic of the camera image.
  - `minimum_score` (default: `0.6`): The minimum score to consider a gesture.
  - `service_name` (default: `state_control`): The name of the service to connect with
- **Services**:
  - `state_control`: The service used to send the recognized gestures as commands to the robot. This can be changed using the `service_name` parameter. Must be a `simulation_interfaces::srv::ControlStates` service type.

### Recognized Gestures

The gestures recognized by the node and their corresponding states are:

- `Closed_Fist`: Sets the state to `CLOSE_GRIPPER`.
- `Pointing_up`: Sets the state to `OPEN_GRIPPER`.
- `Thumb_Up`: Sets the state to `NAVIGATION`.
- `Thumb_Down`: Sets the state to `INTERACTION`.
- `Open_Palm`: Sets the state to `IDLE`.
- `ILoveYou`: Sets the state to `ABORT`.

### Node Workflow

1. The node subscribes to the camera image topic.
2. The images are processed using the MediaPipe library to recognize gestures.
3. If a gesture is recognized with a score above the minimum score, the corresponding state is sent to the robot using the `state_control` service.

### Example

To run the gesture recognition node directly:
```sh
ros2 run gesture_recognition gesture_recognizer_node
```
## Installation

1. Clone the repository into your ROS 2 workspace.
2. Build the package using `colcon`:
```sh
colcon build --packages-select gesture_recognition
```
3. Source the workspace:
```sh
source install/setup.bash
```
## Dependencies

- `rclpy`
- `sensor_msgs`
- `cv_bridge`
- `opencv-python`
- `mediapipe`
- `realsense2_camera`
- `simulation_interfaces`

## Maintainer

- Alessio Lovato (alessio.lovato.1@studenti.unipd.it)

## License

This package is licensed under the Apache-2.0 License.