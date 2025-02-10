# locobot control interfaces

## Overview

`locobot_control_interfaces` is a ROS 2 package that defines custom service interfaces to be used in the locobot package.

## Package Contents

### Services

- **ClearError.srv**: Service to request the clearing of ERROR or ABORT state from the state machine. It also clears the last error message.
  - **Request**: No fields.
  - **Response**:
    - `bool successful`: Indicates whether the error was successfully cleared.

- **ControlGripper.srv**: Service to request gripper opening or closure.
  - **Request**:
    - `uint8 CLOSE=0`: Constant to request closing the gripper.
    - `uint8 OPEN=1`: Constant to request opening the gripper.
    - `uint8 request`: Field to specify the desired action (CLOSE or OPEN).
  - **Response**:
    - `bool successful_request`: Indicates whether the request was successfully processed.

- **ControlStates.srv**: Service to request state transitions, including gripper control and other states.
  - **Request**:
    - `uint8 IDLE=0`: Constant to request the IDLE state.
    - `uint8 NAVIGATION=1`: Constant to request the NAVIGATION state.
    - `uint8 INTERACTION=2`: Constant to request the INTERACTION state.
    - `uint8 OPEN_GRIPPER=3`: Constant to request opening the gripper.
    - `uint8 CLOSE_GRIPPER=4`: Constant to request closing the gripper.
    - `uint8 ABORT=5`: Constant to request the ABORT state.
    - `uint8 state`: Field to specify the desired state.
  - **Response**:
    - `bool successful_request`: Indicates whether the state transition request was successfully processed.

- **LastError.srv**: Service to request the last error message and the state in which it occurred.
  - **Request**: No fields.
  - **Response**:
    - `string last_state`: The state in which the last error occurred.
    - `string error_message`: The last error message.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.