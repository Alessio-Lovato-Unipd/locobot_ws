# LoCoBot Workspace

This repository contains the workspace to control the LoCoBot WX200 with a Kobuki base, an open-source robot developed by [Interbotix](https://www.interbotix.com).<br>
It includes all the necessary ROS packages to operate the robot and is intended to be installed on the onboard NUC with an Ubuntu 22.04 distribution.

> **NOTE**: This package is intended to be used with the CycloneDDS as suggested by Nav2. For this reason the 'image_proc' package is not able to stream out the rectified image and thus the raw image is used.

## Prerequisites

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
- [Python 3](https://www.python.org/downloads/)
- [LoCoBot](http://www.locobot.org/) hardware
- [Gazebo Classic](https://classic.gazebosim.org/tutorials?tut=install_ubuntu)

## Installation

The installation process is divided into two parts: the Locobot and the remote laptop. The Locobot is the onboard computer of the robot, while the remote laptop is the computer that controls the robot. It is recommended to install the workspace on the remote laptop first and then on the Locobot.

> **Note:** The installation process is tested on Ubuntu 22.04 and ROS2 Humble.

> **Note:** ssh will be installed automatically with the installer script.

> **Note:** Before installing, please make sure both the Locobot and the remote laptop are connected to the same network. Furthermore, it is preferable to have a static IP for the Locobot (i.e. 192.168.0.5).

> **Warning:** apriltag_calibration is a private submodule. Please make sure to have the correct permissions to access it before installing the workspace. Also, Make sure that your git configuration is set up correctly in the terminals you are using to clone the repository.

1. Clone the repository:

    ```bash
    git clone https://github.com/AlessioLovato/locobot_ws.git
    # Move to the workspace
    cd locobot_ws/
    ```

2. Use the installer to build your workspace:
    ```bash
    bash installer.sh
    ```
3. Install specific driver:

    - [**Only in the Locobot**] Install udev rules by pasting the following commands in the terminal:
    
        [XArms instructions](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html) -  [Xarms rules file](https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/rpi4/xsarm_rpi4_install.sh) -
        [Kobuki instructions](https://kobuki.readthedocs.io/en/stable/) - [Kobuki rules file](https://github.com/kobuki-base/kobuki_core/blob/release/1.4.x/60-kobuki.rules)

        ```bash
        # X-Arms rules
        sudo cp src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/99-interbotix-udev.rules /etc/udev/rules.d/
        sudo udevadm control --reload-rules && sudo udevadm trigger
        # Kobuki base rules
        sudo cp 60-kobuki.rules /etc/udev/rules.d
        sudo service udev reload
        sudo service udev restart
        ```

    - [**Only in the Remote laptop**] Install the kinect suite:
    
        Go to the [Azure Kinect ROS driver package](src/Azure_Kinect_ROS_Driver), decomment the line 119 in the CMakeLists.txt file and build the package:
        ```bash
        bash install_kinect.sh
        colcon build --packages-select azure_kinect_ros_driver
        ```

4. Source the workspace:

    ```bash
    source install/setup.bash
    ```

5. Install NTP to sync the machines:

    - [**Only in the Locobot**] Create the NTP client:

        ```bash
        sudo apt-get install ntpdate
        sudo ntpdate <LAPTOP_IP> # Replace <LAPTOP_IP> with the IP of the remote laptop
        ```

    - [**Only in the Remote laptop**] Create the NTP server:

        ```
        sudo apt-get install ntp
        sudo ufw allow from any to any port 123 proto udp # Allow NTP traffic
        ```

6. You are ready to go!

## Usage

### Synchronize the locobot and the remote laptop

1. From a terminal in the remote laptop, connect to the Locobot via ssh:
    ```bash
    ssh -X devlab03@<LOCOBOT_IP> # Replace <LOCOBOT_IP> with the IP of the Locobot (i.e. 192.168.0.5)
    ```
2. Sync the time between the Locobot and the remote laptop:
    ```bash
    sudo ntpdate <LAPTOP_IP> # Replace <LAPTOP_IP> with the IP of the remote laptop
    sudo systemctl restart ntp
    ntpq -p # Check the synchronization, a star should appear in the remote laptop IP. If not, try again this command.
    ```

### Calibrate the Kinect cameras
Before using the Kinect cameras, it is necessary to calibrate them. 

> **Note:** Each kinect camera needs to be connected to a different 3.0 USB port. Furthermore, it could be possible that the images are not published if the remote laptop does not have enough power to manage all the cameras.

To do so, follow these steps:

0. Set the kinect cameras and the apriltag markers (family 36h11) on the floor. Make sure the markers are visible from all the cameras and that the cameras are correctly connected to the USB ports. Tag_id 0 is set as the origin of the map, while markers from 1 to 9 (except 3,4) are used to calibrate the cameras.
Coordinate system (right-hand coordinate system) is defined as follows:
    - X: to the right of the marker
    - Y: to the top of the marker
    - Z: exit from the marker
Only the tag_id 0 placement is important, the others can be placed randomly as they're used only to calibrate the cameras position.

1. Make sure to change all the launch file parameters in the calibration folder with the correct values for the kinect serial number and the camera roots.

    > **Note:** The calibration process won't start until all the root frames specified by the calib_master are found. Make sure to have all the kinect launch files publishing before starting the calibration and that the frames are correctly named as in the other launch files of the camera_calibration folder.

2. [**Only if the files have been modified**] <u>**Go to the workspace root folder**</u>, build and source the workspace:
    ```bash
    colcon build --packages-select locobot_control
    source install/setup.bash
    ```

3. Open a terminal for each camera and launch the specific launch file (i.e. k01calib.launch.py, k02calib.launch.py, etc.):
    ```bash
    ros2 launch locobot_control k01calib.launch.py
    ```
    > **Note:** The kinect calibration launch file allows to set the serial number of the kinect camera. See the [README](src/locobot_control/README.md) in the locobot_control package for more information.

4. Open a new terminal and launch the calibration master:
    ```bash
    ros2 launch locobot_control calib_master.launch.py
    ```
5. Open a new terminal <u>in the workspace root folder</u> and launch the calibration script:
    ```bash
    bash calibration.sh
    ```
6. When the calibration is finished, close all the terminals <u>**except the calibration master one**</u>.

### Start the LoCoBot WX200

1. Perform the synchronization between the Locobot and the remote laptop.

2. Perform calibration of the Kinect cameras.

3. Open 4 terminals in the remote laptop (it is suggested to use an app similar to Terminator to manage the terminals).

4. Terminals in the remote laptop connected to the Locobot:
    - In the first two terminals, connect to the Locobot via ssh:
        ```bash
        ssh -X devlab03@<LOCOBOT_IP> # Replace <LOCOBOT_IP> with the IP of the Locobot
        ```
    - Source the workspace in each terminal (depends on the installation path, but default is):
        ```bash
        source locobot_ws/install/setup.bash
        ```

    - In the first terminal, launch the Locobot control:
        ```bash
        ros2 launch locobot_control robot.launch.py state_machine:=true
        ```

    - In the second terminal, launch the gesture_recognition node:
        ```bash
        ros2 launch gesture_recognition gesture_recognition.launch.py
        ```

5. In the third terminal, launch the remote visualization and apriltag detection:
    ```bash
    # To launch only k02 camera
    ros2 launch locobot_control remote.launch.py
    # To launch all the cameras (k01, k02)
    ros2 launch locobot_control remote.launch.py camera_number:=2
    ```
    > **Note:** The remote launch file allows to set the camera number to visualize and the serial number of the kinect cameras. See the [README](src/locobot_control/README.md) in the locobot_control package for more information.

6. Follow the instructions in the gesture recognition README to understand how to use the gestures.

7. In the fourth terminal, whenever you're ready, start the state machine:
    ```bash
    ros2 service call /clear_error_state locobot_control_interfaces/srv/ClearError
    ```

> **Note:** If not both human and robot TF are published, after a certain amount of time (default 5 seconds) the robot will stop and the state machine will go in error state. To clear the error state, use the command in step 7.

## Repository Structure

### UDEV Rules and Installation Scripts
#### [`60-kobuki.rules`](60-kobuki.rules)
udev rules for the kobuki base to be installed in the Locobot's NUC.

#### [`installer.sh`](installer.sh)
Installation script to build the workspace and install the necessary packages.

#### [`install_kinect.sh`](install_kinect.sh)
Installation script to install the Azure Kinect ROS driver.

#### [`calibration.sh`](calibration.sh)
Script to calibrate the Kinect cameras network.

### Imported Modules

The repository includes the following submodules:

- [interbotix_ros_rovers](https://github.com/Alessio-Lovato-Unipd/interbotix_ros_rovers.git) - branch: humble
- [interbotix_ros_core](https://github.com/Alessio-Lovato-Unipd/interbotix_ros_core.git) - branch: humble
- [interbotix_ros_toolboxes](https://github.com/Alessio-Lovato-Unipd/interbotix_ros_toolboxes.git) - branch: humble
- [kobuki_ros](https://github.com/kobuki-base/kobuki_ros.git) - branch: devel
- [kobuki_core](https://github.com/kobuki-base/kobuki_core.git) - branch: release/1.4.x
- [kobuki_ros_interfaces](https://github.com/kobuki-base/kobuki_ros_interfaces.git) - branch: release/1.0.x
- [ecl_core](https://github.com/stonier/ecl_core.git) - branch: release/1.2.x
- [ecl_lite](https://github.com/stonier/ecl_lite.git) - branch: release/1.2.x
- [sophus](https://github.com/clalancette/sophus.git) - from apt package in install script
- [Azure_Kinect_ROS_Driver](https://github.com/mguidolin/Azure_Kinect_ROS_Driver.git) - branch: fix
- [apriltag_ros](https://github.com/Adlink-ROS/apriltag_ros.git) - branch: foxy-devel
- [apriltag_calibration](https://github.com/mguidolin/apriltag_calibration.git) - branch: ros2-humble

### Custom Packages
Packages developed for the LoCoBot workspace:
- [gazebo_ros2_control](src/gazebo_ros2_control/): Gazebo control plugin for ROS2 (version 0.4.6). It is an old version of the plugin, used due to an [issue](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/243) with the latest version.
- [gesture_recognition](src/gesture_recognition/): Gesture recognition package for the LoCoBot. It is used to detect gestures from the Realsense camera and publish them to the state machine.
- [locobot_control](src/locobot_control/): Control package for the LoCoBot. It includes the state machine and the control nodes for the robot.
- [locobot_control_interfaces](src/locobot_control_interfaces/): Custom interfaces for the control package.

## Reference
This work is part of my master thesis in Mechatronics. If you use this repository, please cite the [thesis](https://hdl.handle.net/20.500.12608/77775).

## License

This project is licensed under the terms of the Apache 2.0 License. See the [LICENSE](LICENSE) file for details.
