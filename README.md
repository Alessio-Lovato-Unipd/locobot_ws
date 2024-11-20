# LoCoBot Demo

This branch (demo) contains the files to perform a demo with the LoCoBot, an open-source robot developed by [Interbotix](https://www.interbotix.com).<br>
It includes all the necessary ROS packages to operate the robot and is intended to be installed on the onboard NUC with an Ubuntu 22.04 distribution.<br>
To use the joystick -only ps3 at the moment - for the navigation, the workspace (or only the contained 'joystick' package) has to be installed in the computer which the joystick is connected.

## Prerequisites

- ROS2 Humble
- Python 3
- [LoCoBot](http://www.locobot.org/) hardware

## Installation
> **_NOTE:_**  To connect to the Locobot, please see Connection section and then proceed with installation.

1. Clone the repository:

    ```bash
    git clone -b demo https://github.com/Alessio-Lovato-Unipd/locobot_ws.git
    cd locobot_ws/
    ```

2. Install dependencies
    ```bash
    sudo pip3 install transforms3d modern-robotics
    sudo apt-get install ros-humble-tf-transformations
    rosdep update
    sudo apt-get update
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Update submodules
    ```bash
    git submodule update --init --remote
    ```

    3.1. Update inner submodules
    ```bash
    git submodule update --init --remote --recursive
    ```

    3.2. In case an error occours about the package 'interbotix_xs_driver' execute this script
    ```bash
    cd src/interbotix_ros_core
    git submodule update --init --remote
    cd ../..
    ```

4. In case the package 'gazebo_ros2_control' is installed, remove it:
    ```bash
    sudo apt uninstall ros-humble-gazebo-ros2-control
    ```

5. Build the workspace

    5.1. Build 'sophus' package since it is the heaviest

    ```bash
    colcon build --packages-select sophus
    ```
    5.2 Build all the other packages
    ```bash
    colcon build
    ```
    5.3 [optional]: In case an error occours during building about a missing ament_path of the package 'joystick', please comment line 19 [ findPackage(joystick) ] in the file `src/simulation/CMakeLists.txt` and perform again point 4.2.

6. Source the setup:

    ```bash
    source install/setup.bash
    ```

7. Install udev rules [only on Locobot]:
    - [XArms instructions](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html) -  [Xarms rules file](https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/rpi4/xsarm_rpi4_install.sh)
    - [Locobot instructions]() - [Locobot rules file]()

    ```bash
    # X-Arms rules
    cd src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
    sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    cd ../../..
    # Kobuki rules
    sudo cp 60-kobuki.rules /etc/udev/rules.d
    sudo service udev reload
    sudo service udev restart
    ```

## Connection
It is possbile to connect to the locobot via WiFi or Ethernet.

### Ethernet connection
This is the easiest connection mode. Just attach an ethernet cambel from your computer to the Locobot. Make sure that your network settings are in Link-Local only. It is preferable to create a new profile.

Network -> Settings -> "+" -> IPv4 -> Link-Local Only

Now open a terminal and connect to the locobot via ssh:
```bash
    ssh -x devlab03@devlab.local;
```

Digitate 'yes' if a new connection is asked and then insert the password.
Now you are logged into the Locobot's NUC.

### Wifi connection
The locobot connects automatically to the last Wifi network if available.
To add a wifi network, you must connect a screen to the Locobot, as well as a mouse and a keyboard (no bluetooth) and use the NUC as a normal computer.<br>
Depending on the network is possible that ssh via local (command used with ethernet conncetion) cannot be performed. <br> In this case, lookup for your ip using the network settings.
Then connect to the locobot from your computer using

```bash
    ssh -x devlab03@<IP ADDRESS>;
```

## Usage
1. Turn on the NUC and the base.

2. After the connection, on the ssh terminal type the command:
    ```bash
        source locobot_ws/install/setup.bash
        ros2 launch simulation robot.launch.py
    ```
    This will run the launch file that loads the gesture recognition module, the state machine to control the interaction and the moveit configuration to move the arm. It will also load the base commands.

3. Connect the controller to your computer.
4. On another terminal open the workspace and source it (```source install/setup.bash```).
5. On this second terminal run the command
    ```bash
        ros2 launch joystick joystick_PS3.launch.py
    ```
    And follow the instructions printed on screen. It might necessary disconnect and reconnect the controller (while the program is running) if it doesn't work.

## Recognized gestures

- **Open Palm (towards down)**: To idle state (only from arm extended)
- **Pointing down**: Open gripper (only from arm extended)
- **Closed fist (towards down)**: Close gripper to HOME position (only from arm extended)
- **Thumb Up**: Extend arm 
## Repository Structure

The first commit of this repository contains the following packages in the `src` folder:

- [ecl_core](https://github.com/stonier/ecl_core) - Release 1.2.x
- [ecl_lite](https://github.com/stonier/ecl_lite) - Release 1.2.x
- [interbotix_ros_core](https://github.com/Interbotix/interbotix_ros_core) - humble
- [interbotix_ros_rovers](https://github.com/Interbotix/interbotix_ros_rovers) - humble
- [interbotix_ros_toolboxes](https://github.com/Interbotix/interbotix_ros_toolboxes)
- [kobuki_core](https://github.com/kobuki-base/kobuki_core) - Release 1.4.x
- [kobuki_ros](https://github.com/kobuki-base/kobuki_ros) - devel
- [kobuki_ros_interfaces](https://github.com/kobuki-base/kobuki_ros_interfaces) - Release 1.0.x
- [sophus](https://github.com/clalancette/sophus) - clalancette/fix-compiler-error
- simulation - contains the control node and the state machine
- simulation_interface - contains the custom ROS2 interfaces used to communicate with the state machine
- joystick - contains the joystick node
- gesture_recognition - Contains the gesture recognition node

Futhermore, the rules to install the kobuki base are placed into the file 
- `60-kobuki.rules`: udev rules for the robot.

<br>

## License

This project is licensed under the terms of the Apache 2.0 License. See the [LICENSE](LICENSE) file for details.
