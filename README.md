# LoCoBot Workspace

This repository contains the workspace for LoCoBot, an open-source robot developed by [Interbotix](https://www.interbotix.com).<br>
It includes all the necessary ROS packages to operate the robot and is intended to be installed on the onboard NUC with an Ubuntu 22.04 distribution.

## Prerequisites

- ROS2 Humble
- Python 3
- [LoCoBot](http://www.locobot.org/) hardware

## Installation

1. Clone the repository:

    ```bash
    git clone https://github.com/Alessio-Lovato-Unipd/locobot_ws.git
    cd locobot_ws/
    ```

2. Install dependencies
    ```bash
    sudo pip3 install transforms3d modern-robotics
    sudo apt-get install ros-humble-tf-transformation
    rosdep update
    sudo apt-get update
    rosdep install --from-paths src --ignore-src -r -y
    ```
3. Update submodules
    ```bash
    cd src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_driver/
    git submodules init
    git submodules update --recursive
    cd ../../../..
    ```
3. Build the workspace:

    ```bash
    colcon build
    ```

3. Source the setup:

    ```bash
    source install/setup.bash
    ```

4. Install udev rules:
    - [XArms instauctions](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html) -  [Xarms rules file](https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/rpi4/xsarm_rpi4_install.sh)
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

## Usage



## Repository Structure

The first commit of this repository contains the following packages in the `src` folder:

- [ecl_core](https://github.com/stonier/ecl_core) - Release 1.2.x
- [ecl_lite](https://github.com/stonier/ecl_lite) - Release 1.2.x
- [interbotix_ros_core](https://github.com/Interbotix/interbotix_ros_core) - humble
- [interbotix_ros_rovers](https://github.com/Interbotix/interbotix_ros_rovers) - humble
- [interbotix_ros_toolboxes](https://github.com/Interbotix/interbotix_ros_toolboxes)
- [kobuki_core](https://github.com/kobuki-base/kobuki_core) - Release 1.4.x
- [kobuki_ros](https://github.com/kobuki-base/kobuki_ros) - Release 1.1.x
- [kobuki_ros_interfaces](https://github.com/kobuki-base/kobuki_ros_interfaces) - Release 1.0.x
- [sophus](https://github.com/clalancette/sophus) - clalancette/fix-compiler-error

Futhermore, the rules to install the kobuki base are placed into the file 
- `60-kobuki.rules`: udev rules for the robot.

<br>


> **_NOTE:_**  Further commits will modify those packages to provide full functionality of the LoCoBot.


## License

This project is licensed under the terms of the Apache 2.0 License. See the [LICENSE](LICENSE) file for details.
