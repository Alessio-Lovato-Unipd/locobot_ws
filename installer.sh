#!/bin/bash

# Install python packages
echo "Installing Python dependencies and ssh"
sudo pip3 install transforms3d modern-robotics
sudo apt-get install ssh -y

# Add CycloneDDS export to bashrc
echo "Adding CycloneDDS export to bashrc"
sudo apt-get install ros-humble-cyclonedds -y
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc

# Fetch and update modules
echo "Fetching and updating submodules"
git submodule update --init --recursive

# Update dependencies
echo "Updating dependencies"
rosdep update
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y

# Build packages
echo "Building packages"
source /opt/ros/humble/setup.bash
colcon build --packages-select sophus locobot_control_interfaces
source install/setup.bash
colcon build --packages-skip sophus azure_kinect_ros_driver --parallel-workers 6
source install/setup.bash

echo "Building completed"
