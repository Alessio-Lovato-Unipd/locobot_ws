#!/bin/bash

# This script is used to automate the process of calibration of the kinect cameras.
# 'apriltag_calibration' package needs to be built and sourced.

# Steps:
# 1. Launch the kinect camera calibration node (i.e. K01calib.launch.py)
# 3. Launch the calibration master node (calib_master.launch.py)
# 4. Run this file and attend the result

ros2 service call /hiros/start_acquisition hiros_apriltag_calibration/srv/StartAcquisition "{}"

sleep 10

ros2 service call /hiros/stop_acquisition hiros_apriltag_calibration/srv/StopAcquisition "{}"

echo "Calibration process completed"
