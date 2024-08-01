## SLAM_IN_SIMULATION Package

This package is designed for creating maps of simulated environments using the `Gazebo Simulator` in ROS 2 Humble distribution, specifically tailored for use with the LoCobot software provided by Interbotix.

### Dependencies

Required ROS 2 packages:
- [nav2_bringup](https://index.ros.org/p/nav2_bringup/)
- [slam_toolbox](https://index.ros.org/r/slam_toolbox/github-SteveMacenski-slam_toolbox/)

### Package Structure

The package is organized into the following folders:

- **config**: Contains configuration files used by the `slam_toolbox`.
- **launch**: Includes launch files for bringing up required nodes.
- **maps**: Provides an example map.

### Usage

To use the package, follow these steps:

1. Source your ROS 2 distribution.
2. Launch the Locobot simulation stack:
   ```bash
   ros2 launch interbotix_xslocobot_sim xslocobot_gz_classic.launch.py use_sim:=true hardware_type:=gz_classic use_lidar:=true
    ```
3. In another terminal, launch the slam_in_simulation package:
    ```bash
    ros2 launch slam_in_simulation slam.launch.py
    ```
4. Open Rviz2 and the _'Map'_ plugin.
5. Drive around the locobot using the `keyboard_teloeop` or a joystick.
6. In a separate terminal, save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f map
```
This generates two files in the current directory: `map.yaml` and `map.pgm`.

### License

This project is licensed under Apache-2.0 License. See the `LICENSE` file for details.