cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_util REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(image_proc REQUIRED)
#find_package(spatio_temporal_voxel_layer REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)

# source file dependencies
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(simulation_interfaces REQUIRED)


#moveit dependencies
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Action server State Machine
add_library( action_server SHARED
  src/state_machine/StateMachine.cpp src/LocobotControl.cpp)

target_include_directories(action_server PRIVATE 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(action_server PRIVATE "STATE_MACHINE_BUILDING_DLL")
ament_target_dependencies(action_server  
                              "rclcpp"
                              "rclcpp_action"
                              "rclcpp_components"
                              "geometry_msgs"
                              "nav2_msgs"
                              "moveit_core"
                              "moveit_ros_planning_interface"
                              "moveit_msgs"
                              "simulation_interfaces")

rclcpp_components_register_node(action_server PLUGIN "StateMachine" EXECUTABLE state_machine_action_server)
install(TARGETS action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Demo program
add_executable(demo src/demo.cpp)
ament_target_dependencies(demo rclcpp
                               geometry_msgs 
                               nav2_msgs 
                               rclcpp_action 
                               rclcpp_components
                               moveit_core
                               moveit_ros_planning_interface
                               moveit_msgs)
target_include_directories(demo PUBLIC include)

add_executable(demo_no_lidar src/demo_no_lidar.cpp)
ament_target_dependencies(demo_no_lidar rclcpp
                               geometry_msgs 
                               nav2_msgs 
                               rclcpp_action 
                               rclcpp_components
                               moveit_core
                               moveit_ros_planning_interface
                               moveit_msgs)
target_include_directories(demo_no_lidar PUBLIC include)



# Tf remapper
add_executable(tf_remapper src/LocobotTracker.cpp)
ament_target_dependencies(tf_remapper rclcpp
                                      geometry_msgs
                                      tf2_ros)
target_include_directories(tf_remapper PUBLIC include)

# Arm sleep position
add_executable(arm_sleep_position src/ArmSleepPosition.cpp)
ament_target_dependencies(arm_sleep_position rclcpp
                               geometry_msgs 
                               nav2_msgs 
                               rclcpp_action 
                               rclcpp_components
                               moveit_core
                               moveit_ros_planning_interface
                               moveit_msgs)
target_include_directories(arm_sleep_position PUBLIC include)

# BT demo
add_executable(bt_demo src/BT/demo_navigation.cpp src/LocobotControl.cpp)
ament_target_dependencies(bt_demo 
                              behaviortree_cpp
                              behaviortree_ros2
                              rclcpp
                               geometry_msgs 
                               nav2_msgs 
                               rclcpp_action 
                               rclcpp_components
                               moveit_core
                               moveit_ros_planning_interface
                               moveit_msgs)
target_include_directories(bt_demo PUBLIC include)

# install targets
install(TARGETS
  demo
  demo_no_lidar
  tf_remapper
  arm_sleep_position
  bt_demo
  #state_machine
  DESTINATION lib/${PROJECT_NAME})

# install directories
install(DIRECTORY
    launch
    config
    maps
    rviz
    models
    urdf
    media
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
