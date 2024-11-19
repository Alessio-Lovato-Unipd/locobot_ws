cmake_minimum_required(VERSION 3.8)
project(simulation)

set (CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(joystick REQUIRED)
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


# install directories
install(DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
