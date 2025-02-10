# Copyright 2022 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
@file simulation.launch.py

@brief This file is used to launch a complete simulation of the Locobot without the lidar sensor. Gazebosim classic is used.

@details The simulation includes the Interbotix Locobot, the AprilTag markers, the MoveIt2 and Navigation2. The files
'include_camera.launch.py' and 'include_markers.launch.py' are included in this launch file to include the camera and
the markers in the simulation. The file 'navigation.rviz' is used to visualize the simulation in RViz2.

@param nav2_param_file: the file path to the params YAML file of Nav2. (default: '')
@param external_urdf_loc: the file path to the custom URDF file that you would like to include in the Interbotix robot. (default: 'urdf/locobot_tag.urdf.xacro')
@param camera_number: Number of cameras to be included in the simulation. Can be 1 or 2. (default: 1)
@param spawn_obstacle: Flag to spawn an obstacle in the simulation. Can be true or false. (default: true)
@param container: Name of the container where to load the components. Default is 'nav2_container'.
@param nav_controller: the Nav2 controller plugin to use. Can be 'mppi' (default) or 'rpp'.

@note This file is a blend of the launch files 'interbotix_xslocobot_sim.launch.py', 'interbotix_xslocobot_moveit.launch.py' and 'navigation2.launch.py' 
from the 'interbotix_xslocobot_sim', 'interbotix_xslocobot_moveit' and 'navigation2_bringup' packages.
"""

import os
import yaml
from pathlib import Path


from ament_index_python.packages import get_package_share_directory
from interbotix_xs_modules.xs_common import (
    get_interbotix_xslocobot_models,
)
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xslocobot_semantic_robot_description_command
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    SetEnvironmentVariable,
    OpaqueFunction,
    TimerAction
)
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import (
    Node,
    SetRemap,
    LoadComposableNodes,
    ComposableNodeContainer,
    SetParameter
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile, ComposableNode
from nav2_common.launch import RewrittenYaml



"""
@breif Function to load a YAML file
"""
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None




"""
@breif Function to set the environment variables for Gazebo Simulator
"""
def set_env_vars(context, *args, **kwargs):
    gz_resource_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value=''),
            ':',
            str(Path(
                FindPackageShare('locobot_control').perform(context)
            ).resolve())
        ]
    )

    gz_model_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
            ':',
            str(Path(
                FindPackageShare('locobot_control').perform(context)
            ).resolve()),
            '/models:',
        ]
    )

    gz_media_path_env_var = SetEnvironmentVariable(
        name='GAZEBO_MEDIA_PATH',
        value=[
            EnvironmentVariable('GAZEBO_MEDIA_PATH', default_value=''),
            ':',
            str(Path(
                FindPackageShare('locobot_control').perform(context)
            ).resolve())
        ]
    )

    return [gz_resource_path_env_var, gz_model_path_env_var, gz_media_path_env_var]






"""
@brief Function that loads all the nodes
"""
def launch_description(context, *args, **kwargs):

############################################################################################################
######################################  GAZEBO CLASSIC SIMULATION  #########################################
############################################################################################################

    # locobot simulation launch
    gazebo_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("interbotix_xslocobot_sim"), 'launch', 'xslocobot_gz_classic.launch.py'])),
        launch_arguments={
            'hardware_type': 'gz_classic',
            'use_lidar': 'false',
            'use_rviz': 'false',
            'use_camera': 'true',
            'external_urdf_loc': LaunchConfiguration('external_urdf_loc'),
            'external_srdf_loc': '',
            'use_gazebo_debug': 'false',
            'robot_model': 'locobot_wx200',
            'base_type': 'kobuki',
            'use_gazebo_verbose': 'false',
        }.items(),
    )

    obstacle_model_path = PathJoinSubstitution([FindPackageShare("locobot_control"), 
                                            'models', 'obstacle', 'model.sdf'])

    spawn_obstacle = Node(
        condition=LaunchConfigurationEquals('spawn_obstacle', 'true'),
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_camera_1",
        arguments=["-entity", "obstacle", "-file", obstacle_model_path, 
                   '-x', '-0.5', '-y', '0.0', '-z', '0.0',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output="screen"
    )
############################################################################################################
##############################################  MOVEIT2  ###################################################
############################################################################################################

    config_path = PathJoinSubstitution([
        FindPackageShare('interbotix_xslocobot_moveit'),
        'config',
    ])

    robot_description_semantic = {
        'robot_description_semantic':
            construct_interbotix_xslocobot_semantic_robot_description_command(
                robot_model='locobot_wx200',
                config_path=config_path
            ),
    }

    kinematics_config = PathJoinSubstitution([
        FindPackageShare('interbotix_xslocobot_moveit'),
        'config',
        'kinematics.yaml',
    ])

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin':
                'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error':
                0.1,
        }
    }

    ompl_planning_pipeline_yaml_file = load_yaml(
        'interbotix_xslocobot_moveit', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_pipeline_yaml_file)

    controllers_config = load_yaml(
        'interbotix_xslocobot_moveit',
        'config/controllers/mobile_wx200_controllers.yaml'
    )

    config_joint_limits = load_yaml(
        'interbotix_xslocobot_moveit',
        'config/joint_limits/mobile_wx200_joint_limits.yaml'
    )

    joint_limits = {
        'robot_description_planning': config_joint_limits,
    }

    moveit_controllers = {
        'moveit_simple_controller_manager':
            controllers_config,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution_parameters = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description_semantic': True,
    }

    sensor_parameters = {
        'sensors': [''],
    }

    remappings = [
        (
            'locobot/get_planning_scene',
            '/locobot/get_planning_scene'
        ),
        (
            '/arm_controller/follow_joint_trajectory',
            '/locobot/arm_controller/follow_joint_trajectory'
        ),
        (
            '/gripper_controller/follow_joint_trajectory',
            '/locobot/gripper_controller/follow_joint_trajectory'
        ),
        (
            '/robot_description',
            '/locobot/robot_description'
        ),
        (
            '/robot_description_semantic',
            '/locobot/robot_description_semantic'
        )
    ]

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[
            {
                'planning_scene_monitor_options': {
                    'robot_description':
                        'robot_description',
                    'joint_state_topic':
                        'locobot/joint_states',
                }
            },
            robot_description_semantic,
            kinematics_config,
            ompl_planning_pipeline_config,
            trajectory_execution_parameters,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits,
            sensor_parameters,
        ],
        remappings=remappings,
        output={'both': 'screen'},
    )

    arm_to_sleep_position = Node(
        package='locobot_control',
        executable='arm_sleep_position',
        output='screen',
        # Remapping is mandatory due to the namespace
        remappings=[
            ('/robot_description', '/locobot/robot_description'),
            ('/robot_description_semantic', '/locobot/robot_description_semantic')
        ]
    )

############################################################################################################
############################################  NAVIGATION2  #################################################
############################################################################################################

    # Launch configurations
    nav2_param_file = LaunchConfiguration('nav2_param_file').perform(context)
    nav_controller = LaunchConfiguration('nav_controller').perform(context)

    # Paths to default parameter files
    default_params_mppi = PathJoinSubstitution([
        FindPackageShare('locobot_control'), 'config', 'navigation_mppi.yaml'
    ])
    default_params_rpp = PathJoinSubstitution([
        FindPackageShare('locobot_control'), 'config', 'navigation_rpp.yaml'
    ])

    # Check if param file exists
    if nav2_param_file != '' and not os.path.exists(nav2_param_file):
        raise RuntimeError(f"nav2_param_file '{nav2_param_file}' does not exist")

    # Conditional substitution to select the default parameter file based on nav_controller
    default_params_file = PythonExpression([
        '"', default_params_mppi, '" if "', nav_controller, '" == \'mppi\' else "', default_params_rpp, '"'
    ])

    # Conditional substitution to select the parameter file
    params_file = PythonExpression([
        '"', nav2_param_file, '" if "', nav2_param_file, '" != \'\' else "', default_params_file, '"'
    ])

    # Configure parameters using RewrittenYaml
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={'autostart': 'True'},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
        'map_server',
    ]

    composable_nodes = [
        ComposableNode(
            package='nav2_controller',
            plugin='nav2_controller::ControllerServer',
            name='controller_server',
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            extra_arguments=[{'use_intra_process_comms': True}]
        ),
        ComposableNode(
            package='nav2_planner',
            plugin='nav2_planner::PlannerServer',
            name='planner_server',
            parameters=[configured_params],
            remappings=remappings,
            extra_arguments=[{'use_intra_process_comms': True}]
        ),
        ComposableNode(
            package='nav2_behaviors',
            plugin='behavior_server::BehaviorServer',
            name='behavior_server',
            parameters=[configured_params],
            remappings=remappings,
            extra_arguments=[{'use_intra_process_comms': False}] # intraprocess communication allowed only with volatile durability
        ),
        ComposableNode(
            package='nav2_bt_navigator',
            plugin='nav2_bt_navigator::BtNavigator',
            name='bt_navigator',
            parameters=[configured_params],
            remappings=remappings,
            extra_arguments=[{'use_intra_process_comms': False}] # intraprocess communication allowed only with volatile durability
        ),
        ComposableNode(
            package='nav2_velocity_smoother',
            plugin='nav2_velocity_smoother::VelocitySmoother',
            name='velocity_smoother',
            parameters=[configured_params],
            remappings=remappings,
            extra_arguments=[{'use_intra_process_comms': True}]
        ),
        ComposableNode(
            package='nav2_map_server',
            plugin='nav2_map_server::MapServer',
            name='map_server',
            parameters=[configured_params],
            remappings=remappings,
            extra_arguments=[{'use_intra_process_comms': False}] # intraprocess communication allowed only with volatile durability
        ),
        ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name='lifecycle_manager_navigation',
            parameters=[
                {'autostart': True, 'node_names': lifecycle_nodes}
            ]
        )
    ]

    nav2_launch = GroupAction(
        actions=[
            # Remap the cmd_vel topic published from 'Controller server' to the diffdrive_controller
            SetRemap(src='/cmd_vel', dst='/locobot/diffdrive_controller/cmd_vel_unstamped'),
            
                # If an existing container is not provided, start a container and load nodes into it
            ComposableNodeContainer(
                condition=LaunchConfigurationEquals('container', ''),
                name='nav2_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=composable_nodes,
                output='screen',
                parameters=[configured_params],
            ),

            # If an existing container name is provided, load composable nodes into it
            # This will block until a container with the provided name is available and nodes are loaded
            LoadComposableNodes(
                condition=LaunchConfigurationNotEquals('container', ''),
                composable_node_descriptions=composable_nodes,
                target_container=LaunchConfiguration('container'),
            ),
        ])



############################################################################################################
############################################  APRILTAG  ####################################################
############################################################################################################

    # Include the markers and the relative nodes in the simulation
    include_markers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("locobot_control"),
                                                         'launch', 'include_markers.launch.py'])),
        launch_arguments={
            'camera_number': LaunchConfiguration('camera_number'),
            'use_gazebo': 'false',
        }.items()
    ) 

############################################################################################################
############################################  RVIZ  ########################################################
############################################################################################################

    # RViz2 node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare("locobot_control"), 'rviz', 'navigation.rviz']),
                   '--ros-args', '--log-level', 'warn'],
        output='screen'
    )

############################################################################################################
############################################  STATE MACHINE   ##############################################
############################################################################################################

    state_machine = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('locobot_control'), 'launch', 'state_machine.launch.py')
        ),
        launch_arguments={
            'debug': 'true',
            'use_sim_time': 'true',
        }.items(),
        condition=LaunchConfigurationEquals('state_machine', 'true')
    )
    
############################################################################################################
############################################  EVENTS HANDLER  ##############################################
############################################################################################################

    # NOTE: This TimerAction gives some time to the spawn entities to create the plugin for the camera in the correct namespace
    # If the controller of the locobot (in Intebotix_xslocobot_sim) is loadded too early, the camera plugin loaded in the
    # file 'include_camera.launch.py', will get the namespace of the locobot controller and the apriltag_node will not be able
    # to find the camera topics
    wait_spawn_camera_services = TimerAction(
        period=5.0, #Delay in seconds
        actions=[include_markers_launch]
    )

    # Allow Gazebo to load the simulation before launching MoveIt2 and Navigation2
    wait_gazebo = TimerAction(
        period=10.0, #Delay in seconds
        actions=[move_group_node,
                    arm_to_sleep_position,
                    nav2_launch]
    )

    launch_state_machine = TimerAction(
        period=15.0, #Delay in seconds
        actions=[state_machine]
    )

    return [
        # 'use_sim_time' will be set on all nodes following the line above
        SetParameter(name='use_sim_time', value=True),
        # Environment variables
        OpaqueFunction(function=set_env_vars),
        # Simulation launch
        gazebo_simulation_launch,
        wait_spawn_camera_services,
        wait_gazebo,
        spawn_obstacle,
        # Rviz launch
        rviz,
        # State machine launch
        launch_state_machine
    ]







def generate_launch_description():
    # Declare launch arguments
    nav2_param_file_launch_arg = DeclareLaunchArgument(
            'nav2_param_file',
            default_value='',
            description='the file path to the navigation params YAML file.'
    )
 
    external_urdf_loc_launch_arg = DeclareLaunchArgument(
            'external_urdf_loc',
            default_value=PathJoinSubstitution([FindPackageShare('locobot_control'), 'urdf', 'locobot_tag.urdf.xacro']),
            description='the file path to the custom URDF file that you would like to include in the Interbotix robot.',
    )

    camera_number_launch_arg = DeclareLaunchArgument(
            'camera_number',
            default_value='1',
            choices=['1', '2'],
            description='Number of cameras to be included in the simulation. Can be 1 or 2.',
    )

    spawn_obstacle_launch_arg = DeclareLaunchArgument(
            'spawn_obstacle',
            default_value='true',
            description='Flag to spawn an obstacle in the simulation. Can be true or false.',
            choices=['true', 'false']
    )

    # Declare the 'container' launch configuration
    container_arg = DeclareLaunchArgument(
        name='container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    # Set navigation controller launch configuration
    controller_arg = DeclareLaunchArgument(
        name='nav_controller', default_value='mppi',
        choices=['mppi', 'rpp'],
        description='Select the navigation controller to use, default is MPPI. If nav2_param_file param is set, this argument is ignored.'
    )

    # Set the state machine launch configuration
    state_machine_arg = DeclareLaunchArgument(
        name='state_machine', default_value='false',
        choices=['true', 'false'],
        description='Select if the state machine should be launched, default is false.'
    )

    return LaunchDescription([    
        # Launch arguments
        external_urdf_loc_launch_arg,
        nav2_param_file_launch_arg,
        camera_number_launch_arg,
        spawn_obstacle_launch_arg,
        container_arg,
        controller_arg,
        state_machine_arg,
        # Launch main function
        OpaqueFunction(function=launch_description)])