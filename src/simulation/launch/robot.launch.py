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
@file robot.launch.py
@description Launch file for actual Iterbotix Locobot with Navigation2 and MoveIt2.

@details
This launch file is used to bring up the Interbotix Locobot with Navigation2 and MoveIt2.
It is intended to be used over ssh in the actual robot.
It loads the following components:
    - Interbotix ROS Control
    - Navigation2
    - MoveIt2
    - Realsense2 Camera

The launch file also remaps the cmd_vel topic published from the 'Controller server' to the diffdrive_controller.

@param robot_name: name of the robot (default: locobot)
@param base_type: type of the base (default: kobuki)
@param external_srdf_loc: the file path to the custom semantic description file that you would like to include in the Interbotix robot's semantic description
@param external_urdf_loc: the file path to the custom URDF file that you would like to include in the Interbotix robot
@param nav2_params_file: the file path to the params YAML file (default: config/robot_navigation.yaml)
@param rs_camera_param: the file path to the Realsense camera configuration file (default: config/rs_camera.yaml)
@param container: name of an existing node container to load launched nodes into. If unset, a new container will be created
"""

import os
import yaml

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
    TimerAction,
    GroupAction
)
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import (
    Node,
    SetRemap,
    ComposableNodeContainer,
    LoadComposableNodes,
    SetParameter
)

from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile, ComposableNode

from nav2_common.launch import RewrittenYaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None



def generate_launch_description():

    # Declare launch arguments
    robot_name = LaunchConfiguration('robot_name')
    base_type = LaunchConfiguration('base_type')
    external_srdf_loc = LaunchConfiguration('external_srdf_loc')
    external_urdf_loc = LaunchConfiguration('external_urdf_loc')

    nav2_params_file_launch_arg = DeclareLaunchArgument(
            name='nav2_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('simulation'),
                'config',
                'robot_navigation.yaml'
            ]),
            description='the file path to the params YAML file.'
    )

    robot_name_launch_arg = DeclareLaunchArgument(
            name='robot_name',
            default_value='locobot',
            description='name of the robot (could be anything but defaults to `locobot`).',
    )

    base_type_launch_arg = DeclareLaunchArgument(
            name='base_type',
            default_value='kobuki',
            description='type of the base (could be `kobuki` or `create3`).',
    )
    
    external_srdf_loc_launch_arg = DeclareLaunchArgument(
            name='external_srdf_loc',
            default_value='',
            description=(
                'the file path to the custom semantic description file that you would like to '
                "include in the Interbotix robot's semantic description."
            ),
    )

    external_urdf_loc_launch_arg = DeclareLaunchArgument(
            name='external_urdf_loc',
            default_value=PathJoinSubstitution([FindPackageShare('simulation'), 'urdf', 'locobot_tag.urdf.xacro']),
            description='the file path to the custom URDF file that you would like to include in the Interbotix robot.',
    )
    
    arg_container = DeclareLaunchArgument(
        name='container',
        default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    rs_camera_param_launch_arg = DeclareLaunchArgument(
        name='rs_camera_param',
        default_value=PathJoinSubstitution([FindPackageShare('simulation'), 'config', 'rs_camera.yaml']),
        description='the file path to the Realsense camera configuration file.'
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
                },
                'use_sim_time': True,
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

############################################################################################################
##########################################  LOCOBOT CONTROL  ###############################################
############################################################################################################

    # Load the ROS control launch file
    ros_control = PathJoinSubstitution(
            [FindPackageShare('interbotix_xslocobot_ros_control'), 'launch', 'xslocobot_ros_control.launch.py']
        )
    
    ros_control_locobot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ros_control]),
                launch_arguments={
                    'use_lidar': 'false',
                    'use_base': 'true',
                    'hardware_type': 'actual',
                    'robot_model': 'locobot_wx200',
                    'use_camera': 'false',
                }.items()
    )

    # Load realsense2 node into the container
    # (the configuration file is similar to the one in the 'interbotix_xslocobot_control' package)
    realsense_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration('rs_camera_param'),
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    realsense_node = ComposableNode(
        package='realsense2_camera',
        namespace='locobot',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name="rs_camera",
        parameters=[realsense_params],
        extra_arguments=[{'use_intra_process_comms': True}],
    )
  
    # Create the container for components
    rs_camera = GroupAction(
        actions=[
            # Container name not provided, using navigation container
            ComposableNodeContainer(
                condition=LaunchConfigurationEquals('container', ''),
                name='rs_container',
                namespace='',
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[realsense_node],
                output='screen',
                parameters=[configured_params],
            ),
            # Container name provided, using provided container
            LoadComposableNodes(
                condition=LaunchConfigurationNotEquals('container', ''),
                composable_node_descriptions=[realsense_node],
                target_container=LaunchConfiguration('container'),
            ),
        ]
    )

############################################################################################################
############################################  EVENTS HANDLER  ##############################################
############################################################################################################

    # Delayed items to have better control over the order of execution
    delayed_items = TimerAction(
        period=5.0, #Delay in seconds
        actions=[rs_camera,
                move_group_node
        ]
    )

    return LaunchDescription([
        robot_name_launch_arg,
        base_type_launch_arg,
        external_srdf_loc_launch_arg,
        external_urdf_loc_launch_arg,
        nav2_params_file_launch_arg,
        rs_camera_param_launch_arg,
        arg_container,
    # Robot launch
        ros_control_locobot,
        delayed_items
    ])