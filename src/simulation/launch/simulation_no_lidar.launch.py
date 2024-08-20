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
    OpaqueFunction,
    GroupAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution
)
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
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
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    robot_name = LaunchConfiguration('robot_name')
    use_lidar = LaunchConfiguration('use_lidar')
    base_type = LaunchConfiguration('base_type')
    external_urdf_loc = LaunchConfiguration('external_urdf_loc')

    # Launch arguments' default values
    map_yaml_file_launch_arg = DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('simulation'),
                'maps',
                'simulation_map',
                'simulation_map.yaml'
            ]),
            description='the file path to the map YAML file.'
        )

    params_file_launch_arg = DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('simulation'),
                'config',
                'navigation.yaml'
            ]),
            description='the file path to the params YAML file.'
        )

    robot_name_launch_arg = DeclareLaunchArgument(
            'robot_name',
            default_value='locobot',
            description='name of the robot (could be anything but defaults to `locobot`).',
        )
    
    use_lidar_launch_arg = DeclareLaunchArgument(
            'use_lidar',
            default_value='false',
            description='whether to use the lidar sensor or not.',
        )

    base_type_launch_arg = DeclareLaunchArgument(
            'base_type',
            default_value='kobuki',
            description='type of the base (could be `kobuki` or `create3`).',
        )
    
    external_urdf_loc_launch_arg = DeclareLaunchArgument(
            'external_urdf_loc',
            default_value=PathJoinSubstitution([FindPackageShare('simulation'), 'urdf', 'locobot_tag.urdf.xacro']),
            description=(
                'the file path to the custom semantic description file that you would like to '
                "include in the Interbotix robot's semantic description."
            ),
        )

    # Define Gazebo classic environment variables
    resources = os.path.join(get_package_share_directory('simulation'))
    models = os.path.join(get_package_share_directory('simulation'), 'models')

    if 'GAZEBO_RESOURCE_PATH' in os.environ:
        resources_path =  os.environ['GAZEBO_RESOURCE_PATH'] + ':' + resources
    else:
        resources_path =  resources

    if 'GAZEBO_MODEL_PATH' in os.environ:
        models_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + models
    else:
        models_path = models

    # Navigation2 bringup launch
    package = FindPackageShare('simulation')
    nav2_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    nav2_launch = GroupAction(
        actions=[
            # Remap the cmd_vel topic published from 'Controller server' to the diffdrive_controller
            SetRemap(src='/cmd_vel', dst='/locobot/diffdrive_controller/cmd_vel_unstamped'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir]),
                launch_arguments={
                    'params_file': params_file,
                    'use_sim_time': 'True',
                    'autostart': 'True',
                }.items()
            ),
            # Launch map server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_yaml_file}],
                remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
            ),
            # Activate lifecycle manager for map server
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_map_server',
                output='screen',
                parameters=[{'node_names': ['map_server']},
                            {'autostart': True},
                            {'use_sim_time': True}],
            ),
        ])



    # locobot simulation launch
    gazebo_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("interbotix_xslocobot_sim"), 'launch', 'xslocobot_gz_classic.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'hardware_type': 'gz_classic',
            'use_lidar': use_lidar,
            'use_rviz': 'true',
            'use_camera': 'true',
            'external_urdf_loc': external_urdf_loc,
            'use_gazebo_debug': 'false',
            
        }.items()
    )

    # Launch apriltag tag entity in gazebo
    spawn_apriltag = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_map_tag',
        arguments=['-entity', 'map_tag', '-file',
                        PathJoinSubstitution([FindPackageShare('simulation'), 'models', 'map_tag', 'model.sdf']),
                        '-x', '0.0', '-y', '-1.0', '-z', '0.01', # z = 0.01 because the model 'visual' is 1cm tall
                        '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output='screen'
    )
    spawn_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_camera',
        arguments=['-entity', 'camera', '-file',
                        PathJoinSubstitution([FindPackageShare('simulation'), 'models', 'camera', 'model.sdf']),
                        '-x', '0.0', '-y', '0.0', '-z', '5.0',
                        '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare("simulation"), 'rviz', 'navigation.rviz'])],
        output='screen'
    )

    # AprilTag node that publishes the pose of the tags
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        parameters= [os.path.join(get_package_share_directory('simulation'), 'config', 'apriltag.yaml')],
        remappings=[('/camera_info', '/locobot/env_camera/camera_info'),
                    ('/image_rect', '/locobot/env_camera/image_raw'),
                    ('/tf', '/tf_tag')],

    )

    apriltag_static_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='locobot_tag_br',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '-1.57076',
            '--frame-id', 'apriltag_link',
            '--child-frame-id', 'locobot_tag'
        ],
        output='screen'
    )

    map_static_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='locobot_tag_br',
        arguments=[
            '--x', '-1.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '1.57076',
            '--frame-id', 'map_tag',
            '--child-frame-id', 'map'
        ],
        output='screen'
    )

    tf_remapper = Node(
        package='simulation',
        executable='tf_remapper',
        name='tf_remapper',
        output='screen',
        parameters=[{'use_sim_time':True}],
        remappings=[('/tf', '/tf_tag'),
                    ('/env_camera/tf_locobot', '/tf')]
    )
    

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', value=resources_path),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', value=models_path),
        robot_name_launch_arg,
        use_lidar_launch_arg,
        base_type_launch_arg,
        external_urdf_loc_launch_arg,
        map_yaml_file_launch_arg,
        params_file_launch_arg,
        nav2_launch,
        gazebo_simulation_launch,
        spawn_camera,
        spawn_apriltag,
        apriltag_node,
        tf_remapper,
        apriltag_static_broadcaster,
        map_static_broadcaster,
        #rviz,
    ])