"""
@file include_markers.launch.py

@brief This file is used to include the fiducial marker and camera entities and in gazebo simulation 
 and publish the corresponding tf frames. It also launches a Gazebo Classic simulation if the 'use_gazebo'
 argument is set to 'true'.

@param camera_number: Number of cameras to be included in the simulation. Can be 1 or 2. (default: 1)
@param use_gazebo: Flag to include the Gazebo Classic simulation. Can be true or false. (default: false)
@param container: Name of an existing node container to load launched nodes into. If unset, a new container will be created.

@note Please refer to file 'include_camera.launch.py' to understand the limitations of the
    camera entities in the simulation.
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    ld = LaunchDescription()

    # Declare the launch arguments
    camera_number_arg = DeclareLaunchArgument(
        'camera_number',
        default_value='1',
        choices=['1', '2'],
        description='Number of cameras to be included in the simulation. Can be 1 or 2.',
    )
    
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        choices=['true', 'false'],
        description='Flag to include the Gazebo Classic simulation.',
    )

    container_arg = DeclareLaunchArgument(
        'container',
        default_value='',
        description='Name of an existing node container to load launched nodes into. If unset, a new container will be created.'
    )

    # Load 'include_camera.launch.py' to include the camera entities
    include_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('simulation'), 'launch', 'include_camera.launch.py'])
        ),
        launch_arguments={
            'camera_number': LaunchConfiguration('camera_number'),
            'use_gazebo': LaunchConfiguration('use_gazebo')
        }.items()
    )

   # Create the nodes 
    composable_nodes = [
        ComposableNode(
            package='apriltag_ros',
            plugin='AprilTagNode',
            name='apriltag_node_1',
            parameters=[PathJoinSubstitution([
                FindPackageShare('simulation'), 'config', 'apriltag.yaml']
            )],
            remappings=[('/camera_info', '/locobot/env_camera_1/camera_info'),
                        ('/image', '/locobot/env_camera_1/image_raw')],
        ),
        ComposableNode(
            condition=LaunchConfigurationEquals('camera_number', '2'),
            package='apriltag_ros',
            plugin='AprilTagNode',
            name='apriltag_node_2',
            parameters= [PathJoinSubstitution([
                FindPackageShare('simulation'), 'config', 'apriltag.yaml']
            )],
            remappings=[('/camera_info', '/locobot/env_camera_2/camera_info'),
                        ('/image', '/locobot/env_camera_2/image_raw')],
        )
    ]

    # Create container for the apriltag components
    container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    # Create apriltag node for camera 1
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        target_container=LaunchConfiguration('container'),
        composable_node_descriptions=composable_nodes
    )

    # Static broadcaster for the apriltag marker above the locobot
    apriltag_static_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='locobot_tag_static_tf_broadcaster',
        arguments=[
            '--x', '0.0', '--y', '0.127', '--z', '-0.493',
            '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
            '--frame-id', 'locobot_tag_rotated', '--child-frame-id', 'locobot/base_footprint'
        ],
        output='screen'
    )

    # Create apriltag republisher to rotate # Configuration file
    config_file = os.path.join(get_package_share_directory('simulation'), 'config', 'apriltag.yaml')

    # Rotate the pose of the markers
    with open(config_file, 'r') as stream:
        yaml_data = yaml.safe_load(stream)
        tag_frames = yaml_data['tag_frames']
        for tag_frame in tag_frames:
            ld.add_action(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=[
                    '--x', '0', '--y', '0', '--z', '0',
                    '--roll', '3.14159265359', '--pitch', '-1.57079632679', '--yaw', '0',
                    '--frame-id', tag_frame, '--child-frame-id', tag_frame + '_rotated'
                ]
            ))

    # Create event handlers to create a proper sequence of actions
    delayed_spawn = TimerAction(
        period=2.0, #Delay in seconds
        actions=[container, load_composable_nodes, apriltag_static_broadcaster]
    )

    # Create the actions
    ld.add_action(camera_number_arg)
    ld.add_action(use_gazebo_arg)
    ld.add_action(container_arg)
    ld.add_action(include_camera_launch)
    ld.add_action(delayed_spawn)

    return ld
    