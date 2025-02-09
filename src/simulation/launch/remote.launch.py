"""
@file remote.launch.py

@brief This file is used to launch the simulation environment with the Azure Kinect camera and the apriltag node.

@details The launch file will start the Azure Kinect ROS driver, the image_proc node, the apriltag node, and the RViz2 node. The apriltag node will detect the apriltags in the camera feed and publish the pose of the detected tags.
The RViz2 node will display the camera feed and the detected apriltags. If the 'only_camera' argument is set to 'true', only the camera and apriltag node will be launched.

@note The apriltag node is a composable node that is loaded into a container. The container is created if the 'container' argument is not set. If the 'container' argument is set, the apriltag node is loaded into the existing container.

@param container: Name of an existing node container to load launched nodes into. If unset, a new container will be created.
@param only_camera: If set to 'true', only the camera and apriltag node will be launched.
@param apriltag_config_file: Full path to the apriltag configuration file to use.
@param kinect_config_file: Full path to the kinect configuration file to use.

"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def load_yaml(context, *args, **kwargs):
    absolute_file_path = LaunchConfiguration('kinect_config_file').perform(context)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def setup_nodes(context, *args, **kwargs):
    # Load the kinect configuration file
    kinect_params = load_yaml(context)
    camera_namespace = "k02"
    kinect_node_params = {
        'sensor_sn': 'a000181401712',  # Serial number of the camera 'k02'
        'depth_enabled': True,  # If set to false, the depth frame is rotated wrongly
        'rgb_point_cloud': False,
        'color_enabled': True,
        'color_resolution': '1080P',
        'fps': 30,
        'point_cloud': False,
        'required': True
    }

    # Topics
    image_raw_topic = 'rgb/image_raw'
    info_topic = 'rgb/camera_info'
    image_rect_topic = 'rgb/image_rect'

    # Azure Kinect ROS driver 
    kinectNode = Node(
        package='azure_kinect_ros_driver',
        executable='node',
        output='screen',
        name='kinect_node',
        namespace=camera_namespace,
        parameters=[kinect_node_params,
                    {'tf_prefix': camera_namespace + '_'}],
    )

    # Node composition of the image_proc node and the apriltag_ros node
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            namespace = camera_namespace + '/rgb',
            name='rectify_rgb_node',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect'),
                ('image_rect/compressed', 'image_rect/compressed'),
                ('image_rect/compressedDepth', 'image_rect/compressedDepth'),
                ('image_rect/theora', 'image_rect/theora'),
            ],
        ),
        ComposableNode(
            name=LaunchConfiguration('container'),
            namespace=camera_namespace + '/rgb',
            package='apriltag_ros',
            plugin='AprilTagNode',
            parameters=[LaunchConfiguration('apriltag_config_file')],
            remappings=[('image', 'image_raw')], #TODO: Change to image_rect when the rectification is working
        )
    ]

    # Node to publish a static tf to link the marker to the locobot base
    locobot_apriltag_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="locobot_tag_static_tf_broadcaster",
        arguments = ['--x', '0.0', '--y', '0.0', '--z', '-0.70',
                     '--roll', '0.0', '--pitch', '0.0', '--yaw', '1.5707', 
                     '--frame-id', 'locobot_tag',
                     '--child-frame-id', 'locobot/base_footprint'],
        output='screen'
    )

    # If an existing container is not provided, start a container and load nodes into it
    apriltag_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='apriltag_container',
        namespace=camera_namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )

    # RViz2 node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare("simulation"), 'rviz', 'navigation.rviz']),
                   '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return [
        rviz,
        locobot_apriltag_tf,
        kinectNode,
        apriltag_container,
        load_composable_nodes
    ]

def generate_launch_description():

    ld = LaunchDescription()

    # Declare the 'container' launch configuration
    arg_container = DeclareLaunchArgument(
        name='container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    camera_number_arg = DeclareLaunchArgument(
        name='camera_number', default_value='1',
        choices=['1', '2'],
        description='Select the number of cameras to launch'
    )

    apriltag_config_file = DeclareLaunchArgument(
        name='apriltag_config_file',
        default_value=PathJoinSubstitution([FindPackageShare('simulation'), 'config', 'apriltag.yaml']),
        description='Full path to the apriltag configuration file to use'
    )

    kinect_config_file = DeclareLaunchArgument(
        name='kinect_config_file',
        default_value=PathJoinSubstitution([FindPackageShare('simulation'), 'config', 'kinect.yaml']),
        description='Full path to the kinect configuration file to use'
    )

    second_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("simulation"), 'launch', 'second_kinect.launch.py'])),
        condition=LaunchConfigurationEquals('camera_number', '2')
    )

    ld.add_action(arg_container)
    ld.add_action(camera_number_arg)
    ld.add_action(apriltag_config_file)
    ld.add_action(kinect_config_file)
    ld.add_action(OpaqueFunction(function=setup_nodes))
    ld.add_action(second_camera)

    return ld