# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, conditions
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

import launch.actions
import launch_ros.actions

def generate_launch_description():
    ld = LaunchDescription([
    DeclareLaunchArgument(
        'overwrite_robot_description',
        default_value="true" ,
        description="Flag to publish a standalone azure_description instead of the default robot_description parameter."),
    ##############################################
    DeclareLaunchArgument(
        'depth_enabled',
        default_value="true",
        description="Enable or disable the depth camera"),
    DeclareLaunchArgument(
        'depth_mode',
        default_value="NFOV_UNBINNED",
        description="Set the depth camera mode, which affects FOV, depth range, and camera resolution. See Azure Kinect documentation for full details. Valid options: NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_UNBINNED, WFOV_2X2BINNED, and PASSIVE_IR"),
    DeclareLaunchArgument(
        'depth_unit',
        default_value="16UC1",
        description='Depth distance units. Options are: "32FC1" (32 bit float metre) or "16UC1" (16 bit integer millimetre)'),
    DeclareLaunchArgument(
        'color_enabled',
        default_value="true",
        description="Enable or disable the color camera"),
    DeclareLaunchArgument(
        'color_format',
        default_value="bgra",
        description="The format of RGB camera. Valid options: bgra, jpeg"),
    DeclareLaunchArgument(
        'color_resolution',
        default_value="1536P",
        description="Resolution at which to run the color camera. Valid options: 720P, 1080P, 1440P, 1536P, 2160P, 3072P"),
    DeclareLaunchArgument(
        'fps',
        default_value="30",
        description="FPS to run both cameras at. Valid options are 5, 15, and 30"),
    DeclareLaunchArgument(
        'point_cloud',
        default_value="true",
        description="Generate a point cloud from depth data. Requires depth_enabled"),
    DeclareLaunchArgument(
        'rgb_point_cloud',
        default_value="true",
        description="Colorize the point cloud using the RBG camera. Requires color_enabled and depth_enabled"),
    DeclareLaunchArgument(
        'point_cloud_in_depth_frame',
        default_value="true",
        description="Whether the RGB pointcloud is rendered in the depth frame (true) or RGB frame (false). Will either match the resolution of the depth camera (true) or the RGB camera (false)."),
    DeclareLaunchArgument( # Not a parameter of the node, rather a launch file parameter
        'required',
        default_value="false",
        description="Argument which specified if the entire launch file should terminate if the node dies"),
    DeclareLaunchArgument(
        'sensor_sn',
        default_value="''",
        description="Sensor serial number. If none provided, the first sensor will be selected"),
    DeclareLaunchArgument(
        'tf_prefix',
        default_value="",
        description="The prefix prepended to tf frame ID's"),
    DeclareLaunchArgument(
        'recording_file',
        default_value="''",
        description="Absolute path to a mkv recording file which will be used with the playback api instead of opening a device"),
    DeclareLaunchArgument(
        'recording_loop_enabled',
        default_value="false",
        description="If set to true the recording file will rewind the beginning once end of file is reached"),
    DeclareLaunchArgument(
        'body_tracking_enabled',
        default_value="false",
        description="If set to true the joint positions will be published as marker arrays"),
    DeclareLaunchArgument(
        'body_tracking_smoothing_factor',
        default_value="0.0",
        description="Set between 0 for no smoothing and 1 for full smoothing"),
    DeclareLaunchArgument(
        'rescale_ir_to_mono8',
        default_value="false",
        description="Whether to rescale the IR image to an 8-bit monochrome image for visualization and further processing. A scaling factor (ir_mono8_scaling_factor) is applied."),
    DeclareLaunchArgument(
        'ir_mono8_scaling_factor',
        default_value="1.0",
        description="Scaling factor to apply when converting IR to mono8 (see rescale_ir_to_mono8). If using illumination, use the value 0.5-1. If using passive IR, use 10."),
    DeclareLaunchArgument(
        'imu_rate_target',
        default_value="0",
        description="Desired output rate of IMU messages. Set to 0 (default) for full rate (1.6 kHz)."),
    DeclareLaunchArgument(
        'wired_sync_mode',
        default_value="0",
        description="Wired sync mode. 0: OFF, 1: MASTER, 2: SUBORDINATE."),
    DeclareLaunchArgument(
        'subordinate_delay_off_master_usec',
        default_value="0",
        description="Delay subordinate camera off master camera by specified amount in usec."),
    ])

    k4a = Node(
        package='azure_kinect_ros_driver',
        executable='node',
        output='screen',
        parameters=[
            {'required': launch.substitutions.LaunchConfiguration('required')},
            {'depth_enabled': launch.substitutions.LaunchConfiguration('depth_enabled')},
            {'depth_mode': launch.substitutions.LaunchConfiguration('depth_mode')},
            {'depth_unit': launch.substitutions.LaunchConfiguration('depth_unit')},
            {'color_enabled': launch.substitutions.LaunchConfiguration('color_enabled')},
            {'color_format': launch.substitutions.LaunchConfiguration('color_format')},
            {'color_resolution': launch.substitutions.LaunchConfiguration('color_resolution')},
            {'fps': launch.substitutions.LaunchConfiguration('fps')},
            {'point_cloud': launch.substitutions.LaunchConfiguration('point_cloud')},
            {'rgb_point_cloud': launch.substitutions.LaunchConfiguration('rgb_point_cloud')},
            {'point_cloud_in_depth_frame': launch.substitutions.LaunchConfiguration('point_cloud_in_depth_frame')},
            {'sensor_sn': launch.substitutions.LaunchConfiguration('sensor_sn')},
            {'tf_prefix': launch.substitutions.LaunchConfiguration('tf_prefix')},
            {'recording_file': launch.substitutions.LaunchConfiguration('recording_file')},
            {'recording_loop_enabled': launch.substitutions.LaunchConfiguration('recording_loop_enabled')},
            {'body_tracking_enabled': launch.substitutions.LaunchConfiguration('body_tracking_enabled')},
            {'body_tracking_smoothing_factor': launch.substitutions.LaunchConfiguration('body_tracking_smoothing_factor')},
            {'rescale_ir_to_mono8': launch.substitutions.LaunchConfiguration('rescale_ir_to_mono8')},
            {'ir_mono8_scaling_factor': launch.substitutions.LaunchConfiguration('ir_mono8_scaling_factor')},
            {'imu_rate_target': launch.substitutions.LaunchConfiguration('imu_rate_target')},
            {'wired_sync_mode': launch.substitutions.LaunchConfiguration('wired_sync_mode')},
            {'subordinate_delay_off_master_usec': launch.substitutions.LaunchConfiguration('subordinate_delay_off_master_usec')}
        ])

    ###########
    # Rectify #
    ###########
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_rgb_node',
            remappings=[
                ('image', 'rgb/image_raw'),
                ('camera_info', 'rgb/camera_info'),
                ('image_rect', 'rgb/image_rect'),
                ('image_rect/compressed', 'rgb/image_rect/compressed'),
                ('image_rect/compressedDepth', 'rgb/image_rect/compressedDepth'),
                ('image_rect/theora', 'rgb/image_rect/theora'),
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_rgb_to_depth_node',
            remappings=[
                ('image', 'rgb_to_depth/image_raw'),
                ('camera_info', 'rgb_to_depth/camera_info'),
                ('image_rect', 'rgb_to_depth/image_rect'),
                ('image_rect/compressed', 'rgb_to_depth/image_rect/compressed'),
                ('image_rect/compressedDepth', 'rgb_to_depth/image_rect/compressedDepth'),
                ('image_rect/theora', 'rgb_to_depth/image_rect/theora'),
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_depth_node',
            remappings=[
                ('image', 'depth/image_raw'),
                ('camera_info', 'depth/camera_info'),
                ('image_rect', 'depth/image_rect'),
                ('image_rect/compressed', 'depth/image_rect/compressed'),
                ('image_rect/compressedDepth', 'depth/image_rect/compressedDepth'),
                ('image_rect/theora', 'depth/image_rect/theora'),
            ],
        ),
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_depth_to_rgb_node',
            remappings=[
                ('image', 'depth_to_rgb/image_raw'),
                ('camera_info', 'depth_to_rgb/camera_info'),
                ('image_rect', 'depth_to_rgb/image_rect'),
                ('image_rect/compressed', 'depth_to_rgb/image_rect/compressed'),
                ('image_rect/compressedDepth', 'depth_to_rgb/image_rect/compressedDepth'),
                ('image_rect/theora', 'depth_to_rgb/image_rect/theora'),
            ],
        ),
    ]

    arg_container = DeclareLaunchArgument(
        name='container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_proc_container',
        namespace='',
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

    ld.add_action(k4a)
    ld.add_action(arg_container)
    ld.add_action(image_processing_container)
    ld.add_action(load_composable_nodes)
    return ld
