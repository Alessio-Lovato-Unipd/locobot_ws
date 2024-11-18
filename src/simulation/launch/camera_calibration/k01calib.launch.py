import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, conditions
from launch.actions import (DeclareLaunchArgument, GroupAction)
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

import launch.actions
import launch_ros.actions

import yaml

def generate_launch_description():
    ld = LaunchDescription()

    camera = 'k01'

    image_topic = 'rgb/image_rect'
    info_topic = 'rgb/camera_info'

    tag_config = os.path.join(
        get_package_share_directory('simulation'), 'config/camera_calibration', 'tags_36h11_all.yaml'
    )
    tags = os.path.join(
        get_package_share_directory('simulation'), 'config/camera_calibration', camera + '_tags.yaml'
    )

    tag_node = ComposableNode(
            name=['apriltag'],
            namespace=camera,
            package='apriltag_ros',
            plugin='AprilTagNode',
            parameters=[tag_config, tags],
            remappings=[('image', image_topic), ('camera_info', info_topic)],
    )

    tag_container = ComposableNodeContainer(
            name='tag_container',
            namespace=['apriltag_', camera],
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[tag_node],
            output='screen',
    )

    launch_include_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(TextSubstitution(text=camera)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('framework_launch'),
                        'launch/k4a.launch.py'
                    )
                ),
                launch_arguments={
                    'depth_enabled': 'true',
                    'depth_mode': 'NFOV_UNBINNED',
                    'depth_unit': '16UC1',
                    'color_enabled': 'true',
                    'color_format': 'bgra',
                    'color_resolution': '1080P',
                    'fps': '30',
                    'point_cloud': 'true',
                    'rgb_point_cloud': 'true',
                    'point_cloud_in_depth_frame': 'true',
                    'required': 'false',
                    'sensor_sn': 'a000181401712',
                    'recording_file': '""',
                    'recording_loop_enabled': 'false',
                    'body_tracking_enabled': 'false',
                    'body_tracking_smoothing_factor': '0.0',
                    'rescale_ir_to_mono8': 'false',
                    'ir_mono8_scaling_factor': '1.0',
                    'imu_rate_target': '0',
                    'wired_sync_mode': '0',
                    'subordinate_delay_off_master_usec': '0',
                    'tf_prefix': camera + '_',
                }.items()
            ),
        ]
    )

    ld.add_action(tag_container)
    ld.add_action(launch_include_with_namespace)

    return ld
