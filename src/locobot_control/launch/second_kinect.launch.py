
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution

def setup_nodes(context, *args, **kwargs):
    # Load the kinect configuration file
    camera_namespace = "k02"
    kinect_node_params = {
        'sensor_sn': LaunchConfiguration('cam_sn'), # Serial number of the camera 'k02'
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

    # If an existing container is not provided, start a container and load nodes into it
    apriltag_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='apriltag_container_2',
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

    return [
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

    apriltag_config_file = DeclareLaunchArgument(
        name='apriltag_config_file',
        default_value=PathJoinSubstitution([FindPackageShare('locobot_control'), 'config', 'apriltag.yaml']),
        description='Full path to the apriltag configuration file to use'
    )

    camera_sn_arg = DeclareLaunchArgument(
        name='cam_sn', default_value='a000191301712',
        description='Serial number of the camera k02'
    )

    ld.add_action(arg_container)
    ld.add_action(apriltag_config_file)
    ld.add_action(camera_sn_arg)
    ld.add_action(OpaqueFunction(function=setup_nodes))

    return ld