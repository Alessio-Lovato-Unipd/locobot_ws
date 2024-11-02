
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
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
    camera_namespace = kinect_params['kinect_node']['ros__parameters']['camera_namespace']

    # Extract camera_tf parameters
    camera_tf_params = kinect_params.get('camera_tf', {}).get('ros__parameters', {})
    camera_tf_args = []
    for param in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
        camera_tf_args.extend([f'--{param}', str(camera_tf_params.get(param, '0.0'))])
    camera_tf_args.extend([
        '--frame-id', camera_namespace + '/' + camera_tf_params.get('frame_id', 'camera_link'),
        '--child-frame-id', camera_tf_params.get('child_frame_id', 'second_kinect_link')
    ])

    # Topics
    image_raw_topic = 'rgb/image_raw'
    info_topic = 'rgb/camera_info'
    image_rect_topic = 'rgb/image_rect'

    # Static transform publisher for the camera frame
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_tf",
        arguments=camera_tf_args,
        output='screen'
    )

    # Azure Kinect ROS driver 
    kinectNode = Node(
        package='azure_kinect_ros_driver',
        executable='node',
        output='screen',
        name='kinect_node',
        namespace=camera_namespace,
        parameters=[kinect_params['kinect_node']['ros__parameters'],
                    {'tf_prefix': camera_namespace + '/'}],
    )

    # Node composition of the image_proc node and the apriltag_ros node
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_rgb_node',
            namespace=camera_namespace + '/rgb',
            remappings=[('image', 'image_raw')]
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

    return [
        camera_tf,
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
        default_value=PathJoinSubstitution([FindPackageShare('simulation'), 'config', 'apriltag.yaml']),
        description='Full path to the apriltag configuration file to use'
    )

    kinect_config_file = DeclareLaunchArgument(
        name='kinect_config_file',
        default_value=PathJoinSubstitution([FindPackageShare('simulation'), 'config', 'kinect2.yaml']),
        description='Full path to the kinect configuration file to use'
    )

    ld.add_action(arg_container)
    ld.add_action(apriltag_config_file)
    ld.add_action(kinect_config_file)
    ld.add_action(OpaqueFunction(function=setup_nodes))

    return ld