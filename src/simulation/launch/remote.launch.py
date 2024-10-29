import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    ld = LaunchDescription()

############################################################################################################
##########################################  CONFIGURATION  #################################################
############################################################################################################

      
    # Declare the 'container' launch configuration
    arg_container = DeclareLaunchArgument(
        name='container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    # Configuration file
    config_file = os.path.join(get_package_share_directory('simulation'), 'config', 'apriltag.yaml')

    # Camera
    camera = 'k02'

    # Topics
    image_raw_topic = 'rgb/image_raw'
    info_topic = 'rgb/camera_info'
    image_rect_topic = 'rgb/image_rect'

############################################################################################################
###############################################  NODES  ####################################################
############################################################################################################

    # RViz2 node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare("simulation"), 'rviz', 'navigation_3D.rviz']),
                   '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Static transform publisher for the map frame
    map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ['--x', '0.073', '--y', '0.335', '--z', '3.762',
                     '--roll', '-2.212', '--pitch', '-0.518', '--yaw', '-2.810', 
                     '--frame-id', 'rgb_camera_link',
                     '--child-frame-id', 'map'],
        output='screen'
    )

    # Node to publish a static tf to link the marker to the locobot base
    locobot_apriltag_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ['--x', '0.0', '--y', '0.0', '--z', '-0.63',
                     '--roll', '0.0', '--pitch', '0.0', '--yaw', '1.5707', 
                     '--frame-id', 'locobot_tag',
                     '--child-frame-id', 'locobot/base_footprint'],
        output='screen'
    )

    # Azure Kinect ROS driver 
    kinectNode = Node(
        package='azure_kinect_ros_driver',
        executable='node',
        output='screen',
        namespace=camera,
        parameters=[
            {'sensor_sn': '000071601712'}, # Serial number of the camera 'k02'
            {'depth_enabled': False},
            {'rgb_point_cloud': False},
            {'color_enabled': True},
            {'color_resolution': '1080P'},
            {'fps': 30},
            {'point_cloud': False},
        ]
    )

    # Node composition of the image_proc node and the apriltag_ros node
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_rgb_node',
            namespace=camera + '/rgb',
            remappings=[  
                ('image', 'image_raw')]
        ),
        ComposableNode(
            name=LaunchConfiguration('container'),
            namespace=camera + '/rgb',
            package='apriltag_ros',
            plugin='AprilTagNode',
            parameters=[config_file],
            remappings=[('image', 'image_raw')], #TODO: Change to image_rect when the rectification is working
        )
    ]

    # If an existing container is not provided, start a container and load nodes into it
    apriltag_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='apriltag_container',
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


    ld.add_action(arg_container)
    ld.add_action(rviz)
    ld.add_action(map_tf)
    ld.add_action(locobot_apriltag_tf)
    ld.add_action(kinectNode)
    ld.add_action(apriltag_container)
    ld.add_action(load_composable_nodes)

    return ld
