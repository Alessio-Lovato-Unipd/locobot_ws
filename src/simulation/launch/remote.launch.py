from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    GroupAction,
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution
)


def generate_launch_description():

############################################################################################################
############################################  APRILTAG  ####################################################
############################################################################################################

    # Create apriltag node for camera 1
    apriltag_node_1 = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node_1',
        parameters= [os.path.join(get_package_share_directory('simulation'), 'config', 'apriltag.yaml'),
                        {'publish_tag_detections_image': 'True'}],
        remappings=[('camera_info', '/rgb/camera_info'),
                    ('image_rect', '/rgb/image_raw'),
                    ('/tf', '/tf_marker')],
    )
        
        # Note to publish a static tf to link the marker to the locobot

    apriltag_static_broadcaster = Node(
        package='simulation',
        executable='tf_publisher',
        name='locobot_tag_static_tf_broadcaster',
        parameters=[{'frame_id':'locobot/pan_link'},
                    {'child_frame_id':'locobot_tag'},
                    {'publish_frequency': 20.0},
                    {'x':-0.10},
                    {'y':0.0},
                    {'z':0.1},
                    {'roll':0.0},
                    {'pitch':0.0},
                    {'yaw':-1.57079632679}],
        output='screen'
    )

    # Node that republish the tf from the apriltag_node_1 to the /tf topic of the locobot tag
    locobot_remapper = Node(
        package='simulation',
        executable='tf_remapper',
        name='tf_remapper_1',
        namespace='locobot/env_camera_1',
        output='screen',
        parameters=[{'source_frame':'rgb_camera_link'},
                    {'target_frame':'locobot_tag'},
                    {'publish_frequency': 20.0}],
        remappings=[('/tf', '/tf_marker'),
                    ('/locobot/env_camera_1/tf_locobot', '/tf')]
    )

    # Node that republish the tf from the apriltag_node_1 to the /tf topic of the human tag
    human_remapper = Node(
        package='simulation',
        executable='tf_remapper',
        name='human_remapper',
        namespace='locobot/env_camera_1',
        output='screen',
        parameters=[{'source_frame':'human_tag'},
                    {'target_frame':'rgb_camera_link'},
                    {'publish_frequency': 20.0}],,
        remappings=[('/tf', '/tf_marker'),
                    ('/locobot/env_camera_1/tf_locobot', '/tf')]
    )

############################################################################################################
############################################  RVIZ  ########################################################
############################################################################################################

    # RViz2 node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare("simulation"), 'rviz', 'navigation_3D.rviz']),
                   '--ros-args', '--log-level', 'warn'],
        output='screen'
    )

    tf_camera_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ['--x', '-0.303', '--y', '0.373', '--z', '3.071',
                     '--roll', '2.464', '--pitch', '-0.586', '--yaw', '-0.483', 
                     '--frame-id', 'rgb_camera_link',
                     '--child-frame-id', 'map'],
        output='screen'
    )

    return LaunchDescription([
        include_markers_launch,
        rviz,
        tf_camera_1
    ])