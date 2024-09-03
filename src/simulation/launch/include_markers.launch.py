"""
@file include_markers.launch.py

@brief This file is used to include the fiducial marker and camera entities and in gazebo simulation 
 and publish the corresponding tf frames. It also launches a Gazebo Classic simulation if the 'use_gazebo'
  argument is set to 'True'.

@param camera_number: Number of cameras to be included in the simulation. Can be 1 or 2. (default: 1)

@note Please refer to file 'include_camera.launch.py' to understand the limitations of the
    camera entities in the simulation.
"""
import os
from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch.event_handlers import OnProcessExit
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

"""
@brief Function to create the launch description.
"""
def launch_setup(context):
    # Get the camera number
    number = LaunchConfiguration('camera_number').perform(context)

    # Validate the camera number (See note in the file description)
    if number not in ['1', '2']:
        print('[ERROR] [launch]: Invalid value for camera_number. It must be 1 or 2.')
        return [Shutdown(reason='Invalid value for \'camera_number\'')]
    
    # Load 'include_camera.launch.py' to include the camera entities
    include_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('simulation'), 'launch', 'include_camera.launch.py')),
        launch_arguments={'camera_number': number}.items()
    )

    # Launch apriltag tag entity in gazebo
    spawn_apriltag = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_map_tag',
        arguments=['-entity', 'map_tag', '-file',
                        PathJoinSubstitution([FindPackageShare('simulation'), 'models', 'map_tag', 'model.sdf']),
                        '-x', '0.0', '-y', '-0.5', '-z', '0.01', # z = 0.01 because the model 'visual' is 1cm tall
                        '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output='screen'
    )

    # Create apriltag node for camera 1
    apriltag_node_1 = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node_1',
        parameters= [os.path.join(get_package_share_directory('simulation'), 'config', 'apriltag.yaml')],
        remappings=[('/camera_info', '/env_camera_1/camera_info'),
                    ('/image_rect', '/env_camera_1/image_raw'),
                    ('/tf', '/tf_marker')],
    )

    # Create apriltag node for camera 2
    apriltag_node_2 = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node_2',
        parameters= [os.path.join(get_package_share_directory('simulation'), 'config', 'apriltag.yaml')],
        remappings=[('/camera_info', '/env_camera_2/camera_info'),
                    ('/image_rect', '/env_camera_2/image_raw'),
                    ('/tf', '/tf_marker')],
        condition=IfCondition(PythonExpression(['"', number, '" == "2"']))
    )

    # Static broadcaster for the apriltag marker above the locobot
    apriltag_static_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='locobot_tag_static_tf_broadcaster',
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

    # Static broadcaster for the marker used to localize the map frame
    map_static_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_tag_static_tf_braodcaster',
        arguments=[
            '--x', '-0.5',
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
    
    # Node that republish the tf from the apriltag_node_1 to the /tf topic
    tf_remapper = Node(
        package='simulation',
        executable='tf_remapper',
        name='tf_remapper_1',
        namespace='env_camera_1',
        output='screen',
        parameters=[{'use_sim_time':True},
                    {'camera_frame':'env_camera_1_frame'},
                    {'look_for_map_tag': True}],
        remappings=[('/tf', '/tf_marker'),
                    ('/env_camera_1/tf_locobot', '/tf')]
    )

    # Node that republish the tf from the apriltag_node_2 to the /tf topic
    tf_remapper_2 = Node(
        package='simulation',
        executable='tf_remapper',
        name='tf_remapper_2',
        namespace='env_camera_2',
        output='screen',
        parameters=[{'use_sim_time':True},
                    {'camera_frame':'env_camera_2_frame'}],
        remappings=[('/tf', '/tf_marker'),
                    ('/env_camera_2/tf_locobot', '/tf')],
        condition=IfCondition(PythonExpression(['"', number, '" == "2"']))
    )

    # Create event handlers to create a proper sequence of actions

    delayed_spawn_apriltag = TimerAction(
        period=2.0, #Delay in seconds
        actions=[spawn_apriltag]
    )

    wait_apriltag = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_apriltag,
            on_exit=[apriltag_static_broadcaster, map_static_broadcaster,
                     tf_remapper, tf_remapper_2, apriltag_node_1, apriltag_node_2]
        )
    )    

    # Return the actions
    return [include_camera_launch, delayed_spawn_apriltag, wait_apriltag]


"""
@brief Function to launch Gazebo Classic simulator

@note If the 'use_gazebo' argument is set to 'True', this function will set the necessary environment variables
    for the models to be found and launch the Gazebo Classic simulator.
"""
def launch_gazebo(context):
    use_gazebo = LaunchConfiguration('use_gazebo').perform(context)
    if use_gazebo == 'True' or use_gazebo == 'true':
        # Define Gazebo classic environment variables (necessary for the models to be found)
        # Theese variables are necessary for the models and has to be set before the gazebo launch
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

        gazebo_launch_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ]),
            ])
        )

        return [SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', value=resources_path),
                SetEnvironmentVariable('GAZEBO_MODEL_PATH', value=models_path),
                gazebo_launch_include]

    return []


"""
@brief Main function.
"""
def generate_launch_description():
    # Declare the launch arguments
    camera_number_arg = DeclareLaunchArgument(
        'camera_number',
        default_value='1',
        description='Number of cameras to be included in the simulation. Can be 1 or 2.',
    )
    
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='False',
        description='Flag to include the Gazebo Classic simulation.',
    )

    namespace = PushRosNamespace('')

    ld = LaunchDescription([camera_number_arg,
                            use_gazebo_arg,
                            namespace])

    ld.add_action(OpaqueFunction(function=launch_gazebo))
    ld.add_action(OpaqueFunction(function=launch_setup))

    # Return the launch description
    return ld
    