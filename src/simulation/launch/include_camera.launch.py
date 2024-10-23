"""
@file include_camera.launch.py

@brief This file is used to include the camera entities in gazebo simulation.

@param camera_number: Number of cameras to be included in the simulation. Can be 1 or 2. (default: 1)
@param use_gazebo: Flag to include the Gazebo Classic simulation. Can be true or false. (default: false)

@details The position of the camera is fixed in the environment and a static tf is
 published from camera 1 to camera 2 if the second one is included.
 Gazebo Classic must be running with ros plugins enabled before running this launch file.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    SetEnvironmentVariable,
    OpaqueFunction,
    IncludeLaunchDescription
)

from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


"""
@brief Function to launch Gazebo Classic simulator

@note If the 'use_gazebo' argument is set to 'True', this function will set the necessary environment variables
    for the models to be found and launch the Gazebo Classic simulator.
"""
def launch_gazebo(context):
    if LaunchConfigurationEquals('use_gazebo', 'true').evaluate(context):
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
    else:
        return []


"""
@brief Function to create the launch description.
"""
def generate_launch_description():

    # Create the launch description
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

    # Get the path to the gazebo_ros package
    camera_model_path = PathJoinSubstitution([FindPackageShare("simulation"), 
                                              'models', 'camera_1', 'model.sdf'])

    # Create the camera entities
    spawn_camera_1 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_camera_1",
        arguments=["-entity", "env_camera_1", "-file", camera_model_path, 
                   '-x', '0.0', '-y', '1.0', '-z', '3.0',
                   '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        output="screen"
    )

    # Publish static tf from camera 1 to map
    tf_camera_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ['--x', '1.0', '--y', '0.0', '--z', '3.0',
                     '--yaw', '-1.57079632', '--pitch', '0.0','--roll', '-3.14159265', 
                     '--frame-id', 'env_camera_1_frame',
                     '--child-frame-id', 'map'],
        output='screen'
    )

    # Get the path to the model of the second camera
    camera_model_path = PathJoinSubstitution([FindPackageShare("simulation"), 
                                              'models', 'camera_2', 'model.sdf'])

    # Create the group action to spawn the second camera
    spawn_camera_2 = GroupAction(
        condition=LaunchConfigurationEquals('camera_number', '2'),
        actions=[
            # Spawn the second camera
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_camera_2",
                arguments=["-entity", "env_camera_2", "-file", camera_model_path, 
                        '-x', '-2.0', '-y', '3.2', '-z', '3.0',
                        '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
                output="screen",
            ),

            # Create the static tf publisher
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments = ['--x', '2.2', '--y', '-2.0', '--z', '0.0',
                             '--yaw', '0.0', '--pitch', '0','--roll', '0', 
                             '--frame-id', 'env_camera_2_frame',
                             '--child-frame-id', 'env_camera_1_frame'],
                output='screen'
            )
        ]
    ) 

    # Add the actions to the launch description
    ld.add_action(camera_number_arg)
    ld.add_action(use_gazebo_arg)
    ld.add_action(OpaqueFunction(function=launch_gazebo))
    ld.add_action(spawn_camera_1)
    ld.add_action(tf_camera_1)
    ld.add_action(spawn_camera_2)

    return ld