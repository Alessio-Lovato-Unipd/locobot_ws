"""
@file include_camera.launch.py

@brief This file is used to include the camera entities in gazebo simulation.

@param camera_number: Number of cameras to be included in the simulation. Can be 1 or 2. (default: 1)

@details The position of the camera is fixed in the environment and a static tf is
 published from camera 1 to camera 2 if the second one is included.
 Gazebo Classic must be running with ros plugins enabled before running this launch file.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    RegisterEventHandler,
    OpaqueFunction,
    Shutdown    
)
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


"""
@brief Function to create the launch description.
"""
def launch_setup(context):
    # Get the camera number
    number = LaunchConfiguration('camera_number').perform(context)

    # Validate the camera number
    if number not in ['1', '2']:
        print('[ERROR] [launch]: Invalid value for camera_number. It must be 1 or 2.')
        return [Shutdown(reason='Invalid value for camera_number')]

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
        condition=IfCondition(PythonExpression(['"', number, '" == "2"'])),
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

    return [spawn_camera_1, tf_camera_1, spawn_camera_2]


"""
@brief Main function
"""
def generate_launch_description():
    # Declare the launch arguments
    camera_number_arg = DeclareLaunchArgument(
        'camera_number',
        default_value='1',
        description='Number of cameras to be included in the simulation. Can be 1 or 2.',
    )

    namespace = PushRosNamespace('')
    ld = LaunchDescription([camera_number_arg,
                            namespace])
    ld.add_action(OpaqueFunction(function=launch_setup))

    # Return the launch description
    return ld