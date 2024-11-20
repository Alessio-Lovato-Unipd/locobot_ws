"""
@file gesture_recognizer.launch.py

@brief This file launches the gesture recognizer node and the realsense carmera node if the simulation argument is set to true.

@details
The launch file first checks if the minimum score is between 0 and 1. If not, a ValueError is raised. 
Then, it launches the RealSense camera node and the gesture recognizer node. 
The RealSense camera node is launched if the simulation argument is set to true.

@param simulation Whether to launch the RealSense camera (true or false)
@param minimum_score Minimum score for a gesture to be recognized. Must be between 0 and 1.
@param camera_topic The image topic to subscribe to (not available in simulation)
@param service_name The name of the service to connect with. If null, the service client won't be used. In simulation the service is not used.
@param rotate_image Flag to decide if the image from the camera should be flipped horizontally before being analyzed. If true (default), gestures must be performed towards down.

@note The 'camera_topic' argument is not available in simulation since the image topic is already known.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import sys
import pathlib
import os
from ament_index_python.packages import get_package_share_directory
import subprocess
import re

sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

# Function to get the serial numbers of the RealSense cameras connected to the computer
def get_realsense_serial_numbers():
    # Execute the command `rs-enumerate-devices -s` and capture the output
    result = subprocess.run(['rs-enumerate-devices', '-s'], stdout=subprocess.PIPE)
    output = result.stdout.decode('utf-8')

    # Use a regular expression to extract the serial numbers from the output
    serial_numbers = re.findall(r'Intel RealSense \w+\s+(\d+)\s+', output)
    
    return serial_numbers

"""
@brief This function checks if the minimum score is between 0 and 1. If not, a ValueError is raised.
"""
def check_minimum_score(context, *args, **kwargs):
    minimum_score = float(LaunchConfiguration('minimum_score').perform(context))
    if not (0 <= minimum_score <= 1):
        raise ValueError(f"Minimum score must be between 0 and 1. Provided value: {minimum_score}")


"""
@brief This function launches the RealSense camera node and the gesture recognizer node. The RealSense camera node is launched if the simulation argument is set to true.
"""
def launch_setup(context, *args, **kwargs):
    # Get the value of the simulation argument
    simulation = LaunchConfiguration('simulation').perform(context)

    image_topic = ""
    service_name = ""

    nodes = []

    # Evaluate the simulation argument
    if simulation.lower() == 'true':
        # Get the serial number of the realsense cameras
        serial_numbers = get_realsense_serial_numbers()
        
        if len(serial_numbers) == 0:
            print('No RealSense cameras found')
            return []
        
        image_topic = '/camera/color/image_raw'
        
        # Add the realsense camera launch file to the launch description
        camera = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
                    ),
                    launch_arguments={
                        'serial_number': serial_numbers[0],
                        'camera_name': 'camera',
                        'camera_namespace': ''
                    }.items()
        )

        nodes.append(camera)

    elif simulation.lower() == 'false':
        image_topic = LaunchConfiguration('camera_topic').perform(context)
        service_name = LaunchConfiguration('service_name').perform(context)

    else:
        print('Invalid value for simulation argument. Use true or false.')
        return []

    # Add the hand recognition node to the launch description
    recognizer = Node(
            package='gesture_recognition',
            executable='gesture_recognizer_node',
            name='gesture_recognizer_node',
            output='screen',
            parameters=[
                {'camera_topic': image_topic},
                {'minimum_score': float(LaunchConfiguration('minimum_score').perform(context))},
                {'service_name': service_name},
                {'rotate_image': LaunchConfiguration('rotate_image').perform(context) == 'true'}
            ]
    )

    nodes.append(recognizer)
    return nodes


"""
@brief This function generates the launch description. It declares the arguments simulation, minimum_score and camera topic.
"""
def generate_launch_description():
    # Declare the arguments
    simulation_arg = DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='Whether to launch the RealSense camera (true or false)'
    )

    minimum_score_arg = DeclareLaunchArgument(
            'minimum_score',
            default_value='0.5',
            description='Minimum score for a gesture to be recognized. Must be between 0 and 1.'
    )

    image_topic_arg = DeclareLaunchArgument(
            'camera_topic',
            default_value='/locobot/rs_camera/color/image_raw',
            description='The image topic to subscribe to (not available in simulation)'
    )

    service_name_arg = DeclareLaunchArgument(
            'service_name',
            default_value='state_control',
            description='The name of the service to connect with. If null, the service client won\'t be used. Not available in simulation.'
    )

    rotate_image_arg = DeclareLaunchArgument(
        'rotate_image',
        default_value='true',
        choices=['true', 'false'],
        description="Rotate the image 180° horizontally to perform gesturestowards down. Enhance ergonomics."
    )

    return LaunchDescription([
        simulation_arg,
        minimum_score_arg,
        image_topic_arg,
        service_name_arg,
        rotate_image_arg,
        OpaqueFunction(function=check_minimum_score), # Check if the minimum score is between 0 and 1
        OpaqueFunction(function=launch_setup)
    ])