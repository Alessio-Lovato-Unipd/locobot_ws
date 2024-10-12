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

def launch_setup(context, *args, **kwargs):
    # Get the value of the simulation argument
    simulation = LaunchConfiguration('simulation').perform(context)

    image_topic = ""

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
        image_topic = '/locobot/camera/camera/color/image_raw'

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
                {'minimum_score': 0.7}
            ]
    )

    nodes.append(recognizer)
    return nodes



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'simulation',
            default_value='false',
            description='Whether to launch the RealSense camera (true or false)'
        ),
        OpaqueFunction(function=launch_setup)
    ])