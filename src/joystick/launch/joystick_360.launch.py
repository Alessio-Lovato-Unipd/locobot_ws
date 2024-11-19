
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   
    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[{'dev': '/dev/input/js0'}, {'autorepeat_rate': 10.0}],
        respawn=True)

    talker_node = Node(
            package='joystick',
            executable='talker_360',
            name='talker_360')
    
    vel_node = Node(
            package='joystick',
            executable='joy2cmdvel',
            name='joy2vel')
    
    return LaunchDescription([joystick_node, talker_node, vel_node])