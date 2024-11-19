
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    simulation = LaunchDescription('simulation')
   
    use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='True',
        description='A simulation is being used')

    joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[{'dev': '/dev/input/js0'}, {'autorepeat_rate': 10.0}],
        respawn=True)

    talker_node = Node(
            package='joystick',
            executable='talker_PS3',
            name='talker_PS3')

    #vel_node = Node(
    #        package='joystick',
    #        executable='joy2cmdvel',
    #        name='joy2vel',
    #        remappings=[('/cmdvel_unstamped', '/locobot/diffdrive_controller/cmd_vel_unstamped')]
    #        )
    vel_node = Node(
            package='joystick',
            executable='joy2cmdvel',
            name='joy2vel',
            remappings=[('/cmdvel_unstamped', '/locobot/commands/velocity')]
           )
    
    return LaunchDescription([joystick_node, talker_node, vel_node])
