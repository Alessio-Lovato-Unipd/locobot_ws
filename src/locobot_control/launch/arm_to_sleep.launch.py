from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from interbotix_xs_modules.xs_launch import (
    construct_interbotix_xslocobot_semantic_robot_description_command,
    declare_interbotix_xslocobot_robot_description_launch_arguments,
    determine_use_sim_time_param,
)

def generate_launch_description():

    arm_sleep_position = Node(
        package='locobot_control',
        executable='arm_sleep_position',
        output='screen',
       # namespace='locobot',
        # Remapping is mandatory due to the namespace
        remappings=[
            ('robot_description', '/locobot/robot_description'),
            ('robot_description_semantic', '/locobot/robot_description_semantic')
        ]
    )

    return LaunchDescription([arm_sleep_position])