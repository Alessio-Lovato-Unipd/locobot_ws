"""
@file state_machine.launch.py
@brief Launch file that launches the state machine node.
@param debug - Enable debug mode(default: true)
"""



from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare the debug argument
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Enable debug mode'
    )

    # Declare the human following argument
    human_following_arg = DeclareLaunchArgument(
        'human_following',
        default_value='true',
        description='Enable human following after the first position'
    )

    state_machine = Node(
        package='simulation',
        executable='state_machine',
        output='screen',
        # Remapping is mandatory due to the namespace of interbotix packages
        remappings=[
            ('robot_description', '/locobot/robot_description'),
            ('robot_description_semantic', '/locobot/robot_description_semantic')
        ],
        parameters=[
            {
                'debug': LaunchConfiguration('debug'),
                'follow_human': LaunchConfiguration('human_following')
            }
        ]
    )

    return LaunchDescription([debug_arg,
                              human_following_arg,
                              state_machine])