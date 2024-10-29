from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    
    demo = Node(
        package='simulation',
        executable='demo',
        output='screen',
       # namespace='locobot',
        # Remapping is mandatory due to the namespace
        remappings=[
            ('robot_description', '/locobot/robot_description'),
            ('robot_description_semantic', '/locobot/robot_description_semantic')
        ]
    )

    return LaunchDescription([demo])