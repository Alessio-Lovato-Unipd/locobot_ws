from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction


def launch_setup(context, *args, **kwargs):
    package = FindPackageShare('simulation')
    nav2_bringup_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([package, 'maps', 'simulation_map', 'simulation_map.yaml']),
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([package, 'config', 'navigation.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    nav2_bringup_launch = GroupAction(
        actions=[
            # Remap the cmd_vel topic published from 'Controller server' to the diffdrive_controller
            SetRemap(src='/cmd_vel', dst='/locobot/diffdrive_controller/cmd_vel_unstamped'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
                launch_arguments={
                    'map': map_yaml_file,
                    'params_file': params_file,
                    'use_sim_time': use_sim_time,
                }.items()
            )
        ])

    # locobot simulation launch
    gazebo_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("interbotix_xslocobot_sim"), 'launch', 'xslocobot_gz_classic.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'hardware_type': 'gz_classic',
            'use_base': 'true',
            'use_lidar': 'true',
            'use_rviz': 'false',
            'use_camera': 'true'
        }.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare("simulation"), 'rviz', 'navigation.rviz'])],
        output='screen'
    )

    return [
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            nav2_bringup_launch,
            gazebo_simulation_launch,
            rviz
        ]

    def generate_launch_description():
        declared_arguments = []
        return LaunchDescription(declared_arguments + launch_setup(context))