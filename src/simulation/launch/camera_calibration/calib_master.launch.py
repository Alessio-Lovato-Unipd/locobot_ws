import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    cameras = ['k01', 'k02']
    tags = []

    for camera in cameras:
        tags_yaml = os.path.join(get_package_share_directory(
            'framework_launch'), 'config', 'tags', camera + '_tags.yaml')
        with open(tags_yaml, 'r') as stream:
            yaml_data = yaml.safe_load(stream)
            tag_frames = yaml_data['tag_frames']
            tags = tags + tag_frames

    node = Node(
        package='hiros_apriltag_calibration',
        executable='hiros_apriltag_calibration',
        name='apriltag_calibration',
        namespace='hiros',
        output='screen',
        parameters=[
            {'max_pose_stdev': 0.005},
            {'global_reference_frame': 'map'},
            {'world_tag_id': 0},
            {'cameras': cameras},
            {'camera_frames': ['k01_rgb_camera_link', 'k02_rgb_camera_link']},
            {'tags': tags},
        ]
    )

    ld.add_action(node)
    return ld
