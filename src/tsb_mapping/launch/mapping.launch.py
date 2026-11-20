"""Inicia apenas o mapper. Requer um /odom e /scan em funcionamento."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('tsb_mapping')) / 'config' / 'mapping.yaml')
    return LaunchDescription([
        Node(
            package='tsb_mapping',
            executable='mapping_node',
            name='mapping_node',
            output='screen',
            parameters=[config],
        )
    ])
