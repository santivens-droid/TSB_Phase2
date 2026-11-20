"""Inicia o simulador 2D com os parâmetros de config/simulator.yaml."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = str(Path(get_package_share_directory('tsb_simple_simulator')) / 'config' / 'simulator.yaml')
    return LaunchDescription([
        Node(
            package='tsb_simple_simulator',
            executable='simulator_node',
            name='simple_simulator',
            output='screen',
            parameters=[config],
        )
    ])
