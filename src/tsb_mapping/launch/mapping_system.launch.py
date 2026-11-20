"""Inicia simulador + mapper (e, opcionalmente, Foxglove Bridge)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulator_config = str(
        Path(get_package_share_directory('tsb_simple_simulator')) / 'config' / 'simulator.yaml'
    )
    mapping_config = str(
        Path(get_package_share_directory('tsb_mapping')) / 'config' / 'mapping.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_foxglove', default_value='false',
                              description='Arrancar a Foxglove Bridge na porta interna 8765'),
        Node(
            package='tsb_simple_simulator',
            executable='simulator_node',
            name='simple_simulator',
            output='screen',
            parameters=[simulator_config],
        ),
        Node(
            package='tsb_mapping',
            executable='mapping_node',
            name='mapping_node',
            output='screen',
            parameters=[mapping_config],
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_foxglove')),
            parameters=[{'port': 8765}],
        ),
    ])
