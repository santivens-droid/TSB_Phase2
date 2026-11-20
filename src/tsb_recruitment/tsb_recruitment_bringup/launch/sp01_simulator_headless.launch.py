from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

#!/usr/bin/env python3



def generate_launch_description():
    """Generate launch description for launching SP01 simulator with state publisher and foxglove bridge without viewport."""

    # Simulation environment
    tsb_recruitment_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tsb_recruitment_sim'),
                'launch',
                'tsb_recruitment_sim.launch.py'
            ])
        ]),
        launch_arguments={'config_file': 'sp01_simple.json', 'show_viewport': 'false'}.items()
    )

    # SP01 state publisher
    sp01_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sp01_description'),
                'launch',
                'sp01_state_publisher.launch.py'
            ])
        ])
    )

    # Foxglove bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'send_buffer_limit': 100000000}],
    )

    # Create and return launch description
    return LaunchDescription([
        tsb_recruitment_sim_launch,
        sp01_state_publisher_launch,
        foxglove_bridge
    ])