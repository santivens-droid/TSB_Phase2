#!/usr/bin/env python3
"""
Launch file for TSB Recruitment Simulation Node

This launch file starts the simplified HoloOcean interface node.
Parameters can be overridden via command line or by modifying the sim_params.yaml file.

The node supports 4 core parameters:
- frames_per_sec (overrides scenario JSON)
- ticks_per_sec (overrides scenario JSON)  
- show_viewport (standalone)
- render_quality (standalone)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the launch description for the TSB recruitment simulation."""
    
    # Declare launch argument for config file
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='otter_simple.json',
        description='Configuration file for the simulation scenario'
    )
    
    # Declare launch argument for show_viewport
    show_viewport_arg = DeclareLaunchArgument(
        'show_viewport',
        default_value='true',
        description='Whether to show HoloOcean viewport window'
    )
    
    # Get the package directory
    package_dir = get_package_share_directory('tsb_recruitment_sim')
    
    # Path to the parameters file
    params_file = os.path.join(package_dir, 'config', 'sim_params.yaml')
    
    # Define the node
    sim_node = Node(
        package='tsb_recruitment_sim',
        executable='tsb_recruitment_sim_node',
        name='tsb_recruitment_sim_node',
        output='screen',
        parameters=[
            params_file,  # Load parameters from file
            {'config_file': LaunchConfiguration('config_file')},  # Override config_file parameter
            {'show_viewport': LaunchConfiguration('show_viewport')}  # Override show_viewport parameter
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        show_viewport_arg,
        sim_node
    ])