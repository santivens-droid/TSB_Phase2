#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Setup function to build command with conditional loop argument."""
    
    bag_path = LaunchConfiguration('bag_path').perform(context)
    rate = LaunchConfiguration('rate').perform(context)
    loop = LaunchConfiguration('loop').perform(context)
    
    # Build the command
    cmd = ['ros2', 'bag', 'play', bag_path, '--rate', rate]
    
    # Add --loop if requested
    if loop.lower() == 'true':
        cmd.append('--loop')
    
    # ROS bag playback process
    bag_playback = ExecuteProcess(
        cmd=cmd,
        output='screen',
        name='bag_playback'
    )
    
    # Foxglove bridge node
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[
            {'send_buffer_limit': 100000000},
            {'port': 8765}
        ],
        output='screen'
    )
    
    return [bag_playback, foxglove_bridge]


def generate_launch_description():
    """Generate launch description for playing back a ROS bag with Foxglove bridge."""
    
    # Declare launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='/ws/bags/otter_simulator1',
        description='Path to the ROS bag directory to play'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier (e.g., 2.0 for 2x speed)'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Whether to loop the bag playback (true/false)'
    )

    # Create and return launch description
    return LaunchDescription([
        bag_path_arg,
        rate_arg,
        loop_arg,
        OpaqueFunction(function=launch_setup)
    ])
