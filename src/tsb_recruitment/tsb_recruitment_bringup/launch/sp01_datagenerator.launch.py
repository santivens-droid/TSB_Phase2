from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for SP01 datagenerator with websocket bridge."""

    # Foxglove bridge
    foxglove_bridge = Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                parameters=[{'send_buffer_limit': 100000000}],
    )

    # SP01 DataGenerator Node
    sp01_datagenerator_node = Node(
        package='tsb_recruitment_datagenerator',
        executable='SP01DataGeneratorNode',
        name='sp01_datagenerator_node',
    )

    return LaunchDescription([
        foxglove_bridge,
        sp01_datagenerator_node
    ])