from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pixelbot_display',
            executable='pixelbot_display_node'
        ),
        Node(
            package='pixelbot_audio',
            executable='pixelbot_audio_node'
        ),
        Node(
            package='pixelbot_buttons',
            executable='pixelbot_buttons_node'
        ),
        Node(
            package='pixelbot_interaction',
            executable='pixelbot_interaction_node'
        )
    ])
