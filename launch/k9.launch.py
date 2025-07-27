from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime

# Usage:
# ros2 launch k9_system_pkg k9.launch.py log_level:=debug

def generate_launch_description():
    # Launch argument for log level
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    log_level = LaunchConfiguration('log_level')

    # List of all nodes in the package
    node_names = [
        'context',
        'ollama',
        'back_lights',
        'ears',
        'eyes',
        'tail',
        'voice',
    ]

    # Create all nodes with shared logging config
    nodes = [
        Node(
            package='k9_system_pkg',
            executable=name,
            name=name,
            output='both',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', log_level]
        )
        for name in node_names
    ]

    return LaunchDescription([log_level_arg] + nodes)