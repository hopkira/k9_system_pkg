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
        'eyestail',
        'voice',
        'calendar',
        'weather',
        'garden',
        'k9_stt',
        'face_detect',
        'hotword',
        'oak_points_frame_rewrite'
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

    # Add the behavior tree node from k9_bt_pkg
    k9_bt_node = Node(
        package='k9_bt_pkg',
        executable='k9_bt',        # <--- assumes you installed entry_point "k9_bt = k9_bt_pkg.k9_bt:main"
        name='k9_bt',
        output='both',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([log_level_arg] + nodes + [k9_bt_node])