from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare("k9_system_pkg"),
        "config",
        "voice_piper.yaml",
    ])

    return LaunchDescription([
        Node(
            package="k9_system_pkg",
            executable="voice_piper",
            name="voice_piper",
            output="screen",
            parameters=[params],
        ),
    ])
