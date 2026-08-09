from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("k9_system_pkg"),
        "config",
        "hotword.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="k9_system_pkg",
                executable="hotword",
                name="hotword",
                output="screen",
                parameters=[config],
                respawn=True,
                respawn_delay=2.0,
            )
        ]
    )
