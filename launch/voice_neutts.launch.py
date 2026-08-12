#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("k9_system_pkg")
    params_file = os.path.join(
        package_share,
        "config",
        "voice_neutts.yaml",
    )

    voice_id = LaunchConfiguration("voice_id")
    neutts_venv = LaunchConfiguration("neutts_venv")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "voice_id",
                default_value="0",
                description="NeuTTS voice reference ID (0..999)",
            ),
            DeclareLaunchArgument(
                "neutts_venv",
                default_value=os.path.expanduser(
                    "~/tts_env/neutts/.venv"
                ),
                description="Python virtual environment containing NeuTTS",
            ),
            SetEnvironmentVariable(
                "K9_NEUTTS_VENV",
                neutts_venv,
            ),
            Node(
                package="k9_system_pkg",
                executable="voice_neutts",
                name="k9_tts_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "voice_id": ParameterValue(
                            voice_id,
                            value_type=int,
                        )
                    },
                ],
            ),
        ]
    )
