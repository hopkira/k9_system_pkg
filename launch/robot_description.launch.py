"""Publish the shared K9 hardware geometry without starting controllers."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import xacro


def description_nodes(context):
    share = Path(get_package_share_directory('k9_description'))
    document = xacro.process_file(
        str(share / 'model/k9.urdf.xacro'),
        mappings={'sim': 'false', 'description_share': str(share),
                  'controllers_file': str(share / 'config/controllers.yaml')})
    return [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': document.toxml(), 'use_sim_time': False}])]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=description_nodes)])
