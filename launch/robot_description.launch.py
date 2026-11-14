"""Publish the single authoritative full K9 robot description for physical hardware."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def description_nodes(context, *args, **kwargs):
    share = Path(get_package_share_directory('k9_description'))
    dry_run = LaunchConfiguration('drive_dry_run').perform(context)
    inhibited = LaunchConfiguration('drive_start_inhibited').perform(context)
    device = LaunchConfiguration('drive_device').perform(context)
    model = share / 'model' / 'k9.urdf.xacro'
    if not model.is_file():
        raise RuntimeError(f'K9 description Xacro not installed: {model}')
    document = xacro.process_file(str(model), mappings={
        'sim': 'false',
        'description_share': str(share),
        'controllers_file': str(share / 'config' / 'controllers.yaml'),
        'drive_dry_run': dry_run,
        'drive_start_inhibited': inhibited,
        'drive_device': device,
    })
    xml = document.toxml()
    if '<ros2_control' not in xml or 'K9RoboClawSystem' not in xml:
        raise RuntimeError('Full K9 URDF lacks K9RoboClawSystem; refusing to start')
    return [Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': xml, 'use_sim_time': False}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drive_dry_run', default_value='false'),
        DeclareLaunchArgument('drive_start_inhibited', default_value='true'),
        DeclareLaunchArgument('drive_device', default_value='/dev/roboclaw'),
        OpaqueFunction(function=description_nodes),
    ])
