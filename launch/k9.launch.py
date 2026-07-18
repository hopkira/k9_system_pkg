from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PI_NODES = [
    'back_lights',
    'ears',
    'eyestail',
    'voice',
    'hotword',
]

ORIN_NODES = [
    'internet_monitor',
    'context',
    'ollama',
    'calendar',
    'weather',
    'garden',
    'k9_stt',
    'face_detect',
]


def launch_nodes(context):
    platform = LaunchConfiguration('platform').perform(context)
    log_level = LaunchConfiguration('log_level')

    if platform == 'pi':
        node_names = PI_NODES
        run_bt = False

    elif platform == 'orin':
        node_names = ORIN_NODES
        run_bt = True

    elif platform == 'all':
        node_names = PI_NODES + ORIN_NODES
        run_bt = True

    else:
        raise RuntimeError(
            f"Unknown platform '{platform}'. "
            "Expected: pi, orin or all."
        )

    nodes = [
        Node(
            package='k9_system_pkg',
            executable=name,
            name=name,
            output='both',
            emulate_tty=True,
            arguments=[
                '--ros-args',
                '--log-level',
                log_level
            ],
        )
        for name in node_names
    ]

    if run_bt:
        nodes.append(
            Node(
                package='k9_bt_pkg',
                executable='k9_bt',
                name='k9_bt',
                output='both',
                emulate_tty=True,
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level
                ],
            )
        )

    return nodes


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'platform',
            default_value='pi',
            description='K9 computer role: pi, orin or all',
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level: debug, info, warn, error, fatal',
        ),

        OpaqueFunction(function=launch_nodes),
    ])

