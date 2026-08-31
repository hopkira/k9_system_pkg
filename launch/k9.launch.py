import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


K9_VENV_SITE_PACKAGES = os.path.expanduser(
    '~/k9_venv/lib/python3.12/site-packages'
)


PI_NODES = [
    'back_lights',
    'ears',
    'eyes_tail',
    'eye_camera',
]


JETSON_NODES = [
    'hotword',
    'k9_stt',
    'voice_piper',
    'intent',
    'conversation',
    'face_detector',
    'face_tracker',
    'face_recogniser',
]


def launch_nodes(context):
    """Create the set of K9 nodes appropriate for the selected computer."""

    platform = LaunchConfiguration('platform').perform(context)
    log_level = LaunchConfiguration('log_level')

    # ------------------------------------------------------------------
    # Select the nodes belonging to this K9 computer.
    # ------------------------------------------------------------------

    if platform == 'pi':
        node_names = PI_NODES
        run_bt = False

    elif platform == 'jetson':
        node_names = JETSON_NODES
        run_bt = True

    elif platform == 'all':
        node_names = JETSON_NODES + PI_NODES
        run_bt = True

    else:
        raise RuntimeError(
            f"Unknown platform '{platform}'. "
            "Expected: pi, jetson or all."
        )

    # ------------------------------------------------------------------
    # Locate installed package resources.
    # ------------------------------------------------------------------

    k9_system_share = get_package_share_directory(
        'k9_system_pkg'
    )

    k9_perception_share = get_package_share_directory(
        'k9_perception_pkg'
    )

    hotword_config = os.path.join(
        k9_system_share,
        'config',
        'hotword.yaml',
    )

    nodes = []

    # ------------------------------------------------------------------
    # Construct each requested node.
    # ------------------------------------------------------------------

    for name in node_names:

        # --------------------------------------------------------------
        # STT is a special case.
        #
        # k9_stt_pkg owns its own launch file and therefore its own
        # Whisper/VAD configuration. Including that launch file here
        # prevents the STT parameters being duplicated in k9.launch.py.
        # --------------------------------------------------------------

        if name == 'k9_stt':

            stt_launch = os.path.join(
                get_package_share_directory('k9_stt_pkg'),
                'launch',
                'stt.launch.py',
            )

            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        stt_launch
                    )
                )
            )

            # Do not also launch k9_stt through the generic Node()
            # construction below.
            continue

        # --------------------------------------------------------------
        # Select the ROS package containing the executable.
        # --------------------------------------------------------------

        if name in (
            'face_detector',
            'face_tracker',
            'face_recogniser',
        ):
            package = 'k9_perception_pkg'

        else:
            package = 'k9_system_pkg'

        # --------------------------------------------------------------
        # Common configuration for normally launched K9 nodes.
        # --------------------------------------------------------------

        node_args = {
            'package': package,
            'executable': name,
            'name': name,
            'output': 'both',
            'emulate_tty': True,
            'arguments': [
                '--ros-args',
                '--log-level',
                log_level,
            ],
        }

        # --------------------------------------------------------------
        # Node-specific configuration.
        # --------------------------------------------------------------

        if name == 'hotword':

            node_args['parameters'] = [
                hotword_config
            ]

            existing_pythonpath = os.environ.get(
                'PYTHONPATH',
                ''
            )

            node_args['additional_env'] = {
                'PYTHONPATH': (
                    K9_VENV_SITE_PACKAGES
                    + os.pathsep
                    + existing_pythonpath
                )
            }

            # Hotword is a fundamental sensor node. Restart it
            # automatically if ALSA or Sherpa fails.
            node_args['respawn'] = True
            node_args['respawn_delay'] = 2.0

        elif name == 'eye_camera':

            node_args['parameters'] = [
                os.path.join(
                    k9_system_share,
                    'config',
                    'eye_camera.yaml',
                )
            ]

        elif name in (
            'face_detector',
            'face_tracker',
            'face_recogniser',
        ):

            node_args['parameters'] = [
                os.path.join(
                    k9_perception_share,
                    'config',
                    f'{name}.yaml',
                )
            ]

        # --------------------------------------------------------------
        # Add the configured node to the launch description.
        # --------------------------------------------------------------

        nodes.append(
            Node(**node_args)
        )

    # ------------------------------------------------------------------
    # Behaviour tree runs on Jetson, or when explicitly launching all
    # K9 nodes together.
    # ------------------------------------------------------------------

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
                    log_level,
                ],
            )
        )

    return nodes


def generate_launch_description():
    """Build the top-level K9 launch description."""

    return LaunchDescription([
        DeclareLaunchArgument(
            'platform',
            default_value='pi',
            description='K9 computer role: pi, jetson or all',
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description=(
                'Logging level: debug, info, warn, error, fatal'
            ),
        ),

        OpaqueFunction(
            function=launch_nodes
        ),
    ])
