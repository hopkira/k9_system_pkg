from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

K9_VENV_SITE_PACKAGES = os.path.expanduser(
    '~/k9_venv/lib/python3.12/site-packages'
)

PI_NODES = [
    'back_lights',
    'ears',
    'eyestail',
]

JETSON_NODES = [
    'hotword',
    'k9_stt',
    'voice_neutts',
]


def launch_nodes(context):
    platform = LaunchConfiguration('platform').perform(context)
    log_level = LaunchConfiguration('log_level')

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

    # Installed location of k9_system_pkg
    system_pkg_share = get_package_share_directory('k9_system_pkg')

    voice_neutts_config = os.path.join(
        system_pkg_share,
        'config',
        'voice_neutts.yaml',
        )
    voice_id = LaunchConfiguration('voice_id')

    neutts_venv = LaunchConfiguration(
        'neutts_venv'
    ).perform(context)

    hotword_config = os.path.join(
        system_pkg_share,
        'config',
        'hotword.yaml',
    )

    nodes = []

    for name in node_names:

        package = (
            'k9_stt_pkg'
            if name == 'k9_stt'
            else 'k9_system_pkg')

        # Common settings for every K9 system node
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

        # Node-specific configuration
        if name == 'hotword':
            node_args['parameters'] = [hotword_config]
            existing_pythonpath = os.environ.get('PYTHONPATH', '')

            node_args['additional_env'] = {
                'PYTHONPATH': (
                    K9_VENV_SITE_PACKAGES
                    + os.pathsep
                    + existing_pythonpath
                )
            }
            # Hotword is a fundamental sensor node.
            # Restart it automatically if ALSA/Sherpa fails.
            node_args['respawn'] = True
            node_args['respawn_delay'] = 2.0

        if name == 'voice_neutts':

            node_args['name'] = 'k9_tts_node'

            node_args['parameters'] = [
                voice_neutts_config,
                {
                    'voice_id': ParameterValue(
                        voice_id,
                        value_type=int,
                    )
                },
            ]

            node_args['additional_env'] = {
                'K9_NEUTTS_VENV': neutts_venv,
            }
            
        nodes.append(Node(**node_args))

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

    return LaunchDescription([
        DeclareLaunchArgument(
            'platform',
            default_value='pi',
            description='K9 computer role: pi, jetson or all',
        ),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level: debug, info, warn, error, fatal',
        ),

        DeclareLaunchArgument(
            'voice_id',
            default_value='43',
            description='NeuTTS voice reference ID (0..999)',
        ),

        DeclareLaunchArgument(
            'neutts_venv',
            default_value=os.path.expanduser(
                '~/tts_env/neutts/.venv'
            ),
            description='Python virtual environment containing NeuTTS',
        ),

        OpaqueFunction(function=launch_nodes),
    ])
