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
    k9_system_share = get_package_share_directory('k9_system_pkg')
    k9_perception_share = get_package_share_directory('k9_perception_pkg')


    hotword_config = os.path.join(
        k9_system_share,
        'config',
        'hotword.yaml',
    )

    eye_camera_node = Node(
        package="k9_system_pkg",
        executable="eye_camera",
        name="eye_camera",
        output="screen",
        parameters=[
            os.path.join(
                k9_system_share,
                "config",
                "eye_camera.yaml",
            )
        ],
    )

    face_detector_node = Node(
        package="k9_perception_pkg",
        executable="face_detector",
        name="face_detector",
        output="screen",
        parameters=[
            os.path.join(
                k9_perception_share,
                "config",
                "face_detector.yaml",
            )
        ],
    )

    face_tracker_node = Node(
        package="k9_perception_pkg",
        executable="face_tracker",
        name="face_tracker",
        output="screen",
        parameters=[
            os.path.join(
                k9_perception_share,
                "config",
                "face_tracker.yaml",
            )
        ],
    )

    face_recogniser_node = Node(
        package="k9_perception_pkg",
        executable="face_recogniser",
        name="face_recogniser",
        output="screen",
        parameters=[
            os.path.join(
                k9_perception_share,
                "config",
                "face_recogniser.yaml",
            )
        ],
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


        OpaqueFunction(function=launch_nodes),
    ])
