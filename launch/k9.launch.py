import os
from sys import platform

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

K9_VENV = os.path.expanduser(
    '~/k9_venv'
)

K9_VENV_SITE_PACKAGES = os.path.join(
    K9_VENV,
    'lib',
    'python3.12',
    'site-packages',
)

PHANTOM_CHESSBOARD_SRC = os.path.expanduser(
    '~/phantom_chessboard/src'
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
    'k9_rag',
]

def k9_venv_environment():
    existing_pythonpath = os.environ.get(
        'PYTHONPATH',
        ''
    )

    existing_path = os.environ.get(
        'PATH',
        ''
    )

    pythonpath = K9_VENV_SITE_PACKAGES

    if existing_pythonpath:
        pythonpath += (
            os.pathsep
            + existing_pythonpath
        )

    return {
        'VIRTUAL_ENV': K9_VENV,
        'PATH': (
            os.path.join(
                K9_VENV,
                'bin',
            )
            + os.pathsep
            + existing_path
        ),
        'PYTHONPATH': pythonpath,
    }

def launch_nodes(context):
    """Create the set of K9 nodes appropriate for the selected computer."""

    platform = LaunchConfiguration('platform').perform(context)
    log_level = (
        LaunchConfiguration('log_level')
        .perform(context)
        .upper()
    )

    enable_chess = LaunchConfiguration('enable_chess').perform(context).lower() in ('1', 'true', 'yes', 'on')
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

    hotword_config = os.path.join(
        k9_system_share,
        'config',
        'hotword.yaml',
    )

    nodes = []

    if platform in ('pi', 'all'):
        drive_launch = os.path.join(
            get_package_share_directory('k9_drive_pkg'),
            'launch',
            'drive.launch.py',
        )

        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    drive_launch
                ),
                launch_arguments={
                    'dry_run': 'false',
                    'start_inhibited': 'true',
                    'device': '/dev/roboclaw',
                    'publish_robot_description': 'false',
                }.items(),
            )
        )

    if platform in ('jetson', 'all'):
        imu_launch = os.path.join(
            get_package_share_directory('k9_imu_pkg'),
            'launch',
            'imu.launch.py',
        )

        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    imu_launch
                )
            )
        )

    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                drive_launch
            ),
            launch_arguments={
                'dry_run': 'false',
                'start_inhibited': 'true',
                'device': '/dev/roboclaw',
                'publish_robot_description': 'false',
            }.items(),
        )
    )

    # Navigation owns the sensor configuration; system bring-up owns startup.
    if platform in ('pi', 'all') and LaunchConfiguration(
            'enable_ld06').perform(context).lower() in ('true', '1', 'yes', 'on'):
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('k9_ros2_nav'),
                         'launch', 'ld06.launch.py'))))

    # OAK hardware and processing stay on the Pi; isolate driver launch arguments.
    if platform in ('pi', 'all') and LaunchConfiguration(
            'enable_oak').perform(context).lower() in ('true', '1', 'yes', 'on'):
        oak_share = os.path.join(get_package_share_directory('k9_ros2_nav'), 'oak')
        oak_actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(oak_share, 'oak.launch.py')))]
        if LaunchConfiguration('enable_oak_floor_filter').perform(context).lower() in (
                'true', '1', 'yes', 'on'):
            oak_actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(oak_share, 'oak_floor_filter.launch.py'))))
        nodes.append(GroupAction(actions=oak_actions, scoped=True))

    # The Pi owns shared robot geometry; Jetson consumes TF over ROS.
    if platform in ('pi', 'all') and LaunchConfiguration(
            'enable_robot_description').perform(context).lower() in ('true', '1', 'yes', 'on'):
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(k9_system_share, 'launch', 'robot_description.launch.py'))))

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
                    ),
                    launch_arguments={
                        'log_level': log_level,
                    }.items(),
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

            node_args['additional_env'] = (
                k9_venv_environment()
            )

            node_args['respawn'] = True
            node_args['respawn_delay'] = 2.0

        elif name == 'k9_rag':

            node_args['parameters'] = [
                os.path.join(
                    k9_system_share,
                    'config',
                    'rag.yaml',
                )
            ]

            rag_env = k9_venv_environment()

            rag_env['PYTHONWARNINGS'] = (
                'ignore::UserWarning:torch.cuda'
            )

            node_args['additional_env'] = rag_env

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
                PathJoinSubstitution(
                    [
                        FindPackageShare('k9_perception_pkg'),
                        'config',
                        f'{name}.yaml',
                    ]
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

# ------------------------------------------------------------------
# Chess subsystem.
#
# Keep all K9 runtime nodes under this single top-level launch file.
# The Phantom adapter needs both the K9 virtual environment and the
# standalone phantom_chessboard source tree on PYTHONPATH.
# ------------------------------------------------------------------

    if run_bt and enable_chess:

        chess_share = get_package_share_directory(
            'k9_chess_pkg'
        )

        chess_config = os.path.join(
            chess_share,
            'config',
            'chess.yaml',
        )

        phantom_config = os.path.join(
            chess_share,
            'config',
            'phantom_board.yaml',
        )

        # --------------------------------------------------------------
        # Chess engine.
        # --------------------------------------------------------------

        nodes.append(
            Node(
                package='k9_chess_pkg',
                executable='chess_engine',
                name='chess_engine',
                output='both',
                emulate_tty=True,
                parameters=[
                    chess_config,
                ],
                arguments=[
                    '--ros-args',   
                    '--log-level',
                    log_level,
                ],
                additional_env=(
                    k9_venv_environment()
                ),
            )
        )

        # --------------------------------------------------------------
        # Chess manager.
        # --------------------------------------------------------------

        nodes.append(
            Node(
                package='k9_chess_pkg',
                executable='chess_manager',
                name='chess_manager',
                output='both',
                emulate_tty=True,
                parameters=[
                    chess_config,
                ],
                arguments=[
                    '--ros-args',
                    '--log-level',
                    log_level,
                ],
                additional_env=(
                    k9_venv_environment()
                ),
            )
        )

        # --------------------------------------------------------------
        # Phantom Chessboard adapter.
        #
        # This node uses the standalone phantom_chessboard Python package
        # as well as dependencies from the K9 venv.
        # --------------------------------------------------------------

        phantom_env = (
            k9_venv_environment()
        )

        phantom_env['PYTHONPATH'] = (
            PHANTOM_CHESSBOARD_SRC
            + os.pathsep
            + phantom_env['PYTHONPATH']
        )

        nodes.append(
            Node(
                package='k9_chess_pkg',
                executable='phantom_board',
                name='phantom_board',
                output='both',
                emulate_tty=True,
                parameters=[
                    phantom_config,
                ],
                arguments=[
                    '--ros-args',   
                    '--log-level',
                    log_level,
                ],
                additional_env=phantom_env,
            )
        )

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
            'enable_oak', default_value='true',
            description='Launch OAK camera and processing on Pi/all; false skips both'),
        DeclareLaunchArgument(
            'enable_oak_floor_filter', default_value='true',
            description='Run floor filter with OAK on Pi/all; false keeps only raw cloud'),
        DeclareLaunchArgument(
            'enable_ld06', default_value='true',
            description='Launch LD06 and 20 cm filter on Pi/all; disable if already running'),
        DeclareLaunchArgument(
            'enable_robot_description', default_value='true',
            description='Publish shared hardware TF on Pi/all; disable if already running'),
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

        DeclareLaunchArgument(

            'enable_chess',

            default_value='true',

            description=(

                'Launch k9_chess_pkg on Jetson/all platforms '

                '(true/false)'

            ),

        ),

        OpaqueFunction(
            function=launch_nodes
        ),
    ])
