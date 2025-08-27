import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'k9_system_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages= find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'assets'), glob('assets/*')),
    ],
    install_requires=['setuptools',
                      'faster-whisper',
                      'pvporcupine',
                      'pvrecorder',
                      'mediapipe',
                      ],
    zip_safe=True,
    maintainer='Richard Hopkins',
    maintainer_email='hopkira@googlemail.com',
    description='System nodes for K9',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'back_lights = k9_system_pkg.back_lights:main',
            'ears = k9_system_pkg.ears:main',
            'eyestail = k9_system_pkg.eyestail:main',
            'k9_client = k9_system_pkg.k9_client:main',
            'voice = k9_system_pkg.voice:main',
            'context = k9_system_pkg.context:main',
            'ollama = k9_system_pkg.ollama_wrap:main',
            'calendar = k9_system_pkg.k9_calendar:main',
            'hotword = k9_system_pkg.hotword:main',
            'weather = k9_system_pkg.weather:main',
            'garden = k9_system_pkg.gardentasks:main',
            'k9_stt = k9_system_pkg.k9_stt_ros2:main',
            'face_detect = k9_system_pkg.face_detection:main',
        ],
    },
)
