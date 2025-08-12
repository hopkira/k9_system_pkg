import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'k9_system_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'assets'), glob('assets/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Richard Hopkins',
    maintainer_email='hopkira@googlemail.com',
    description='System nodes for K9',
    license='Apache License',
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
            'calendar = k9_system_pkg.calendar:main',
            'hotword = k9_system_pkg.hotword:main',
        ],
    },
)
