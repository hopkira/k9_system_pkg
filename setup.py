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
            'eyes = k9_system_pkg.eyes:main',
            'k9_client = k9_system_pkg.k9_client:main',
            'tail = k9_system_pkg.tail:main',
            'voice = k9_system_pkg.voice:main',
            'context = k9_system_pkg.context:main',
            'ollama = k9_system_pkg.ollama_wrap:main',            
        ],
    },
)
