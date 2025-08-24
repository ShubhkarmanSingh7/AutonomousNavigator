import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'AutonomousNavigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Include all world files from the 'worlds' directory
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        # Include all config files from the 'config' directory
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shubh',
    maintainer_email='shubh@todo.todo',
    description='A service robot for the elderly.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add explorer node here
            'explorer = elderly_service_bot.explorer:main',
        ],
    },
)
