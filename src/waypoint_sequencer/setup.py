import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'diy_waypoint_sequencer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DIY Challenge Team',
    maintainer_email='team@example.com',
    description=(
        'Standalone, optional automated goal-pose sequencer for the custom '
        'A* + PD/pure-pursuit navigation stack.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_sequencer_node = diy_waypoint_sequencer.waypoint_sequencer_node:main',
        ],
    },
)
