import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'diy_state_estimate'

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
        'Single-EKF odom-frame fusion of wheel odometry, IMU yaw rate and '
        'FAST-LIO2 lidar odometry for Nav2.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensor_covariance_relay = diy_state_estimate.sensor_covariance_relay:main',
            'localization_watchdog = diy_state_estimate.localization_watchdog:main',
        ],
    },
)
