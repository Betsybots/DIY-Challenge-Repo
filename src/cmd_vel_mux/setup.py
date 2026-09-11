from setuptools import find_packages, setup

package_name = 'diy_cmd_vel_mux'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DIY Challenge Team',
    maintainer_email='team@example.com',
    description='Command-velocity multiplexer for DIY Challenge 2026.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_mux_node = diy_cmd_vel_mux.cmd_vel_mux_node:main',
        ],
    },
)
