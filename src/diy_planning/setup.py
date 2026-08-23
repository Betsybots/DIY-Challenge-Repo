from setuptools import find_packages, setup

package_name = 'diy_planning'

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
    description='A* path-planning package for the DIY Challenge robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'a_star_planner_node = diy_planning.a_star_planner_node:main',
        ],
    },
)