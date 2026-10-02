from glob import glob

from setuptools import setup

package_name = 'course_supervisor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juggernauts',
    maintainer_email='juggernauts@example.com',
    description='Section-by-section controller selection for the obstacle course.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'course_supervisor = course_supervisor.supervisor_node:main',
            'make_course_file = course_supervisor.make_course_file:main',
            'plan_tags = course_supervisor.plan_tags:main',
        ],
    },
)
