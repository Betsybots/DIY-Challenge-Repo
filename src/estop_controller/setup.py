from setuptools import find_packages, setup

package_name = 'diy_estop_controller'

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
    description='E-stop controller bridge for DIY Challenge 2026.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'estop_controller = diy_estop_controller.estop_controller_node:main',
        ],
    },
)
