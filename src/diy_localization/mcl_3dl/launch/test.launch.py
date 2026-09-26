from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("mcl_3dl"), "config", "test_localization.yaml"
            ]),
        ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
        Node(
            package="mcl_3dl",
            executable="mcl_3dl",
            name="mcl_3dl",
            output="screen",
            parameters=[config_file],
            remappings=[
                ("/cloud", "/cloud_registered_body")
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", PathJoinSubstitution([
                FindPackageShare("mcl_3dl"), "config", "mcl_3dl_demo.rviz"
            ])],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])