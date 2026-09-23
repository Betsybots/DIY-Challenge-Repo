import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_cfg = PathJoinSubstitution([FindPackageShare("map_hba"), "rviz", "map_hba.rviz"])
    config_path = PathJoinSubstitution([FindPackageShare("map_hba"), "config", "map_hba.yaml"])
    use_rviz = LaunchConfiguration("use_rviz")

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz alongside map_hba",
            ),
            launch_ros.actions.Node(
                package="map_hba",
                namespace="map_hba",
                executable="map_hba_node",
                name="map_hba_node",
                output="screen",
                parameters=[
                    {"config_path": config_path.perform(launch.LaunchContext())}
                ],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="map_hba",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            ),
        ]
    )
