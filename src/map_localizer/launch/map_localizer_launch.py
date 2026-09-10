import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launches only the map_localizer backend node (+ optional rviz).

    This does NOT launch any LIO/odometry front end - it expects
    FAST_LIO_Hesai_ROS2 (fast_lio_ros2) to already be running and publishing
    /Odometry and /cloud_registered_body, as configured in
    config/map_localizer.yaml. It relocalizes against a saved map (produced
    by map_hba) via the /map_localizer/relocalize service.
    """
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("map_localizer"), "rviz", "map_localizer.rviz"]
    )
    config_path = PathJoinSubstitution(
        [FindPackageShare("map_localizer"), "config", "map_localizer.yaml"]
    )
    use_rviz = LaunchConfiguration("use_rviz")

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz alongside map_localizer",
            ),
            launch_ros.actions.Node(
                package="map_localizer",
                namespace="map_localizer",
                executable="map_localizer_node",
                name="map_localizer_node",
                output="screen",
                parameters=[{"config_path": config_path.perform(launch.LaunchContext())}]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="map_localizer",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            )
        ]
    )
