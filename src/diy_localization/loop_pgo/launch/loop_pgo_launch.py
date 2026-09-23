import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launches only the loop_pgo backend node (+ optional rviz).

    This does NOT launch any LIO/odometry front end - it expects
    FAST_LIO_Hesai_ROS2 (fast_lio_ros2) to already be running and publishing
    /Odometry and /cloud_registered_body, as configured in
    config/loop_pgo.yaml.
    """
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("loop_pgo"), "rviz", "loop_pgo.rviz"]
    )
    pgo_config_path = PathJoinSubstitution(
        [FindPackageShare("loop_pgo"), "config", "loop_pgo.yaml"]
    )
    use_rviz = LaunchConfiguration("use_rviz")

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Launch RViz alongside loop_pgo",
            ),
            launch_ros.actions.Node(
                package="loop_pgo",
                namespace="loop_pgo",
                executable="loop_pgo_node",
                name="loop_pgo_node",
                output="screen",
                parameters=[{"config_path": pgo_config_path.perform(launch.LaunchContext())}]
            ),
            launch_ros.actions.Node(
                package="rviz2",
                namespace="loop_pgo",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_cfg.perform(launch.LaunchContext())],
            )
        ]
    )
