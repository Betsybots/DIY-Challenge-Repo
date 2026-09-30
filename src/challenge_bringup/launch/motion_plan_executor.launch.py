import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('challenge_bringup')
    default_plan = os.path.join(pkg_share, 'config', 'motion_plan.yaml')

    plan_file = LaunchConfiguration('plan_file')
    pose_topic = LaunchConfiguration('pose_topic')
    cmd_topic = LaunchConfiguration('cmd_topic')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'plan_file',
            default_value=default_plan,
            description='Absolute path to YAML motion plan file',
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/wheel_odom',
            description='Pose input topic (geometry_msgs/msg/PoseStamped)',
        ),
        DeclareLaunchArgument(
            'cmd_topic',
            default_value='/cmd_vel_smoothed',
            description='Twist command output topic',
        ),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/zed/zed_node/point_cloud/cloud_registered',
            description='ZED PointCloud2 topic used by WALL_FOLLOW and obstacle stops',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_footprint',
            description='Robot frame for transforming and interpreting the point cloud',
        ),
        Node(
            package='plan_b',
            executable='motion_plan_executor_node',
            name='motion_plan_executor',
            output='screen',
            parameters=[{
                'plan_file': plan_file,
                'pose_topic': pose_topic,
                'cmd_topic': cmd_topic,
                'pointcloud_topic': pointcloud_topic,
                'base_frame': base_frame,
            }],
        ),
    ])
