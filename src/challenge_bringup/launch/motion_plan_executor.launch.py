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

    return LaunchDescription([
        DeclareLaunchArgument(
            'plan_file',
            default_value=default_plan,
            description='Absolute path to YAML motion plan file',
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/ground_truth_pose',
            description='Pose input topic (geometry_msgs/msg/PoseStamped)',
        ),
        DeclareLaunchArgument(
            'cmd_topic',
            default_value='/cmd_vel',
            description='Twist command output topic',
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
            }],
        ),
    ])
