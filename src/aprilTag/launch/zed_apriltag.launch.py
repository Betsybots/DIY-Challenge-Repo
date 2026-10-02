import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_labels = os.path.join(
        get_package_share_directory("zed_apriltag"), "config", "tag_labels.yaml")

    tag_size = DeclareLaunchArgument("tag_size", default_value="0.16")
    tag_family = DeclareLaunchArgument("tag_family", default_value="tag36h11")
    image_topic = DeclareLaunchArgument(
        "image_topic", default_value="/zed/zed_node/rgb/color/rect/image")
    camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic", default_value="/zed/zed_node/rgb/color/rect/camera_info")
    tag_labels_file = DeclareLaunchArgument(
        "tag_labels_file", default_value=default_labels)

    node = Node(
        package="zed_apriltag",
        executable="zed_apriltag_node",
        name="zed_apriltag_node",
        output="screen",
        parameters=[{
            "tag_size": LaunchConfiguration("tag_size"),
            "tag_family": LaunchConfiguration("tag_family"),
            "image_topic": LaunchConfiguration("image_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "tag_labels_file": LaunchConfiguration("tag_labels_file"),
            "publish_debug_image": True,
        }],
    )

    return LaunchDescription([
        tag_size, tag_family, image_topic, camera_info_topic, tag_labels_file, node,
    ])
