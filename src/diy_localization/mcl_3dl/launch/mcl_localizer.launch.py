from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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
        DeclareLaunchArgument("cloud_topic", default_value="/cloud_registered_body"),
        # Default is the real IMU (Aceinna, on the Raspberry Pi) - frame_id "imu_link",
        # which has a static TF from base_link/base_footprint. The ZED's own IMU
        # (/zed/zed_node/imu/data, frame_id "zed_imu_link") has NO such static TF
        # (zed_wrapper's publish_imu_tf defaults false), so pointing mcl_3dl at it
        # makes every cbImu() TF lookup fail silently -> no IMU correction ever
        # applied, plus a spurious periodic "Detected time jump in imu" warning
        # every ~5s (imu_last_ never advances while the lookup keeps failing).
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("odom_topic", default_value="/zed/zed_node/odom"),
        Node( 
            package="pcl_ros",
            executable="pcd_to_pointcloud",
            name="pcd_to_pointcloud",
            output="screen",
            parameters=[config_file],
            remappings=[
                ("/cloud_pcd", "/mapcloud")
            ]
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="mcl_3dl",
                    executable="mcl_3dl",
                    name="mcl_3dl",
                    output="screen",
                    parameters=[config_file],
                    remappings=[
                        # ("/cloud", "/cloud_registered_body")
                        ("/cloud", LaunchConfiguration("cloud_topic")),
                        ("/imu/data", LaunchConfiguration("imu_topic")),
                        ("/odom", LaunchConfiguration("odom_topic"))
                    ],
                ),
            ],
        ),
            #ros2 run pcl_ros pcd_to_pointcloud   
            # --ros-args   -p file_name:=/home/juggernauts/DIY-Challenge-Repo/src/diy_localization/map/refined_map.pcd  
            # -p publish_rate:=1.0 
            # -r cloud_pcd:=/mapcloud 
            # -p tf_frame:=map
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
