import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Setup paths to required packages
    pkg_robot_description = get_package_share_directory('diy_robot_description')
    pkg_sim = get_package_share_directory('diy_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_challenge_bringup = get_package_share_directory('challenge_bringup')

    # 2. Declare configurations/arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Path to your custom world file inside diy_sim
    world_path = os.path.join(pkg_sim, 'worlds', 'diy_world.sdf')
    rviz_config = os.path.join(pkg_challenge_bringup, 'rviz', 'lidar_points.rviz')

    # 3. Process the Xacro file into a string for robot description parameters
    urdf_path = os.path.join(pkg_robot_description, 'urdf', 'robot.urdf.xacro')
    raw_robot_desc = Command(['xacro', ' ', urdf_path])
    robot_desc_param = ParameterValue(raw_robot_desc, value_type=str)
    
    bridge_config_file = os.path.join(pkg_sim, 'config', 'bridge_config.yaml')

    # 4. Configure the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc_param
        }]
    )

    # 5. Configure the joint_state_publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc_param
        }]
    )

    # 6. Include the Ignition Gazebo launch description (Server + Client GUI)
    # This automatically boots up ignition fortress with your designated world file
    ignition_spawn_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    # 7. Spawn the robot model inside the running Ignition simulation instance
    robot_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'juggernaut',
            '-topic', 'robot_description'
        ]
    )

    # 8. Bridge lidar point cloud topic between Gazebo and ROS 2
#    ros_gz_lidar_bridge_node = Node(
#        package='ros_gz_bridge',
#        executable='parameter_bridge',
#        name='ros_gz_lidar_bridge',
#        output='screen',
#        arguments=['/lidar/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'],
#        #remappings=[('/lidar', '/lidar_points')],
#        parameters=[{
#            'use_sim_time': use_sim_time
#        }]
#    )

    # 8. Bridge lidar point cloud topic between Gazebo and ROS 2
    ros_gz_lidar_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_lidar_bridge',
        output='screen',
        parameters=[
            {'config_file': bridge_config_file},
            {'use_sim_time': use_sim_time}
        ]
    )

    # 9. Launch RViz with lidar point cloud display
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        arguments=[
            '0', '0', '0', '0', '0', '0', 
            'lidar_link', 
            'juggernaut/base_footprint/gpu_lidar'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        ignition_spawn_sim,
        robot_spawn_node,
        ros_gz_lidar_bridge_node,
        rviz_node,
        static_transform_publisher_node
    ])
