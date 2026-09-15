import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. Setup paths to required packages
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_sim = get_package_share_directory('sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_challenge_bringup = get_package_share_directory('challenge_bringup')
    #fast_lio_dir = get_package_share_directory('fast_lio')
    
    # 2. Combine the path with the actual launch file name
    #fast_lio_launch_file = os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')

    # 2. Declare configurations/arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Path to your custom world file inside diy_sim
    world_path = os.path.join(pkg_sim, 'worlds', 'diy_world.sdf')
    rviz_config = os.path.join(pkg_challenge_bringup, 'rviz', 'lidar_points.rviz')

    # 3. Process the Xacro file into a string for robot description parameters
    urdf_path = os.path.join(pkg_robot_description, 'urdf', 'robot.urdf.xacro')
    raw_robot_desc = Command(['xacro', ' ', urdf_path, ' sim:=true'])
    robot_desc_param = ParameterValue(raw_robot_desc, value_type=str)
    
    bridge_config_file = os.path.join(pkg_sim, 'config', 'bridge_config.yaml')
    workspace_root = os.path.abspath(
        os.path.join(pkg_challenge_bringup, '..', '..', '..', '..')
    )

    # Ignition/Gazebo needs the model/resource path and loopback networking so it
    # does not hang on VPN interfaces or missing model paths when starting the sim.
    sim_models_dir = os.path.join(pkg_sim, 'models')
    robot_share_dir = os.path.dirname(pkg_robot_description)
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''))
    new_gz_path = ':'.join(filter(None, [sim_models_dir, robot_share_dir, existing_gz_path]))
    os.environ['GZ_SIM_RESOURCE_PATH'] = new_gz_path
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = new_gz_path
    os.environ.setdefault('IGN_IP', '127.0.0.1')

    script_path = os.path.join(workspace_root, 'src', 'lidar_patch_script.py')
    
    # Define the process execution
    run_python_script = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen'
    )
    
    # 4. Configure the robot_state_publisher node (Needed for both SIM and live hardware)
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

    # 5. Configure the joint_state_publisher node (Needed for SIM)
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
    # This automatically boots up ignition fortress with your designated world file (Needed for SIM)
    ignition_spawn_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [world_path, TextSubstitution(text=' -r')],
            'on_exit_shutdown': 'true',
        }.items()
    )

    # 7. Spawn the robot model inside the running Ignition simulation instance (Needed for both SIM and live hardware)
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

    # 8. Bridge lidar point cloud topic between Gazebo and ROS 2 (Needed for SIM)
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

    # 9. Launch RViz with lidar point cloud display (Needed for both SIM and live hardware)
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

    #10. Static transform node to change the frame name for lidar and imu topic
    #TODO: Still need to check if the frame name has been changed (Needed for SIM)
    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_bridge',
        arguments=[
            '0', '0', '0', '0', '0', '0', 
            'lidar_link', 
            'juggernaut/body/gpu_lidar'
        ]
    )
    
    # 11. Run FAST_LIO algorithm (Needed for both SIM and live hardware)
    #fast_lio_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(fast_lio_launch_file),
    #    launch_arguments={'use_sim_time': use_sim_time}.items() # Optional arguments
    #)
    
    body_to_base_link_bridge = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link_bridge',
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'imu_link'],
            parameters=[{'use_sim_time': True}]
        )

    # 12. Fuse wheel odometry + IMU (angular rate) via EKF -> /FinalOdometry (Needed for SIM)
    ekf_config = os.path.join(pkg_challenge_bringup, 'config', 'ekf_sim.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_sim',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/FinalOdometry')]
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_gz_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', new_gz_path),
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node,
        ignition_spawn_sim,
        TimerAction(
            period=8.0,
            actions=[
                robot_spawn_node,
                ros_gz_lidar_bridge_node,
                rviz_node,
                run_python_script,
                ekf_node,
            ],
        ),
    ])
