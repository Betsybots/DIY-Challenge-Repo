#!/usr/bin/env python3
"""
challenge_master.launch.py — Top-level competition bringup
══════════════════════════════════════════════════════════
Single file that starts the entire robot stack for competition or debug runs.
Every subsystem is gated by a boolean argument so the same file works whether
you are running the full competition stack on the Jetson, a lightweight relay
setup on the Raspberry Pi, or a laptop-only replay / debug session.

ARGUMENT → DEVICE PROFILE MAPPING
──────────────────────────────────
Each argument mirrors a  DIY_*  environment variable set by a device profile:
  profiles/jetson.env   — full hardware stack (all subsystems enabled)
  profiles/raspi.env    — lightweight relay (no localization, no Nav2)
  profiles/laptop.env   — debug/replay (no hardware drivers)

Source a profile, then launch:
  source scripts/env.sh jetson
  ros2 launch challenge_bringup challenge_master.launch.py

CLI overrides are also supported without editing this file:
  ros2 launch challenge_bringup challenge_master.launch.py use_nav2:=true mux_mode:=JOYSTICK

STARTUP ORDER
─────────────
   1. robot_description  — publishes URDF / TF tree  (MUST be first)
   2. micro_ros_agent        — STM32 serial link (disabled by default — see BLOCK 4)
   3. estop_controller_node  — reads STM32 state, publishes /estop_active
   4. cmd_vel_mux_node       — velocity arbitration (single-owner device — see BLOCK 5)
   5. hesai_ros_driver       — lidar driver feeding FAST-LIO2
   6. zed_wrapper (zed2i)    — RGB-D + stereo + IMU
   7. localization stack     — FAST-LIO2 + single EKF (ekf_odom.yaml) + NDT-OMP
   8. joystick_drive         — conditionally enabled
   9. differential_drive     — motor driver subscribing /cmd_vel_safe (mux output)
  10. nav2 bringup           — conditionally enabled
  11. zone_nav               — zone-aware state machine; requires use_nav2=true
  12. rviz2                  — conditionally enabled (off by default to save resources)

NOTE ON THE ACEINNA IMU:
────────────────────────
The IMU driver (imu_can_interface) is launched INDEPENDENTLY on the RPi,
outside this repo entirely — it is not vendored here and not part of this
launch file. FAST-LIO2 and the localization EKF (both on the Jetson) still
consume /imu/data as an external input over the Zenoh bridge; this file has
no responsibility for starting that driver.

Launch arguments (all correspond to DIY_* profile variables):
  use_joystick       bool  Enable joystick teleop              (default false)
  use_nav2           bool  Enable Nav2 autonomous stack         (default false)
  use_zone_nav       bool  Enable zone-aware nav state machine  (default false)
  use_zed            bool  Enable ZED2i camera driver           (default true)
  use_motor_driver   bool  Enable differential-drive node       (default true)
  use_hesai          bool  Enable Hesai QT64 lidar driver       (default true)
  use_micro_ros      bool  Enable micro-ROS agent + STM32 estop (default false)
                           mirror (diy_estop_controller). This robot has no
                           STM32 — the real hardware e-stop is an RJ45
                           break-loop wired directly into the motor power
                           path (fail-safe, satisfies competition rule 1.2.7
                           independent of any software). Only set true if an
                           STM32 + micro-ROS bridge is actually present —
                           otherwise estop_controller_node's fail-safe design
                           (heartbeat never received -> permanent E-stop)
                           locks cmd_vel_mux into ESTOP_LOCK from tick one.
  use_localization   bool  Enable FAST-LIO2 + EKF stack         (default true)
  use_cmd_vel_mux    bool  Enable cmd_vel_mux node              (default true)
                           Single-owner across a multi-machine robot — see
                           BLOCK 5. Set true on exactly one device (this
                           robot: the RPi, co-located with the motor driver
                           and e-stop wiring); false on every other device.
  use_rviz           bool  Launch RViz2                         (default false)
  mux_mode           str   cmd_vel_mux startup mode             (default AUTONOMOUS)
  fastlio_config     str   FAST-LIO2 config filename            (see localization/config/)
  waypoints_file     str   Override zone_waypoints.yaml path    (default: package default)
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir     = get_package_share_directory('challenge_bringup')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')  # MPPI + behaviour config
    # map_yaml removed — navigation_launch.py does not use AMCL/map_server.
    # NDT-OMP provides the map→odom TF directly on the real robot.

    # ── LaunchConfiguration handles (lazy substitutions) ──────────────────────
    # These are NOT yet resolved strings — they are substitution objects that
    # ROS 2 launch evaluates at start-up.  Pass them directly to IfCondition()
    # or as node parameters.  Do NOT use them in Python if/else; use an
    # OpaqueFunction for that (see localization.launch.py for an example).
    autonomous   = LaunchConfiguration('autonomous')
    # use_nav2         = LaunchConfiguration('use_nav2')
    # use_zone_nav     = LaunchConfiguration('use_zone_nav')
    # use_zed          = LaunchConfiguration('use_zed')
    # use_hesai        = LaunchConfiguration('use_hesai')
    # use_micro_ros    = LaunchConfiguration('use_micro_ros')
    # use_localization = LaunchConfiguration('use_localization')
    # use_cmd_vel_mux  = LaunchConfiguration('use_cmd_vel_mux')
    use_rviz         = LaunchConfiguration('use_rviz')
    # mux_mode         = LaunchConfiguration('mux_mode')
    fastlio_config   = LaunchConfiguration('fastlio_config')
    # waypoints_file   = LaunchConfiguration('waypoints_file')
    startup_delay = LaunchConfiguration('startup_delay')

    # ── Argument declarations ──────────────────────────────────────────────
    # Defaults match the jetson.env full-hardware profile.
    # Override per-session at the CLI without editing this file:
    #   ros2 launch challenge_bringup challenge_master.launch.py use_nav2:=true
    # declare_use_autonomous = DeclareLaunchArgument('autonomous', default_value='true')
    # declare_use_nav2 = DeclareLaunchArgument('use_nav2', default_value='false')
    # declare_use_zone_nav = DeclareLaunchArgument('use_zone_nav', default_value='false')
    # declare_use_zed = DeclareLaunchArgument('use_zed', default_value='true')
    # declare_use_hesai = DeclareLaunchArgument('use_hesai', default_value='true')
    # declare_use_micro_ros = DeclareLaunchArgument('use_micro_ros', default_value='false')
    # declare_use_localization = DeclareLaunchArgument('use_localization', default_value='true')
    # declare_use_cmd_vel_mux = DeclareLaunchArgument('use_cmd_vel_mux', default_value='true')
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='false')
    # declare_mux_mode = DeclareLaunchArgument('mux_mode', default_value='AUTONOMOUS')
    declare_fastlio_config = DeclareLaunchArgument('fastlio_config', default_value='fast_lio_hesai_qt64.yaml')
    declare_autonomous = DeclareLaunchArgument('autonomous', default_value='true')

    declare_startup_delay = DeclareLaunchArgument(
        'startup_delay',
        default_value='5.0',
        description='Seconds to wait after starting the Hesai driver before '
                    'launching the rest of the stack (lets the sensor come online).',
    )

    # ── BLOCK 1: Hesai QT64 lidar driver ──────────────────────────────────────
    # Connects to the lidar over UDP and publishes /hesai/points
    # (sensor_msgs/PointCloud2 @ ~10 Hz).  This is FAST-LIO2's primary
    # input for scan matching and map building.
    # Network settings come from DIY_HESAI_* env vars set by the profile.
    hesai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hesai_ros_driver'),
                'launch',
                'start.py',
            )
        ),
    )

    # ── BLOCK 2: Robot description launch (URDF → TF static transforms and joint transforms) ──
    # MUST be first.  robot_state_publisher reads the URDF and broadcasts
    # every joint as a static TF transform (base_link → lidar_link, imu_link,
    # camera_link, etc.).  All downstream nodes depend on these transforms
    # to project sensor data into robot-body coordinates.
    
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_description'),
                'launch',
                'description.launch.py',
            )
        ),
    )

    # ── BLOCK 3: Fast-LIO2 Launch  ──────────────────────────
    # Launches the Fast-LIO2 localizer, which performs real-time LiDAR-inertial odometry and mapping.
    # Needs the Hesai lidar driver and IMU to be running.
    #
    # pcd_save_en: only accumulate a PCD map while driving manually (mapping
    # runs); autonomous runs replay an existing map, so skip the save.
    pcd_save_en = PythonExpression(["'false' if '", autonomous, "' == 'true' else 'true'"])

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fast_lio_ros2'),
                'launch',
                'lio_localizer.launch.py',
            )
        ),
        launch_arguments={
            'pcd_save_en': pcd_save_en,
        }.items(),
    )


    map_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('map_localizer'),
                'launch',
                'map_localizer_launch.py',
            )
        ),
        condition=IfCondition(autonomous),
        launch_arguments={
            'use_rviz': 'false',
        }.items(),
    )

    # ── Block 4: loop_pgo: loop closure / pose-graph optimization ─────────────────
    # Only runs during manual mapping drives (autonomous=false); autonomous
    # runs localize against an already-finished map, so no PGO is needed.
    loop_pgo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('loop_pgo'),
                'launch',
                'loop_pgo_launch.py',
            )
        ),
        condition=UnlessCondition(autonomous),
    )

    # ── Block 5: map_hba: hierarchical bundle adjustment map refinement ────────────
    # Runs after loop_pgo, same gating (manual mapping drives only).
    map_hba_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('map_hba'),
                'launch',
                'map_hba_launch.py',
            )
        ),
        condition=UnlessCondition(autonomous),
    )


    # ── BLOCK 6: Nav2 autonomous navigation stack ─────────────────────────────
    # Nav2 planning + control stack: MPPI controller, SmacHybrid planner,
    # Behaviour Trees, global/local costmaps, lifecycle manager.
    # Does NOT start AMCL or map_server — per the Navigation Design Guide,
    # NDT-OMP provides the map→odom TF directly from the 3D point cloud map.
    # Requires use_localization=true for /tf and /odometry/filtered inputs.
    # Publishes /cmd_vel_nav which the mux forwards when in AUTONOMOUS mode.
    #
    # pure_pursuit_navigation.launch.py takes plain DeclareLaunchArgument
    # values (not a Nav2-style params_file), so pure_pursuit.yaml's
    # ros__parameters are read here and forwarded individually as
    # launch_arguments — edit that yaml to change map_yaml, velocities, etc.
    pure_pursuit_config = os.path.join(
        get_package_share_directory('motion_planner'),
        'config',
        'pure_pursuit.yaml',
    )
    with open(pure_pursuit_config, 'r') as f:
        pure_pursuit_params = yaml.safe_load(f)['/**']['ros__parameters']

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('motion_planner'),
                'launch',
                'pure_pursuit_navigation.launch.py',   # no AMCL — NDT-OMP handles map→odom
            )
        ),
        condition=IfCondition(autonomous),
        launch_arguments={
            key: str(value) for key, value in pure_pursuit_params.items()
        }.items(),
    )

    # ── BLOCK 13: RViz2  (developer / debug visualisation) ─────────────────────
    # Disabled by default to conserve CPU/GPU on the Jetson during competition.
    # Enable for debugging:  ros2 launch ... use_rviz:=true
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
    )

    # ── Mode-specific sets ──────────────────────────────────────────────────
    # Populate these with whichever launches/nodes should run only in that
    # mode (e.g. nav2_launch/zone_nav_launch under autonomous_only, a
    # joystick teleop node under manual_only). Gated on use_autonomous so
    # exactly one group is active at a time.
    autonomous_only = GroupAction(
        condition=IfCondition(autonomous),
        actions=[
            # TODO: autonomous-only launches/nodes
        ],
    )
    manual_only = GroupAction(
        condition=UnlessCondition(autonomous),
        actions=[
            # TODO: manual-only launches/nodes
        ],
    )

    # Everything except the Hesai driver itself waits hesai_startup_delay
    # seconds so the sensor is online before FAST-LIO2 and the rest of the
    # stack start consuming /hesai/points.
    delayed_fast_lio = TimerAction(
        period=startup_delay,
        actions=[
            fast_lio_launch,
        ],
    )

    delayed_map_localizer = TimerAction(
            period=startup_delay,
            actions=[
                map_localizer_launch,
            ],
        )

    delayed_nav2 = TimerAction(
            period=startup_delay,
            actions=[
                nav2_launch,
            ],
        )

    delayed_loop_pgo = TimerAction(
        period=startup_delay,
        actions=[
            loop_pgo_launch,
            map_hba_launch,
        ],
    )

    return LaunchDescription([
        # Declare launch Arguments
        declare_fastlio_config,
        declare_startup_delay,
        declare_autonomous,

        # Launch sequence starts here
        robot_description_launch,
        hesai_launch,
        delayed_fast_lio,
        delayed_nav2,
        delayed_map_localizer,
        delayed_loop_pgo,
    ])
