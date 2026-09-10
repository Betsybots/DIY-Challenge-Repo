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
   1. diy_robot_description  — publishes URDF / TF tree  (MUST be first)
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
  fastlio_config     str   FAST-LIO2 config filename            (see diy_localization/config/)
  waypoints_file     str   Override zone_waypoints.yaml path    (default: package default)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _zone_nav_launch(context, use_zone_nav_lc, waypoints_file_lc):
    """
    OpaqueFunction that conditionally includes zone_nav.launch.py.

    OpaqueFunction is used (rather than a plain IncludeLaunchDescription with
    IfCondition) because we need to inspect the waypoints_file value at runtime
    to decide whether to forward it or let zone_nav.launch.py use its own
    default.  IfCondition can gate the include but cannot modify launch_arguments
    based on another argument's resolved value.
    """
    if context.perform_substitution(use_zone_nav_lc).lower() != 'true':
        return []

    zone_nav_pkg = get_package_share_directory('diy_zone_nav')
    zone_nav_launch = os.path.join(zone_nav_pkg, 'launch', 'zone_nav.launch.py')

    launch_args = {}
    wp_file = context.perform_substitution(waypoints_file_lc).strip()
    if wp_file:
        # Caller overrode the waypoints file (e.g. for a custom map run).
        launch_args['waypoints_file'] = wp_file

    # Tell zone_nav.launch.py NOT to start another gate node — master launches
    # it unconditionally under IfCondition(use_localization) so the EKF always
    # has a /lidar_odometry_gated input, even when use_zone_nav=false.
    launch_args['launch_gate'] = 'false'

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zone_nav_launch),
            launch_arguments=launch_args.items(),
        )
    ]


def _zed_launch(context, use_zed_lc):
    """
    OpaqueFunction that conditionally includes zed_wrapper's zed_camera.launch.py.

    Same reasoning as _zone_nav_launch above: get_package_share_directory()
    must not run at all when the camera is disabled, since zed_wrapper is
    only installed on the Jetson (not in this repo, not in dev/laptop
    environments) — a plain IncludeLaunchDescription+IfCondition would still
    call get_package_share_directory('zed_wrapper') unconditionally while
    building the launch description, raising PackageNotFoundError and
    crashing the whole master launch on any machine without it installed.
    """
    if context.perform_substitution(use_zed_lc).lower() != 'true':
        return []

    zed_pkg = get_package_share_directory('zed_wrapper')
    zed_launch = os.path.join(zed_pkg, 'launch', 'zed_camera.launch.py')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={
                'camera_model':   'zed2i',
                # All three default true in zed_wrapper. Forced false here:
                # this stack's TF tree is already fully owned elsewhere —
                # diy_robot_description publishes the static camera_link
                # transform, and FAST-LIO2 + the localization EKF + NDT-OMP own the dynamic
                # odom->base_link and map->odom transforms. Leaving these at
                # their defaults would start a second, competing
                # robot_state_publisher + TF broadcaster for the same frames.
                'publish_urdf':   'false',
                'publish_tf':     'false',
                'publish_map_tf': 'false',
            }.items(),
        )
    ]


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
    use_joystick     = LaunchConfiguration('use_joystick')
    use_nav2         = LaunchConfiguration('use_nav2')
    use_zone_nav     = LaunchConfiguration('use_zone_nav')
    use_zed          = LaunchConfiguration('use_zed')
    use_motor_driver = LaunchConfiguration('use_motor_driver')
    use_hesai        = LaunchConfiguration('use_hesai')
    use_micro_ros    = LaunchConfiguration('use_micro_ros')
    use_localization = LaunchConfiguration('use_localization')
    use_cmd_vel_mux  = LaunchConfiguration('use_cmd_vel_mux')
    use_rviz         = LaunchConfiguration('use_rviz')
    mux_mode         = LaunchConfiguration('mux_mode')
    fastlio_config   = LaunchConfiguration('fastlio_config')
    waypoints_file   = LaunchConfiguration('waypoints_file')

    return LaunchDescription([

        # ── BLOCK 1: Argument declarations ────────────────────────────────────
        # Defaults match the jetson.env full-hardware profile.
        # Override per-session at the CLI without editing this file:
        #   ros2 launch challenge_bringup challenge_master.launch.py use_nav2:=true
        DeclareLaunchArgument('use_joystick',     default_value='false'),
        DeclareLaunchArgument('use_nav2',         default_value='false'),
        DeclareLaunchArgument('use_zone_nav',     default_value='false'),
        DeclareLaunchArgument('use_zed',           default_value='true'),
        DeclareLaunchArgument('use_motor_driver', default_value='true'),
        DeclareLaunchArgument('use_hesai',        default_value='true'),
        DeclareLaunchArgument('use_micro_ros',    default_value='false'),
        DeclareLaunchArgument('use_localization', default_value='true'),
        DeclareLaunchArgument('use_cmd_vel_mux',  default_value='true'),
        DeclareLaunchArgument('use_rviz',         default_value='false'),
        DeclareLaunchArgument('mux_mode',         default_value='AUTONOMOUS'),
        DeclareLaunchArgument(
            'fastlio_config',
            default_value='fast_lio_hesai_qt64.yaml',
            # Change to swap sensor configs without editing this file
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='',   # empty → zone_nav.launch.py uses its package default
            description='Absolute path to zone_waypoints.yaml; leave blank for the default',
        ),

        # ── BLOCK 2: Robot description  (URDF → TF static transforms) ─────────
        # MUST be first.  robot_state_publisher reads the URDF and broadcasts
        # every joint as a static TF transform (base_link → lidar_link, imu_link,
        # camera_link, etc.).  All downstream nodes depend on these transforms
        # to project sensor data into robot-body coordinates.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('diy_robot_description'),
                    'launch',
                    'description.launch.py',
                )
            ),
        ),

        # ── BLOCK 3: micro-ROS agent  (STM32 ↔ ROS 2 serial bridge) ──────────
        # Opens a serial link to the STM32 over USB.  The STM32 firmware runs a
        # micro-ROS executor that publishes:
        #   /stm32/heartbeat  — 10 Hz liveness signal (watchdog source for estop)
        #   /stm32/estop      — hardware e-stop button state
        # These become full ROS 2 topics only after this agent is running.
        # Serial device and baud rate read from DIY_MICRO_ROS_* env vars set by
        # the active device profile (scripts/env.sh).
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            output='screen',
            condition=IfCondition(use_micro_ros),   # skip on laptop / no-hardware runs
            arguments=[
                'serial',
                '--dev', os.environ.get('DIY_MICRO_ROS_SERIAL',   '/dev/ttyACM0'),
                '-b',    os.environ.get('DIY_MICRO_ROS_BAUDRATE',  '921600'),
            ],
        ),

        # ── BLOCK 4: E-stop controller  (advisory ROS mirror of STM32 state) ──
        # Listens to /stm32/estop (hardware button) and /stm32/heartbeat
        # (watchdog), then publishes /estop_active (Bool, latched QoS).
        # The mux (BLOCK 5) reads /estop_active and zeroes all velocity output.
        #
        # This robot has no STM32. The real hardware e-stop is an RJ45
        # break-loop wired directly into the motor power path (fail-safe:
        # loop open -> motors de-energized, independent of software/firmware
        # -- this is what satisfies competition rule 1.2.7's "must fail safe
        # within 1 second" requirement, not this node).
        #
        # Gated on use_micro_ros (same as BLOCK 3): this node's fail-safe
        # design (see diy_estop_controller's module docstring) treats
        # "/stm32/heartbeat never received" as a permanent E-stop condition,
        # clearable only once a heartbeat arrives -- so on a robot with no
        # STM32, running it unconditionally would permanently latch
        # /estop_active=True and lock cmd_vel_mux's ESTOP_LOCK forever.
        # Gating it means cmd_vel_mux simply never receives an /estop_active
        # message at all, which is safe: it defaults _estop_active=False and
        # only sets it True on an actual received message (see
        # cmd_vel_mux_node.py) -- no message means no false-positive lock.
        Node(
            package='diy_estop_controller',
            executable='estop_controller_node',
            name='estop_controller_node',
            output='screen',
            condition=IfCondition(use_micro_ros),
        ),

        # ── BLOCK 5: cmd_vel multiplexer  (velocity source arbitration) ───────
        # Arbitrates between /cmd_vel_joy (joystick) and /cmd_vel_nav (Nav2)
        # and emits the winning command on /cmd_vel_safe.
        # Startup mode controls which source is active at launch:
        #   AUTONOMOUS — forward /cmd_vel_nav  (competition default)
        #   JOYSTICK   — forward /cmd_vel_joy  (manual driving / testing)
        #   ESTOP_LOCK — zero velocity regardless of any inputs
        # Switch mode at runtime:  ros2 param set /cmd_vel_mux_node mode JOYSTICK
        #
        # Single-owner on a multi-machine robot: exactly one device should run
        # this node. Unlike robot_state_publisher (safe to duplicate — static,
        # idempotent TF), two instances here would both independently publish
        # /cmd_vel_safe, racing to drive the motors. This robot runs it on the
        # RPi (co-located with the motor driver and e-stop wiring) — set
        # use_cmd_vel_mux=false on any other device.
        Node(
            package='diy_cmd_vel_mux',
            executable='cmd_vel_mux_node',
            name='cmd_vel_mux_node',
            output='screen',
            condition=IfCondition(use_cmd_vel_mux),
            parameters=[{'mode': mux_mode}],
        ),

        # ── BLOCK 6: Hesai QT64 lidar driver ──────────────────────────────────
        # Connects to the lidar over UDP and publishes /hesai/points
        # (sensor_msgs/PointCloud2 @ ~10 Hz).  This is FAST-LIO2's primary
        # input for scan matching and map building.
        # Network settings come from DIY_HESAI_* env vars set by the profile.
        Node(
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            name='hesai_ros_driver',
            output='screen',
            condition=IfCondition(use_hesai),
            parameters=[{
                'device_ip':  os.environ.get('DIY_HESAI_IP',       '192.168.1.201'),
                'lidar_port': int(os.environ.get('DIY_HESAI_PORT',  '2368')),
                'frame_id':   os.environ.get('DIY_HESAI_FRAME_ID',  'lidar_link'),
            }],
        ),

        # NOTE: the ACEINNA IMU driver (imu_can_interface) is launched
        # INDEPENDENTLY on the RPi, outside this repo — not vendored here,
        # not part of this launch file. FAST-LIO2's internal EKF and the
        # localization EKF (BLOCK 8 below, on the Jetson) still consume
        # /imu/data as an external input over the Zenoh bridge.

        # ── BLOCK 7: ZED2i camera  (RGB-D + stereo + IMU) ──────────────────────
        # Requires the zed_wrapper package (Stereolabs ZED SDK +
        # zed-ros2-wrapper: https://github.com/stereolabs/zed-ros2-wrapper),
        # installed directly on the Jetson (same pattern as hesai_ros_driver —
        # not vendored in this repo). CONFIRMED this is the actual driver in
        # use on the real robot. Still not yet run end-to-end in this repo's
        # launch — confirm topics with `ros2 topic list` on first real run.
        #
        # Publishes (default camera_name/namespace "zed", node_name "zed_node"):
        #   /zed/zed_node/rgb/image_rect_color   — RGB (recording / visual tasks)
        #   /zed/zed_node/depth/depth_registered — depth map
        #   /zed/zed_node/left/image_rect_color  — left rectified  (VSLAM input)
        #   /zed/zed_node/right/image_rect_color — right rectified (VSLAM input)
        #   /zed/zed_node/imu/data               — 6-DOF IMU
        #
        # See _zed_launch() above for why this is an OpaqueFunction and why
        # publish_urdf/publish_tf/publish_map_tf are forced false.
        OpaqueFunction(function=_zed_launch, args=[use_zed]),

        # ── BLOCK 8: Localisation stack  (FAST-LIO2 + single EKF + NDT-OMP) ──
        # Delegates to localization.launch.py with fixed runtime arguments.
        # use_rviz is suppressed here — BLOCK 12 manages the single shared
        # RViz instance to avoid duplicate visualisation windows on screen.
        # NOTE: localization.launch.py no longer takes a use_gps argument —
        # map->odom is now provided by NDT-OMP matching against the offline
        # LIO-SAM map (GPS/navsat_transform was dropped; see the launch
        # file's module docstring for why).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('diy_localization'),
                    'launch',
                    'localization.launch.py',
                )
            ),
            condition=IfCondition(use_localization),
            launch_arguments={
                'mode':        'runtime',
                'config_file': fastlio_config,
                'use_rviz':    'false',    # master owns the single RViz instance
            }.items(),
        ),

        # ── BLOCK 9: Joystick teleop ───────────────────────────────────────────
        # Includes joystick_drive.launch.py which starts:
        #   joy_node         — reads gamepad → sensor_msgs/Joy
        #   teleop_twist_joy — converts Joy → geometry_msgs/Twist on /cmd_vel_joy
        # The mux must be in JOYSTICK mode to forward /cmd_vel_joy to the output:
        #   ros2 param set /cmd_vel_mux_node mode JOYSTICK
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'joystick_drive.launch.py')
            ),
            condition=IfCondition(use_joystick),
        ),

        # ── BLOCK 10: Motor driver  (differential drive) ──────────────────────
        # Subscribes to /cmd_vel_safe (the mux's arbitrated, estop-gated output)
        # and converts Twist → left/right wheel PWM commands.
        #
        # IMPORTANT: the remap /cmd_vel → /cmd_vel_safe is intentional.
        # Never wire any hardware actuator directly to /cmd_vel — all commands
        # must pass through the mux so estop and mode arbitration cannot be
        # bypassed by any individual node.
        Node(
            package='differential-drive',
            executable='differential-drive',
            name='differential_drive',
            output='screen',
            condition=IfCondition(use_motor_driver),
            remappings=[('/cmd_vel', '/cmd_vel_safe')],   # read mux output, not raw /cmd_vel
        ),

        # ── BLOCK 11: Nav2 autonomous navigation stack ─────────────────────────
        # Nav2 planning + control stack: MPPI controller, SmacHybrid planner,
        # Behaviour Trees, global/local costmaps, lifecycle manager.
        # Does NOT start AMCL or map_server — per the Navigation Design Guide,
        # NDT-OMP provides the map→odom TF directly from the 3D point cloud map.
        # Requires use_localization=true for /tf and /odometry/filtered inputs.
        # Publishes /cmd_vel_nav which the mux forwards when in AUTONOMOUS mode.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'navigation_launch.py',   # no AMCL — NDT-OMP handles map→odom
                )
            ),
            condition=IfCondition(use_nav2),
            launch_arguments={
                'params_file':  nav2_params,  # MPPI critics + costmap config
                'use_sim_time': 'false',      # always false on real hardware
                'autostart':    'true',       # lifecycle nodes activate automatically
            }.items(),
        ),

        # ── BLOCK 12: Zone-aware navigation state machine ──────────────────────
        # Monitors robot position against course zones and adjusts Nav2 behaviour
        # (speed limit via /speed_limit, costmap inflation, obstacle layer enable)
        # and the cmd_vel_mux mode (AUTONOMOUS ↔ BLIND_DRIVE) accordingly.
        #
        # Launches zone_nav_manager_node only (launch_gate:=false is passed so
        # zone_nav.launch.py does NOT start a second gate node — we start it
        # below under Block 12b, always, whenever use_localization is true).
        #
        # REQUIRES: use_nav2=true — the state machine calls Nav2 costmap and
        # controller_server SetParameters services.  Starting zone_nav without
        # Nav2 is harmless (service calls are skipped when the service is not
        # ready) but the robot will not respond to zone transitions.
        #
        # Optional: pass waypoints_file:=/absolute/path/to/zone_waypoints.yaml
        # to override the default shipped with the package.
        OpaqueFunction(function=_zone_nav_launch, args=[use_zone_nav, waypoints_file]),

        # ── BLOCK 12b: Lidar odometry gate ─────────────────────────────────────
        # MUST run whenever localization is active — NOT only when zone_nav is on.
        # The localization EKF (ekf_odom.yaml, odom1) subscribes to
        # /lidar_odometry_gated. Without this node the EKF receives no lidar
        # odometry input and degrades to wheel + IMU-angular-rate fusion only
        # (dead reckoning), causing significant drift outdoors.
        # Default nav_mode is "INIT" which leaves the gate open (pass-through),
        # so localization accuracy is unaffected when zone_nav is disabled.
        Node(
            package='diy_zone_nav',
            executable='lidar_odom_gate_node',
            name='lidar_odom_gate',
            output='screen',
            condition=IfCondition(use_localization),
        ),

        # ── BLOCK 13: RViz2  (developer / debug visualisation) ─────────────────
        # Disabled by default to conserve CPU/GPU on the Jetson during competition.
        # Enable for debugging:  ros2 launch ... use_rviz:=true
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
