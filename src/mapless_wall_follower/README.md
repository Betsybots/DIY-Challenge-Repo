# Mapless Ackermann Wall Follower

This standalone ROS 2 node drives from the FAST-LIO body-frame point cloud. It
does not use a map, localization, Nav2, a global path, or odometry.

## Behavior

1. `CENTERING`: when both walls are valid, track their centerline.
2. `LEFT_WALL`: when only the left wall is valid, follow it at the configured
   distance.
3. `RIGHT_WALL`: when only the right wall is valid, follow it at the configured
   distance. A right-turn bias is added while the front is blocked.
4. `NO_WALL_DETECTED`: stop when neither wall is valid.
5. Also stop if the cloud is stale or the front emergency distance is crossed.

The controller publishes its current state on
`/mapless_wall_follower/state`.

The controller prints a `CONTROL DEBUG` line in its launch terminal at the
configured `debug_log_frequency`. It reports cloud age, front distance,
retained point counts, wall validity, wall positions and headings, inlier
counts, RMS fit errors, controller errors, curvature, requested commands, and
rate-limited commands.

The controller starts disabled and publishes stop commands. Enable motion only
after verifying the cloud and command mux:

```bash
ros2 topic pub --once /mapless_wall_follower/enable std_msgs/msg/Bool "{data: true}"
```

Disable it with:

```bash
ros2 topic pub --once /mapless_wall_follower/enable std_msgs/msg/Bool "{data: false}"
```

## Build

```bash
cd ~/DIY-Challenge-Repo
colcon build --symlink-install --packages-select mapless_wall_follower
source install/setup.bash
```

## Launch

Do not use the shared `challenge_bringup/master.launch.py` for this test because
it also starts Nav2 or mapping components. Start only the dependencies below.

### 1. Start the IMU

On the Raspberry Pi, start the external IMU driver:

```bash
source ~/driveStack/install/setup.bash
ros2 launch imu_can_interface imu_sensors.launch.py
```

Verify that it publishes:

```bash
ros2 topic echo /imu/data --once
```

### 2. Start the robot description

On the Jetson:

```bash
source ~/DIY-Challenge-Repo/install/setup.bash
ros2 launch robot_description description.launch.py
```

### 3. Start the Hesai LiDAR

```bash
source ~/DIY-Challenge-Repo/install/setup.bash
ros2 launch hesai_ros_driver start.py
```

Verify the LiDAR input:

```bash
ros2 topic hz /lidar_points
```

The expected scan rate is approximately 10 Hz.

### 4. Start FAST-LIO

Start FAST-LIO only after `/lidar_points` and `/imu/data` are available:

```bash
source ~/DIY-Challenge-Repo/install/setup.bash
ros2 launch fast_lio_ros2 lio_localizer.launch.py
```

Verify the wall-follower input:

```bash
ros2 topic echo /cloud_registered_body --once
ros2 topic hz /cloud_registered_body
```

The cloud must have `frame_id: base_link`, with +X forward, +Y left, and +Z up.

### 5. Start the Ackermann driver for a moving test

On the computer connected to the CAN bus and steering servo:

```bash
source ~/driveStack/install/setup.bash
ros2 launch ackermann-drive ackermann_autonomous.launch.py
```

The current Ackermann driver listens on `/cmd_vel_smoothed`.

### 6. Start the wall follower

For a stationary controller test, leave the default output on `/cmd_vel_nav`:

```bash
ros2 launch mapless_wall_follower mapless_wall_follower_ackermann.launch.py
```

For a direct moving test without a command mux or separate velocity smoother,
connect the wall follower directly to the current Ackermann driver:

```bash
ros2 launch mapless_wall_follower \
  mapless_wall_follower_ackermann.launch.py \
  cmd_vel_topic:=/cmd_vel_smoothed
```

For the future production pipeline, keep the default arrangement:

```text
wall follower -> /cmd_vel_nav -> safety/mux/smoother
              -> /cmd_vel_smoothed -> Ackermann driver
```

Do not run Nav2 or another autonomous controller on the same command topic.

### 7. Verify before enabling

Confirm the expected connections:

```bash
ros2 topic info /cloud_registered_body --verbose
ros2 topic info /cmd_vel_smoothed --verbose
ros2 topic echo /mapless_wall_follower/state
```

The state must initially be `DISABLED`, and the command must be zero. Keep the
physical emergency stop ready, then enable:

```bash
ros2 topic pub --once /mapless_wall_follower/enable \
  std_msgs/msg/Bool "{data: true}"
```

Start field testing with `straight_speed` between `0.20` and `0.30 m/s`.
Verify the state transitions and emergency stop before increasing speed.

## Key tuning parameters

Tune one group at a time and record the `CONTROL DEBUG` output. The speed-course
path is 36 in (0.9144 m) wide and the vehicle is approximately 16 in
(0.4064 m) wide, giving about 10 in (0.254 m) clearance per side when centered.

### 1. Wall detection

- `min_wall_points`: minimum inliers needed to accept a wall. Lower it if real
  walls are frequently invalid; raise it if small objects are accepted as walls.
- `fit_residual_threshold`: maximum point-to-line distance for an inlier. Raise
  it for rough walls; lower it when obstacles contaminate the fit.
- `max_fit_rms`: maximum accepted overall fit error. Lower it to reject noisy
  fits.
- `min_z`, `max_z`: vertical wall slice. Adjust these only after confirming
  that `/cloud_registered_body` uses +Z upward.
- `min_x`, `max_x`: longitudinal fitting region. More forward range provides
  earlier information but can mix wall segments across tight bends.
- `side_min_abs_y`, `side_max_abs_y`: lateral fitting region. Keep
  `side_min_abs_y` outside the vehicle body and wheels.

Watch `left_points`, `right_points`, `left_inliers`, `right_inliers`,
`left_rms`, `right_rms`, `left_valid`, and `right_valid` in the terminal.

### 2. Position in the course

- `right_wall_target_distance`: desired vehicle-center distance from the right
  wall in `RIGHT_WALL`.
- `left_wall_target_distance`: desired vehicle-center distance from the left
  wall in `LEFT_WALL`.
- `wall_lookahead`: distance ahead where fitted wall positions are evaluated.

Both wall targets default to 0.46 m, the center of the 0.9144 m course. Increase
the target on a side to move farther away from that wall. Decrease it to move
closer.

### 3. Steering response

- `center_gain`: centering correction when both walls are visible. Reduce it if
  the vehicle oscillates; increase it if lateral correction is too slow.
- `heading_gain`: alignment with the fitted wall direction. Reduce it if
  steering is nervous; increase it if the vehicle remains angled to the walls.
- `right_wall_gain`, `left_wall_gain`: distance correction in single-wall
  modes. Tune them independently if one side tracks differently.
- `right_turn_curvature_bias`: extra right curvature when only the right wall is
  valid and the front is closing. Reduce it if turns are too sharp; increase it
  if right turns start too slowly.

Change gains in small steps of approximately 0.1 to 0.2.

### 4. Speed and turn slowdown

- `straight_speed`: speed used while both walls are valid.
- `turn_speed`: lower speed used while following only one wall.
- `max_lateral_acceleration`: automatically limits speed as curvature rises.
- `max_linear_acceleration`: forward speed ramp rate.
- `max_linear_deceleration`: controlled braking rate.
- `max_yaw_acceleration`: steering/yaw-command slew rate.

Use the staged progression documented in the YAML: 0.30/0.15 m/s first,
0.50/0.25 m/s after three clean runs, and 1.00/0.30 m/s only after the lower
speeds are reliable.

### 5. Turn and stop distances

- `turn_enter_front_distance`: distance at which right-wall mode adds its
  right-turn steering bias.
- `emergency_stop_distance`: front obstacle distance that commands a stop.
- `front_half_width`: half-width of the front obstacle-detection corridor.
- `cloud_timeout`: maximum accepted age of the latest point cloud.

Do not reduce `emergency_stop_distance` merely to avoid false stops. First
inspect `front_distance` and verify the point-cloud axes and front ROI. Increase
the turn and stop distances before increasing speed.

### Recommended tuning order

1. Verify point-cloud axes: +X forward, +Y left, +Z up.
2. Make left and right wall detection reliable while stationary.
3. Confirm both 0.46 m wall targets place the vehicle correctly.
4. Tune `center_gain`, then `heading_gain`, at low speed.
5. Tune the right and left single-wall gains.
6. Tune right-turn bias and front-distance thresholds.
7. Verify stopping behavior.
8. Increase speed last.
