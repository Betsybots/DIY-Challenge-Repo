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
5. `RECOVERY_BRAKING` -> `RECOVERY_REVERSING` -> `RECOVERY_SETTLING`: when an
   obstacle is within `emergency_stop_distance` along the commanded arc, stop,
   reverse `recovery_reverse_distance` while the heading turns away from the
   blocking wall (toward the path center), stop, pre-steer away, and resume.
   If the blocking point is nearly head-on, it keeps turning the way the
   controller was already steering. If `recovery_flip_after_attempts`
   back-to-back attempts fail, later attempts steer the opposite way as a
   fallback. After `recovery_max_attempts` back-to-back
   attempts without `recovery_reset_distance` of forward driving, it latches
   `EMERGENCY_FRONT_STOP`.
6. Also stop if the cloud is stale.

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

### Green-light start

`green_light_trigger_node` subscribes to `/color_detector/green_light`
(published by `diy_zed_color_detection`). After `required_consecutive`
consecutive `true` frames it latches and publishes `true` on
`/mapless_wall_follower/enable` `enable_publish_count` times at
`enable_publish_rate`, only once the wall follower is subscribed. It never
publishes `false`; stop with the disable command above or the physical
emergency stop. Restart the trigger node to arm it again.

The trigger is started only with `wait_for_green_light:=true` and reads the
`green_light_trigger` section of the same YAML:

- `green_light_topic`: detector output topic.
- `enable_topic`: wall-follower enable topic.
- `required_consecutive`: debounce against false green detections.
- `enable_publish_count`, `enable_publish_rate`: enable repeats.

```bash
ros2 launch diy_zed_color_detection color_detector.launch.py
ros2 launch mapless_wall_follower mapless_wall_follower_ackermann.launch.py \
  wait_for_green_light:=true
```

Verify without the camera by publishing a fake green light:

```bash
ros2 topic pub -r 10 /color_detector/green_light std_msgs/msg/Bool "{data: true}"
ros2 topic echo /mapless_wall_follower/enable
```

## Automatic MPPI fallback

The obstacle-course MPPI launches start this controller automatically with:

```text
start_enabled = true
cmd_vel_topic = /cmd_vel_wall_follower
```

`/cmd_vel_wall_follower` is a private recovery input and does not command the
vehicle while MPPI is operating normally. If planning or MPPI path following
fails after its contextual recovery, the Nav2 behavior tree:

1. Runs `AssistedTeleop` for 5 seconds.
2. Collision-checks and forwards the wall-follower command through the MPPI
   velocity smoother to `/cmd_vel_nav`.
3. Stops the recovery command and retries the same Nav2 goal.
4. Repeats this sequence up to three times.
5. Returns final navigation failure after the third unsuccessful retry.

The MPPI progress checker requires 0.20 m of movement within 8 seconds. The
behavior tree clears the local costmap and retries MPPI once before invoking
the outer wall fallback, so a completely stationary controller can take
approximately 16 seconds before wall following starts.

During fallback, `AssistedTeleop` checks only the local costmap and full robot
footprint. It permits soft inflation costs so the robot can leave an inflation
area, but scales or stops commands projected into a lethal obstacle. If the
current footprint already overlaps lethal cells, the fallback may be unable to
move and will retry MPPI after its 5-second allowance.

The automatic fallback is enabled by:

```bash
ros2 launch challenge_bringup \
  master_mppi_wall_fallback_obstacle_course_ackermann.launch.py
```

The explicitly named speed-filter variant is:

```bash
ros2 launch challenge_bringup \
  master_mppi_speed_filter_wall_fallback_obstacle_course_ackermann.launch.py
```

The original MPPI launch names remain available without wall fallback. Manual
enable and direct `/cmd_vel_smoothed` output are only for standalone
wall-follower tests.

## Build

```bash
cd ~/DIY-Challenge-Repo
colcon build --symlink-install --packages-select mapless_wall_follower
source install/setup.bash
```

## Launch

Do not use the shared `challenge_bringup/master.launch.py` for this test because
it also starts Nav2 or mapping components. Start only the dependencies below.

### Jetson one-command bringup

`challenge_bringup/launch/mapless_wall_follower_jetson.sh` runs steps 2-4 and 6
below in order, waiting for each required topic before the next step:

1. Waits for `/imu/data` (the IMU still runs on the Raspberry Pi).
2. Starts `robot_description`.
3. Starts the Hesai LiDAR and waits for `/lidar_points`.
4. Starts FAST-LIO and waits for `/cloud_registered_body`.
5. Starts the wall follower in the foreground so `CONTROL DEBUG` is visible.

Start the IMU (step 1) and the Ackermann driver (step 5) separately first.
Script options come first; all remaining arguments are forwarded to the
wall-follower launch:

- `--terminals`: open each launch in its own terminal window instead of
  logging to files.
- `--green-light`: also start `diy_zed_color_detection` (with the ZED wrapper),
  wait for `/color_detector/green_light`, and launch the wall follower with
  `wait_for_green_light:=true`. Passing `wait_for_green_light:=true` directly
  does the same.

```bash
cd ~/DIY-Challenge-Repo
# Stationary test (output /cmd_vel_nav)
src/challenge_bringup/launch/mapless_wall_follower_jetson.sh
# Direct moving test to the Ackermann driver
src/challenge_bringup/launch/mapless_wall_follower_jetson.sh \
  cmd_vel_topic:=/cmd_vel_smoothed
# Race start on the green light, one window per launch
src/challenge_bringup/launch/mapless_wall_follower_jetson.sh --terminals \
  --green-light cmd_vel_topic:=/cmd_vel_smoothed
```

Environment overrides:

- `DIY_WS`: workspace root containing `install/setup.bash` (default: derived
  from the script location).
- `TOPIC_TIMEOUT`: seconds to wait for each topic (default `30`). The script
  exits if a topic does not appear.
- `CAMERA_TIMEOUT`: seconds to wait for `/color_detector/green_light`
  (default `90`). On timeout the script only warns and still starts the wall
  follower (disabled); the trigger stays armed, so enable manually or publish a
  fake green light as shown above.
- `LOG_DIR`: background launch logs (default
  `/tmp/mapless_wall_follower_<timestamp>`).

Ctrl+C stops all started launches in reverse order. Build the workspace first;
the script sources `install/setup.bash`.

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

- `straight_speed`: ceiling while both walls are valid (straight zones).
- `turn_speed`: ceiling while following only one wall.
- `min_speed`: floor while driving so the steering keeps authority.
- `max_lateral_acceleration`: main bend-speed knob. Speed is limited by
  `sqrt(a_lat / k)`, where `k` is the larger of the commanded curvature and the
  bend curvature previewed from `front_distance` and `preview_wall_offset`.
- `max_linear_acceleration`: forward speed ramp rate.
- `max_linear_deceleration`: controlled braking rate, also used to cap speed so
  the vehicle can stop before `emergency_stop_distance` along its arc.
- `max_curvature_rate`: steering (curvature) slew rate. Speed and curvature are
  rate-limited separately, and the yaw rate is published as `speed * curvature`
  so braking does not tighten the turn.
- `control_latency`: delay added to the braking distance.

See the YAML header for the staged speed progression and expected bend speeds.

### 5. Turn and stop distances

- `turn_enter_front_distance`: distance at which right-wall mode adds its
  right-turn steering bias.
- `emergency_stop_distance`: obstacle distance along the commanded arc that
  starts the reverse recovery.
- `recovery_reverse_distance`, `recovery_reverse_speed`: how far and how fast
  to back up. Distance is integrated from the command (no odometry).
- `recovery_curvature`: steering while reversing; higher turns away more.
- `recovery_pause`: zero-speed hold before and after reversing.
- `recovery_max_attempts`, `recovery_reset_distance`: retry limit; `0`
  attempts restores the plain emergency stop.

The recovery publishes negative `linear.x` with `angular.z = linear.x *
curvature`. Verify on blocks that the Ackermann driver reverses and that the
nose swings away from the wall; nothing behind the robot is checked.
- `front_half_width`: half-width of the front band and of the swept arc corridor.
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
