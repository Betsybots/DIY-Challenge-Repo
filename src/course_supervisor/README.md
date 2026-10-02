# course_supervisor

Picks which controller drives the robot, section by section, along the
obstacle course. It's optional: the existing launches
(`obstacle_course_jetson.sh`, `nav2_navigation_mppi_*_obstacle_course_ackermann.launch.py`)
are unchanged and remain the fallback.

## How it works

```
controller_server (MPPI, FollowPath) -> /cmd_vel_mppi -+
built-in path tracker (pure pursuit)                   +-> course_supervisor -> cmd_vel_mppi_raw
                                                       |     -> velocity_smoother -> /cmd_vel_smoother_out
                                                       |     -> course_supervisor -> /cmd_vel_smoothed
mapless_wall_follower -> /cmd_vel_wall_follower ------+-> course_supervisor -> /cmd_vel_smoothed (direct)
```

`/cmd_vel_smoothed` is what `ackermann-drive` (driveStack) subscribes to
(`Twist`: `linear.x` speed, `angular.z` yaw rate → steering
`atan(wheelbase·wz/vx)`, 1 s watchdog). `course_supervisor` is the only node
publishing it:
* Nav2 and the path tracker go through the velocity smoother (acceleration
  limits, forward-only).
* The wall follower's command goes to the drive unchanged, exactly as when it
  runs standalone on `/cmd_vel_smoothed`. That keeps its own rate limits and its
  reverse recovery, which the forward-only smoother would block. It's also fed
  to the smoother, so handovers start from the current speed.

Override the drive topic with `drive_cmd_vel_topic:=...` if needed.

* **Driving line + sections** (`config/obstacle_course.yaml`): the run3
  driving line in the map frame of `mcl_map.pcd`, and the sections along it,
  each with a `mode` (`nav2`, `wall_follower`, `path_tracker`, `stop`), a
  `lidar` flag and an optional tracker `speed`.
* **Progress**: the map pose (TF `map -> base_footprint`) is projected onto the
  line only within [s − 0.5 m, s + 2.5 m] of the current progress s, and s
  never goes back. That way a parallel lane elsewhere (e.g. the tunnel next to
  the wide section) can't capture it, and pose drift can't send it back a section.
* **Selector**: only the active section's controller output is forwarded; if it
  goes quiet for `cmd_timeout`, zero is sent.
  * `nav2`: the supervisor sends the rest of the driving line to
    `controller_server` (`FollowPath` action), starting `nav2_preload_distance`
    before a nav2 section. Only the controller and its local costmap run: no
    planner, global costmap or `map_server`.
  * `wall_follower`: `mapless_wall_follower` is enabled only here (via its
    enable topic).
  * `path_tracker`: pure pursuit on the driving line from the map pose,
    ignoring the costmap, so it pushes through the ramp and car wash.
* **Fallback**: if a section's mode is switched off, it uses `path_tracker`,
  then `nav2`.
* **Optional**:
  * mcl_3dl lidar gate per section (`drive_lidar_gate`).
  * AprilTag pose fixes into mcl_3dl (`use_apriltag_fix`).
  * AprilTag section triggers (`use_tag_triggers`).
  * Tag-map survey (`tag_survey_file`).

## The course (sections)

`config/obstacle_course.yaml` (74.6 m driving line, 2 laps by default):

| Section | Starts at s (m) | Mode | mcl_3dl lidar | Started by tag (`use_tag_triggers`) |
|---|---|---|---|---|
| start_lane | 0.0 | nav2 | on | |
| ramp_and_bridge | 2.7 | path_tracker 0.30 m/s | off | `RampDetection` (ID0) ≤ 0.95 m |
| helix | 8.1 | path_tracker 0.25 m/s | off | |
| tunnel | 14.1 | path_tracker 0.30 m/s | off | |
| narrow_path (tunnel exit → 20 in lane → 26 in turn → gravel) | 19.2 | wall_follower | on | `TUNNEL_EXIT` (ID2) ≤ 0.42 m |
| after_gravel_to_hoops (flat, bank, potholes, buckets, gap, hoops) | 30.3 | nav2 | on | `NARROW_PATH_END` (ID1) ≤ 0.89 m |
| car_wash | 71.8 | path_tracker 0.35 m/s | off | `CAR_WASH_ENTRY` (ID3) ≤ 0.82 m |
| finish | 74.2 | nav2 | on | |

Without tags (or with `use_tag_triggers:=false`) sections switch on odometry
progress alone. A tag only moves progress **forward** to its section.

## Running

```bash
# Full stack in terminals (fallback: obstacle_course_jetson.sh)
src/challenge_bringup/launch/obstacle_course_supervisor_jetson.sh [launch args]

# or through master.launch.py
ros2 launch challenge_bringup master.launch.py use_course_supervisor:=true use_lidar_gate:=true
```

Launch arguments of `course_supervisor.launch.py` (all switchable):

| Argument | Default | Effect |
|---|---|---|
| `course_file` | config/obstacle_course.yaml | driving line + sections |
| `supervisor_params_file` | config/course_supervisor.yaml | supervisor parameters |
| `drive_cmd_vel_topic` | /cmd_vel_smoothed | the drive's input; only course_supervisor publishes it |
| `laps` | 2 | laps of the driving line before stopping |
| `wait_for_green_light` | true | wait for `/green_light` (`std_msgs/Bool`) |
| `use_nav2` | true | controller_server (MPPI) + FollowPath on the driving line |
| `nav2_params_file` / `nav2_override_params_file` | challenge_bringup nav2 params / obstacle-course MPPI params | Nav2 parameters |
| `use_speed_filter` | false | speed mask on the local costmap (`mask_yaml_file`, in the current map frame) |
| `use_wall_follower` | true | start mapless_wall_follower for wall_follower sections |
| `wall_follower_params_file` / `wall_follower_override_file` | wall follower defaults / config/mapless_wall_follower_obstacle_course.yaml | narrow-lane tuning |
| `use_path_tracker` | true | allow the push-through tracker |
| `drive_lidar_gate` | false (script: true) | publish `mcl_measurement_enabled` per section; start mcl_3dl with `use_lidar_gate:=true start_lidar_gate_zones:=false` |
| `use_apriltag_fix` | false | tag sightings with a known map pose → `mcl_measurement` |
| `use_tag_triggers` | false | a section's `trigger_label` / `trigger_tag` advances progress to it |
| `start_apriltag` | false | also launch `zed_apriltag` |
| `tag_size` | 0.16 | printed tag edge length (m, black square), passed to `zed_apriltag` |
| `tag_map_file` | config/tag_map_obstacle_course.yaml | tag map poses for fixes |
| `tag_survey_file` | '' | record tag map poses to this file (survey mode: the supervisor drives nothing) |

A mode switched off falls back to `path_tracker`, then `nav2`.

State: `ros2 topic echo /course_supervisor/state`. Driving line for RViz:
`/course_supervisor/course_path`.

## Editing the course

Edit `config/obstacle_course_template.yaml` (section starts by keyframe index
or map position, mode, lidar, speed, triggers) and regenerate:

```bash
ros2 run course_supervisor make_course_file \
  --poses src/diy_localization/map/Obstacle_course/poses.txt \
  --template src/course_supervisor/config/obstacle_course_template.yaml \
  --output   src/course_supervisor/config/obstacle_course.yaml
```

The driving line and all coordinates must be in the same map frame as the
`mcl_map.pcd` that mcl_3dl loads.

## AprilTags

Each sighting of a tag with a known map pose is an absolute fix: position from
the tag's measured depth and sideways offset, heading from the tag only if it
agrees with the current heading within 8° (guards against the PnP pose flip).
The fix goes to mcl_3dl as a measurement (`/mcl_measurement`), which corrects
map→odom; odometry stays smooth. It works inside lidar-gated sections too.

**Detection range: 2.0 m.** Tags are planned for `zed_apriltag` reporting tags
up to 2 m away (`tag_max_range: 2.0`). If the Jetson copy limits detection to
1 m, raise it to 2 m: in the wide lanes (after the gravel, car wash) a
wall-top tag leaves the camera view before it gets within 1 m. Section switches
still happen close to the tag (all `trigger_range` ≤ 1 m).

Planned placements (`config/tag_plan_obstacle_course.yaml`, already in
`config/tag_map_obstacle_course.yaml`). Each tag stands on top of the wall at
its inner edge, upright, centre ~0.55 m above the floor, printed face turned
back toward the approaching robot:

| ID | Label | Where | Map x, y | Facing |
|---|---|---|---|---|
| 0 | RampDetection | left wall, foot of the 20 % ramp | (3.72, 0.17) | 168° |
| 4 | HELIX_EXIT | right wall, helix bottom before the bridge (fix only) | (7.29, 0.20) | 82° |
| 2 | TUNNEL_EXIT | right wall, 0.4 m into the 20 in lane after the tunnel | (6.99, −5.52) | 88° |
| 1 | NARROW_PATH_END | left wall, just after the gravel (2 in ramp down) | (−0.79, −10.54) | 10° |
| 3 | CAR_WASH_ENTRY | left wall, ~0.8 m into the car wash (no ribbon in front) | (−1.84, 0.78) | 175° |

Labels: `aprilTag/config/tag_labels.yaml`; triggers match the **label text**.

To move a tag: edit the plan (`s` along the driving line, `side`, `height`,
`trigger_range`), then

```bash
ros2 run course_supervisor plan_tags --plan src/course_supervisor/config/tag_plan_obstacle_course.yaml \
  --course src/course_supervisor/config/obstacle_course.yaml --figure /tmp/tag_plan.png
```

It prints where the tag is visible from and warns if a trigger can't fire.
Copy the printed lines into the tag map and the `trigger_range` into the
template, then regenerate the course file.

Refine the poses with a survey after mounting (`docs/apriltag_survey.pdf`):
`start_apriltag:=true tag_survey_file:=/tmp/tags.yaml`, drive with the
joystick, copy the result into the tag map. A pose can also be written in the
label: `ID2: "TUNNEL_EXIT @ 6.99, -5.52, 0.43, 88"` (x, y, [z,] facing_deg).

Requires TF `base_footprint -> <ZED left optical frame>` (else
`camera_offset_xyz/rpy` from `course_supervisor.yaml` is used and logged).

## Testing

Full details: `docs/course_supervisor_guide.pdf` (section 8). Each step adds
one piece; keep the E-stop in hand. Pass arguments to the Jetson script:
`src/challenge_bringup/launch/obstacle_course_supervisor_jetson.sh <args>`.

**0. Offline checks (dev machine)**

```bash
colcon build --packages-select course_supervisor mcl_3dl mapless_wall_follower
cd src/course_supervisor && PYTHONPATH=. python3 -m pytest -q test
```

**A. Without driving autonomously**

| Step | Args | Pass when |
|---|---|---|
| T0 Static | (default, waits for green light) | `/course_supervisor/state` shows `waiting_for_green_light ... pose=ok`; `ros2 topic info /cmd_vel_smoothed` → 1 publisher |
| T1 Dry run | `wait_for_green_light:=false`, drive stack in **manual** (joystick) | drive the course by joystick: sections and modes switch at the places in the table above |
| Tags | T1 + `start_apriltag:=true use_apriltag_fix:=true use_tag_triggers:=true` | log `tag N (...) fix:` near the expected pose and `progress -> <section>` at each tag |

**B. One controller at a time (autonomous)**

| Step | Args | Pass when |
|---|---|---|
| T2 Path tracker | `use_nav2:=false use_wall_follower:=false wait_for_green_light:=false` | follows the line < 0.15 m off at 0.25–0.35 m/s |
| T3 + Wall follower | `use_nav2:=false wait_for_green_light:=false` | narrow path (tunnel exit → gravel) centred, no `RECOVERY_*`, enabled only there |
| T4 + Nav2 | `wait_for_green_light:=false` | MPPI drives start lane and after-gravel → hoops (through the hoops, around the buckets) |
| T5 + Lidar gate | script default | `/mcl_measurement_enabled` false only in ramp/helix/tunnel/car wash; no pose jump after the tunnel |

**C. Full run**

Script default + tag args, `laps:=2`, start with
`ros2 topic pub --once /green_light std_msgs/msg/Bool "{data: true}"`.
Pass: state goes `lap=1/2` → `lap=2/2` → `finished`, stopped at the finish.

**Not tested yet (watch first):** the `/cmd_vel_smoothed` remap on live nodes,
the wall follower on the real lidar (incl. the 50 in gravel), robot height vs
the 0.36 m hoop arch, the lap-2 changeover (unit-tested only), the camera TF
chain and the `zed_apriltag` build on the Jetson.

**Fallback:** `src/challenge_bringup/launch/obstacle_course_jetson.sh` (old
setup, unchanged).

## Notes / tuning

* The wall follower ignores points below `min_z` (body frame), so it can't see
  the 6 in rails on the ramp and helix. Those sections use `path_tracker`.
* `config/mapless_wall_follower_obstacle_course.yaml` lowers
  `min_corridor_width` to 0.45 m for the 20 in section and caps speeds.
* While a non-nav2 section is active, the FollowPath goal is cancelled, so MPPI
  doesn't run (or fail its progress checker) on sections it doesn't drive.
