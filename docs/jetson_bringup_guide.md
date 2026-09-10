# Jetson Bring-Up Guide — Fresh Clone to Running Localization

Step-by-step guide for cloning this repo onto the Jetson Orin Nano and
getting the localization pipeline (FAST-LIO2 + EKF + `map_localizer`)
running for the first time, plus what to check and what actually reaches
the controller. This is a focused walkthrough — for the full script/
launch-file/profile reference, see [testing_guide.md](testing_guide.md);
for *why* things are built the way they are, see
[reuse_plan_step1.md](reuse_plan_step1.md).

---

## 1. What has to already be true before you start

This repo does **not** install these — confirm each one separately:

| Requirement | Why | How to check |
|---|---|---|
| ROS 2 Humble installed | Everything depends on it | `ls /opt/ros/humble/setup.bash` |
| Real Hesai lidar driver (`hesai_ros_driver`) built/installed on the Jetson | `challenge_master.launch.py` launches it by package name — it must already be on the Jetson's ROS overlay, it is not vendored in this repo | `ros2 pkg prefix hesai_ros_driver` |
| Real ZED SDK + `zed_wrapper` (Stereolabs `zed-ros2-wrapper`) installed | Same as above — camera driver isn't vendored here | `ros2 pkg prefix zed_wrapper` |
| ACEINNA IMU driver running **on the RPi** (separately, not this repo) | FAST-LIO2/EKF need `/imu/data`; this repo has no IMU package anymore — it's launched independently on the RPi | Ask whoever owns that setup; confirm `/imu/data` appears once bridged |
| Zenoh bridge running on **both** the Jetson and the RPi | Without it, `/imu/data` and `/wheel_odom` never reach the Jetson, and `/cmd_vel_nav` never reaches the RPi's motor driver | Lives entirely outside this repo — must be manually (re)started on both sides after any restart |
| `libyaml-cpp-dev`, PCL, Eigen3 system packages | `map_localizer`/`fast_gicp` hard-require these at build time | `rosdep install` (part of `setup.sh`) should pull them in via each package's declared `<depend>`s |

**Hardware physically on the Jetson**: Hesai QT64 lidar (UDP, default
`192.168.1.201:2368` — see `profiles/jetson.env`), ZED2i camera (USB3).
**Hardware NOT on the Jetson**: the IMU and the motor/CAN bus — both live
on the RPi.

---

## 2. Clone and build

```bash
# Recommended: clone at exactly this path. profiles/jetson.env's
# DIY_ROS_WS defaults to ${HOME}/ros2_ws, and diy_localization's
# map_pcd_path launch argument defaults to
# $DIY_ROS_WS/src/DIY-Challenge-Repo/maps/refined_map.pcd — if you clone
# somewhere else, you'll need to pass map_pcd_path explicitly every time
# (see §4).
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/Betsybots/DIY-Challenge-Repo.git
cd DIY-Challenge-Repo

# Builds third_party_ws (fast_gicp, ndt_omp_ros2, lidar_imu_calib, LIO-SAM,
# imu_utils_ros2_humble) + the first-party DIY packages, in the right order.
bash setup.sh jetson
```

**If `setup.sh` fails partway through** — a few known, unrelated-to-this-repo
sandbox/system gaps that may or may not apply to your specific Jetson image:
- `code_utils`/`imu_utils` failing on `elfutils/libdw.h` — missing
  `libdw-dev`; only affects IMU Allan-variance calibration tooling, not the
  main pipeline.
- `lio_sam` failing on missing `GTSAM` — only affects offline map
  generation (`offline_mapping.launch.py`), not runtime localization.

Both are safe to ignore if you're not using those specific tools right now.

### Sourcing order matters

```bash
source /opt/ros/humble/setup.bash
source third_party_ws/install/setup.bash   # MUST come before the next line
source install/setup.bash
```
`diy_ndt_localization` (and anything else linking `third_party_ws`
packages like `ndt_omp_ros2`) will fail to even be found by `ros2 launch`/
`colcon build` if `third_party_ws/install/setup.bash` isn't sourced first.
`scripts/env.sh <profile>` does this in the right order automatically —
prefer it over sourcing manually:

```bash
source scripts/env.sh jetson
```

---

## 3. Launching

### Option A — the full stack (normal operation)

```bash
scripts/run_robot.sh jetson
```

This runs `challenge_master.launch.py` with every `DIY_USE_*` flag from
`profiles/jetson.env` mapped to a launch argument — lidar driver, camera,
FAST-LIO2, the single EKF, `map_localizer`, Nav2, zone_nav. Must be run
**alongside** `scripts/run_robot.sh raspi` on the RPi (owns the motor
driver, `cmd_vel_mux`, and the IMU) with the Zenoh bridge up on both sides.
Never run the same profile on both devices — see
[testing_guide.md](testing_guide.md) §2/caveat #5.

### Option B — localization only (recommended for first-time testing)

Isolates FAST-LIO2 + EKF + `map_localizer` without Nav2/zone_nav/motor
control — much easier to debug on a first run:

```bash
ros2 launch diy_localization localization.launch.py \
    mode:=runtime \
    use_rviz:=true \
    map_pcd_path:=/absolute/path/to/your/refined_map.pcd
```

`map_pcd_path` only needs to be passed explicitly if you didn't clone at
`~/ros2_ws/src/DIY-Challenge-Repo` (see §2) or want to use a different map
than `maps/refined_map.pcd`.

> **Note:** this is a separate, unrelated map from the one the custom
> A\*/PD controller stack uses. `map_pcd_path` here is a 3D point cloud
> (`.pcd`) for `map_localizer`'s `map→odom` TF. The controller's map is a
> 2D occupancy grid (`.pgm`/`.yaml`) configured via `map_yaml` /
> `$DIY_MAP_YAML` — see `custom_nav_stack_design.md` §5. A `.pgm`/`.yaml`
> pair cannot be used as a `map_pcd_path` value or vice versa.

**What this starts, and in what order things become available:**
1. `fastlio_mapping` (FAST-LIO2) — needs `/hesai/points` (or `/lidar_points`
   — see the open topic-name question in §5) and `/imu/data` immediately.
2. `ekf_filter_node_odom` (the single EKF) — needs `/wheel_odom` (from the
   RPi, over Zenoh), `/imu/data`, and FAST-LIO2's gated output.
3. `map_localizer_node` — starts immediately but does nothing until step 4.
4. `trigger_map_relocalize.py` — waits for `map_localizer_node`'s
   `/relocalize` service, calls it once with `map_pcd_path` + the initial
   pose (defaults to map origin), then **exits** (by design — it's not a
   long-running node). Watch its log line: `/relocalize succeeded:
   relocalize success` confirms the map loaded. If you instead see it hang
   on "Waiting up to 30s for /relocalize service", `map_localizer_node`
   isn't up yet or crashed — check its own log for the real error.

---

## 4. What actually reaches the controller

Regardless of whether the controller is Nav2 or a custom one — there is
**no controller-specific wiring on the localization side**. Two things are
published, and any controller consumes them the standard ROS 2 way:

**1. TF tree: `map → odom → base_link`**
- `map → odom`: broadcast by `map_localizer_node` (only after step 4 above
  succeeds — before that, this transform doesn't exist at all).
- `odom → base_link`: broadcast by the EKF (`ekf_filter_node_odom`).
- **There is no `/pose` or `/ndt_pose`-style topic for global position** —
  `map_localizer` only publishes TF + a debug `/map_cloud` + its two
  services. A controller that needs the robot's pose *in the map frame*
  must do a standard TF lookup:

```cpp
// C++ (rclcpp) — same pattern Nav2 itself uses internally
geometry_msgs::msg::TransformStamped map_to_base =
    tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
```
```python
# Python (rclpy)
from tf2_ros import Buffer, TransformListener
tf_buffer = Buffer()
TransformListener(tf_buffer, node)
map_to_base = tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
```

**2. `/odometry/filtered`** (`nav_msgs/Odometry`, `odom` frame, 50 Hz) —
published by the EKF. Most controllers subscribe to this directly for
local velocity/pose feedback (smooth, continuous, no TF-tree traversal
needed) rather than doing a TF lookup for every control-loop tick.

**Which one your teammate's controller should use** depends on what it
needs: local smooth feedback for closed-loop velocity control →
`/odometry/filtered`; global position for waypoint/path following against
the map → the `map→base_link` TF lookup.

---

## 5. Verifying it's actually working

```bash
# All the topics you'd expect, and their rates
ros2 topic hz /hesai/points          # or /lidar_points — see open item below
ros2 topic hz /imu/data              # from the RPi, over Zenoh
ros2 topic hz /lidar_odometry        # FAST-LIO2 output
ros2 topic hz /odometry/filtered     # EKF output, should be ~50 Hz

# Confirm the map actually loaded
ros2 service call /relocalize_check slam_interfaces/srv/IsValid "{code: 1}"

# Confirm the full TF chain resolves
ros2 run tf2_ros tf2_echo map base_link

# Full pre-flight checklist (nodes, topic rates, TF, e-stop state, Nav2 lifecycle)
scripts/health_check.sh jetson
```

If `map→base_link` never resolves: check `ros2 node list` for
`map_localizer_node` and `ekf_filter_node_odom` both present, then check
`ros2 topic list` for `/lidar_odometry` and `/cloud_registered_body`
actually publishing (both required before `map_localizer`'s synced
callback ever fires — see §3, step 4).

---

## 6. Known open items — check these before trusting results

These affect the localization pipeline specifically and are not yet
resolved as of this doc — see `reuse_plan_step1.md` for full detail on
each:

- **`hesai_ros_driver`'s real output topic is unconfirmed**: `/hesai/points`
  (assumed by `challenge_master.launch.py`) vs `/lidar_points` (what
  `fast_lio_ros2` has actually been tested against). Check with
  `ros2 topic list` on your specific Jetson before trusting the lidar leg.
- **`extrinsic_R` was just fixed to a valid rotation (identity) but not
  yet re-verified on hardware** — the new value doesn't match this
  repo's own from-scratch Rx(-90°) derivation from an earlier
  accelerometer test. Re-verify with a live accelerometer reading if
  results look physically wrong (e.g. yaw/roll swapped).
- **Whether `maps/refined_map.pcd` is genuinely this course's map** (vs. a
  bench/test map) is unconfirmed. If localization looks completely wrong
  from the start, this is the first thing to check.
- **VGICP alignment accuracy has only been verified with synthetic data in
  a sandbox** — never against real lidar scans. Watch `map_localizer`'s
  own log output and `/map_cloud` in RViz for obviously bad alignment.
- **`base_link→camera_link` has NO publisher at all right now.** The URDF
  (`robot.urdf.xacro`) doesn't define `camera_link` (only `lidar_link`/
  `imu_link` are real — verified by actually running `xacro` and checking
  the generated output), and `zed_wrapper`'s own `publish_tf` was
  deliberately disabled in `challenge_master.launch.py` based on the
  (false) assumption that the URDF already covers it. If anything needs
  to project ZED2i image/depth data into the robot frame via TF, it will
  fail until this is fixed either by adding `camera_link` to the URDF or
  re-enabling `zed_wrapper`'s own TF publishing.
- **This robot has no GPS hardware at all** (confirmed) — `DIY_USE_GPS`
  is `false` on every profile; no `/gps/fix` topic will ever appear, by
  design, not because anything is broken.

---

## 7. Quick reference — commands used most often

```bash
# One-time setup
git clone --recurse-submodules <repo-url> ~/ros2_ws/src/DIY-Challenge-Repo
cd ~/ros2_ws/src/DIY-Challenge-Repo
bash setup.sh jetson

# Every new terminal
source scripts/env.sh jetson

# Test localization in isolation
ros2 launch diy_localization localization.launch.py mode:=runtime use_rviz:=true

# Full stack (after localization checks out)
scripts/run_robot.sh jetson

# Pre-flight check
scripts/health_check.sh jetson
```
