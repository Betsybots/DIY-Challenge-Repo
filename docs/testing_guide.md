# Testing & Operations Guide

Living reference for every script, launch file, and profile in this repo —
what it does, when to run it, and on which device. **Update this file
whenever you find something wrong or change how a piece is used** — that's
the whole point of it existing.

Cross-reference: [docs/reuse_plan_step1.md](reuse_plan_step1.md) is the
running decision log (why things changed); this file is the how-to-use
reference (how things run today). [docs/pipeline_diagram.png](pipeline_diagram.png)
(source: [pipeline_diagram.dot](pipeline_diagram.dot)) is a visual snapshot
of what's currently connected and how far the pipeline has actually been
tested — regenerate it (`dot -Tpng pipeline_diagram.dot -o pipeline_diagram.png`)
whenever the tested/broken status of a subsystem changes.
[docs/Jetson_Bringup_Guide.pdf](Jetson_Bringup_Guide.pdf) (source markdown:
[jetson_bringup_guide.md](jetson_bringup_guide.md), regenerate with
`python3 docs/generate_jetson_bringup_guide.py` after editing) is a focused,
step-by-step walkthrough for a fresh clone on the Jetson specifically —
use that first if you're setting up a new device; use this file once
you're up and running and need the full script/flag reference.
[docs/Custom_Nav_Stack_Design.pdf](Custom_Nav_Stack_Design.pdf) (source
markdown: [custom_nav_stack_design.md](custom_nav_stack_design.md),
regenerate with `python3 docs/generate_custom_nav_stack_design.py` after
editing) covers the custom A*/PD nav stack and why `diy_zone_nav` is
already safely decoupled from it — see caveat #9 below for the summary.

---

## 0. Known caveats to read before you start

These affect how you should interpret several sections below — read this
first so you don't get tripped up.

1. **`test_step1`–`test_step4` scripts use the OLD legacy motor package**
   (`diy_motor_control_legacy`, `ros2 run diy_motor_control_legacy
   diff-drive-main`), **not** `driveStack`'s tested fork that's actually
   confirmed working on the robot. The legacy package has different wheel
   geometry and a different CAN bus number (`can1` vs `driveStack`'s
   `can0`) — see §4 for what this means before you run any of them.
   `test_step2`/`test_step4` now use the merged `ekf_odom.yaml` (see
   caveat #7) but still remap its `odom0`/`imu0` topics to match the legacy
   package's naming, not driveStack's.
2. **ZED2i camera driver source is now CONFIRMED**: `stereolabs/zed-ros2-wrapper`
   (https://github.com/stereolabs/zed-ros2-wrapper) is the actual driver in
   use on the real robot — `use_zed:=true` in `challenge_master.launch.py`
   can now be trusted as targeting the right package/launch file/topics.
   Still open: `scripts/calibrate_cam_lidar.sh`/`calibrate_camera_intrinsics.sh`
   remain RealSense-specific (not updated for ZED2i) — ZED2i normally uses
   Stereolabs' own SDK calibration tools rather than the ROS
   `camera_calibration` checkerboard flow these scripts currently wrap;
   needs a team decision on approach, not just a mechanical rename.
3. **`hesai_ros_driver`'s exact output topic is unconfirmed** — `/hesai/points`
   (assumed by `challenge_master.launch.py`) vs `/lidar_points` (what
   `fast_lio_ros2` has actually been tested against). Verify on the Jetson
   with `ros2 topic list` before trusting the lidar leg of any full-stack run.
4. **FAST-LIO2's `extrinsic_R` fixed, but the new value needs on-robot
   re-verification.** The old determinant -1 matrix (invalid rotation) was
   replaced with a real, teammate-committed value pulled from
   `LIO_Localization` branch `localizer_v2.2` — now a valid rotation
   (`check_config.py` PASSes, det=1.0). However, the new value is plain
   identity, which does NOT match this file's own in-comment Rx(-90°)
   derivation from a live accelerometer/physical-mounting analysis — see
   the inline note above `extrinsic_R:` in `fast_lio_hesai_qt64.yaml`.
   Re-verify with a live accelerometer reading before fully trusting this
   for autonomous nav. Run `python3 src/fast_lio_ros2/tools/check_config.py
   --config src/diy_localization/config/fast_lio_hesai_qt64.yaml` after any
   change here.
5. **Two devices, two roles — do not run the same flags on both.** The RPi
   owns `cmd_vel_mux` + motor driver + the IMU (see caveat #7); the Jetson
   owns lidar/camera/localization/Nav2. Always launch via
   `scripts/run_robot.sh <profile>` (which sources the right profile
   automatically) rather than calling `challenge_master.launch.py` directly
   with hand-picked flags — see §2.
6. **`driveStack` is a separate, manual-only repo/bringup** — not part of
   `challenge_master.launch.py` at all. Use it for manual/teleop driving
   only (§7), never mixed with the autonomy stack in the same run.
7. **Localization is now a single EKF — the old two-EKF cascade
   double-counted the IMU, confirmed only by actually running the nodes,
   not by inspection.** The old two-stage EKF1(`ekf_wimu.yaml`)→EKF2
   (`ekf_local.yaml`) cascade double-counted the IMU (FAST-LIO2 already
   fuses it internally) and has been collapsed into one EKF
   (`ekf_odom.yaml`: wheel velocity + IMU angular-rate + gated lidar
   odometry → `/odometry/filtered`). Separately, all three EKF configs had a
   real pre-existing bug — mixed `0`/`0.05` types in covariance matrices —
   that made `ekf_node` refuse to start at all (`ros2 run
   robot_localization ekf_node --params-file ...` failed with `Sequence
   should be of same type`); fixed by normalizing all bare `0` → `0.0`
   inside the covariance blocks. **The ACEINNA IMU driver
   (`imu_can_interface`) is launched independently on the RPi, outside this
   repo entirely** — it is not vendored here and `challenge_master.launch.py`
   has no responsibility for starting it. FAST-LIO2 and the localization EKF
   (both on the Jetson) still consume `/imu/data` as an external input over
   the Zenoh bridge. **None of the EKF fixes above have been run on the real
   robot yet** — verified in this sandbox only via `ros2 run`/`ros2 node
   info`/`colcon build` (no CAN/lidar/camera hardware here). Confirm
   on-robot: `/imu/data` is visible on the Jetson over the Zenoh bridge, and
   `ros2 node info /ekf_filter_node_odom` shows all three sensor
   subscriptions actually connected (not just declared).
8. **`diy_ndt_localization` (NDT-OMP) is DEPRECATED — replaced by
   `map_localizer` (VGICP) for map→odom TF.** Found a real bug reading the
   NDT-OMP source: its `publishTF()` broadcast the raw scan-matched
   map→base_link pose directly as "map→odom" TF, with no
   `tf2_ros::Buffer`/`TransformListener` anywhere in the class to compose
   against the actual odom→base_link transform — since odom→base_link is
   ALSO published separately by the EKF, this would have double-applied
   the odom offset the moment it actually ran (it also could never start
   anyway — hardcoded `GlobalMap.pcd` path that doesn't exist in this
   repo). Vendored `map_localizer` + `slam_interfaces` + `fast_gicp` (the
   VGICP dependency, added as a real git submodule at
   `third_party_ws/src/fast_gicp`) from `LIO_Localization` branch
   `localizer_v2.2`; its TF math was checked term-by-term and correctly
   composes `map→odom = map→body × inverse(odom→body)`.
   **`map_localizer` does NOT auto-load a map at startup** — unlike
   NDT-OMP, it only loads a `.pcd` (and sets the initial pose guess) in
   response to a `/relocalize` service call. `localization.launch.py` now
   also launches `trigger_map_relocalize.py`, which makes that call once
   at startup (waiting for the service first) using the `map_pcd_path`
   launch argument (defaults to
   `$DIY_ROS_WS/src/DIY-Challenge-Repo/maps/refined_map.pcd`). If that
   script is ever removed from the launch file, `map_localizer` will run
   but silently never produce any output — no error.
   **Verified so far:** full workspace rebuild (14 packages, `fast_gicp`
   CPU-only/no CUDA needed), a real end-to-end launch
   (`fastlio_mapping` + `ekf_node` + `map_localizer_node` +
   `trigger_map_relocalize.py` all start cleanly, `/relocalize succeeded`
   against the real `maps/refined_map.pcd`), and a synthetic-data test
   (manually publishing fake `/cloud_registered_body` +
   `/lidar_odometry` messages) confirming `map→odom` TF is actually
   broadcast under the correct frame names. **NOT yet verified:** real
   VGICP alignment quality against actual lidar scans (impossible without
   real hardware in this sandbox — the synthetic test's trivial cloud
   couldn't produce a meaningful match, so the TF held at the default
   identity/zero fallback, which is itself expected/correct behavior for a
   failed alignment, not a bug). Whether `maps/refined_map.pcd` is even
   the real course map (vs. a bench/test map) is unconfirmed — see
   `docs/reuse_plan_step1.md` Step 11.
   **Also fixed while investigating this:** a `third_party_ws/COLCON_IGNORE`
   file (reintroduced by a teammate's commit during a `git pull --rebase`,
   re-triggering a bug already found and fixed earlier this session) was
   silently blocking `colcon list`/`build` from seeing anything in
   `third_party_ws` at all — removed again; always verify with
   `cd third_party_ws && colcon list` if third-party packages ever seem to
   vanish.
9. **The custom A* + PD/pure-pursuit controller (`diy_planning`,
   `diy_motion_planner`) is a SEPARATE navigation stack from Nav2 — it
   uses only `nav2_map_server`/`nav2_lifecycle_manager` for map serving,
   not the full Nav2 navigation system.** It publishes `/cmd_vel_nav`
   (same topic `cmd_vel_mux` already reads for AUTONOMOUS mode — no mux
   changes needed) and has **zero dependency on `diy_zone_nav`**
   (confirmed by grepping the actual source — no reference to
   `/nav_mode`/`/speed_limit` anywhere). `diy_zone_nav` can be run or not
   run with this stack with zero behavior difference either way — its
   Nav2-costmap/speed-limit service calls already fail gracefully
   (`service_is_ready()` checked) since those services don't exist here.
   **Fixed while confirming this:** `pd_navigation.launch.py`/
   `pure_pursuit_navigation.launch.py`'s `cmd_vel_topic` argument used to
   default to `/cmd_vel` (sim value) despite `base_frame`/`use_sim_time`
   already defaulting to hardware values — a bare hardware launch with no
   override would silently never move the robot. Now defaults to
   `/cmd_vel_nav`. Also added a `/pd/goal_reached` (`std_msgs/Bool`)
   publisher to both controller nodes (previously only a log line, no ROS
   signal at all) so an external sequencer can react to goal completion.
   **New, optional package `diy_waypoint_sequencer`** auto-publishes
   `/goal_pose` in sequence for competition runs — deliberately its own
   separate package (not inside `diy_zone_nav`) so removing it doesn't
   entangle with the zone_nav question at all; if not run, `/goal_pose`
   is simply set manually instead (RViz "2D Goal Pose"). See
   `docs/reuse_plan_step1.md` Step 13 for the full investigation and a
   real functional test (7 passing behavioral assertions) of the new node.

---

## 1. First-time setup

| Step | Command | Where |
|---|---|---|
| Clone + submodules | `git clone --recurse-submodules <repo-url>` | new machine |
| One-command setup | `bash setup.sh [jetson\|raspi\|laptop]` | Jetson, RPi, or laptop |
| Manual rebuild (first-party only) | `colcon build --symlink-install --base-paths src` | any device — **always** pass `--base-paths src`, see caveat below |
| Manual rebuild (third-party) | `cd third_party_ws && colcon build --executor sequential --parallel-workers 1 --symlink-install` | any device |

**Why `--base-paths src` matters:** without it, a plain `colcon build` from
the repo root also discovers packages under `third_party_ws/src/` and can
silently build a second, unpatched copy of a third-party package (e.g.
`ndt_omp_ros2`) straight into the top-level `install/` — shadowing the
correctly-patched one built via `third_party_ws`'s own build step, causing
confusing link failures that look unrelated to the actual cause. `setup.sh`
already does this correctly; only matters if you run `colcon build` by hand.

---

## 2. Running the robot (both devices)

**`profiles/*.env`** — one file per device, sourced (not executed) to set
every `DIY_*` environment variable the launch file reads:

| Profile | Device | Role |
|---|---|---|
| `profiles/jetson.env` | Jetson | Sensors (lidar, ZED2i), localization (FAST-LIO2/EKF/map_localizer), Nav2, zone_nav |
| `profiles/raspi.env` | Raspberry Pi | `cmd_vel_mux` (single-owner across the robot), motor driver (CAN), joystick input relay |
| `profiles/laptop.env` | Laptop | Single-machine debug/replay — owns everything locally, no bridge |

**`scripts/env.sh [profile]`** — sources a profile (auto-detects device by
hostname if no argument given) plus the ROS 2 + third-party overlays, in the
correct order. Source this, don't execute it:
```bash
source scripts/env.sh jetson
```

**`scripts/run_robot.sh [profile]`** — sources the profile via `env.sh`,
then launches `challenge_bringup challenge_master.launch.py` with every
`DIY_*` var mapped to its matching launch argument. This is the **normal way
to start the robot** — don't call `ros2 launch challenge_bringup
challenge_master.launch.py` by hand unless you're deliberately overriding a
specific flag for debugging.

```bash
# On the Jetson:
scripts/run_robot.sh jetson

# On the RPi:
scripts/run_robot.sh raspi
```

**Zenoh bridge:** both devices' ROS graphs are stitched together over a
Zenoh bridge that lives **outside this repo** — kill and relaunch it on
*both* devices before every run if either device was restarted. Not scripted
here by design (handled by the team separately).

**`src/challenge_bringup/launch/challenge_master.launch.py`** — the actual
launch file `run_robot.sh` wraps. See its own module docstring for the full
block-by-block breakdown and every launch argument. Key ones to know:

| Argument | Default | Notes |
|---|---|---|
| `use_cmd_vel_mux` | `true` | **Single-owner** — `true` on RPi only, `false` on Jetson. See caveat #5. |
| `use_micro_ros` | `false` | This robot has no STM32 — leave `false` on every device. Real e-stop is an RJ45 break-loop wired directly into motor power. |
| `use_zed` | `true` | See caveat #2. |
| `use_hesai` | `true` | See caveat #3. |
| `use_motor_driver` | profile-dependent | `true` on RPi, `false` on Jetson/laptop. |
| `mux_mode` | `AUTONOMOUS` | `raspi.env` overrides to `JOYSTICK` at startup for safety — switch with `scripts/set_mux_mode.sh`. |

---

## 3. Pre-flight / health checks

Run before every test session, and again right before a competition run.

| Script | What it checks | Usage |
|---|---|---|
| `scripts/health_check.sh [profile]` | ROS env, required nodes alive, topic rates (lidar/IMU/GPS), TF tree, e-stop state, Nav2 lifecycle | `scripts/health_check.sh jetson` |
| `scripts/health_check_zone_nav.sh` | Zone-nav specific topics/services/mode values | `scripts/health_check_zone_nav.sh` |
| `scripts/inspect_zone_nav.sh` | Live 1 Hz dashboard of zone-nav state (not pass/fail — just a monitor) | `scripts/inspect_zone_nav.sh` |

`health_check.sh` correctly skips checks for whatever's disabled in your
current profile (e.g. it won't fail on a missing `/cmd_vel_mux_node` on the
Jetson, since `DIY_USE_CMD_VEL_MUX=false` there is expected).

---

## 4. Incremental subsystem tests (`test_step1`–`test_step4`)

Bench-test scripts that bring up one slice of the stack at a time, standalone
(not via `challenge_master.launch.py`) — designed for isolated, single-machine
testing during development, each addable with `--viz rviz|foxglove`.

**⚠️ Read caveat #1 above first** — all four currently use
`diy_motor_control_legacy`, not `driveStack`'s tested fork. Confirm the CAN
bus number (`can1` in the legacy code) actually matches your physical wiring
before running any of these on real hardware, or update the scripts to use
`driveStack`'s package instead if that's what's actually wired up.

| Script | Brings up | Verify |
|---|---|---|
| `scripts/test_step1_wheel_odom.sh [--viz ...] [profile]` | Motor node + mux + joystick | `/wheel_cmd_vel`, `/joint_states` @ ~50 Hz |
| `scripts/test_step2_fused_odom.sh [--viz ...] [profile]` | + EKF fusing wheel+IMU (`ekf_odom.yaml`; requires the independent RPi IMU driver already running for imu0) | `/odometry/filtered`, `odom→base_link` TF |
| `scripts/test_step3_fastlio.sh [--viz ...] [profile]` | + Hesai lidar + FAST-LIO2 + EKF + map_localizer (`localization.launch.py mode:=runtime` — no FAST-LIO2-only mode) | `/lidar_odometry` @ ~10 Hz |
| `scripts/test_step4_motion_plan.sh [--viz ...] [fastlio\|fused] [profile]` | + `plan_b` motion executor (forward→90°turn→forward demo). `fastlio`/`fused` only selects which already-running pose topic feeds it. | `/cmd_vel_mux_node` → `AUTONOMOUS`, `/cmd_vel_safe` flowing |

---

## 5. Debug / replay (no hardware needed)

| Script | Purpose | Usage |
|---|---|---|
| `scripts/debug_robot.sh [profile] [node_name]` | Single-node debug launch, or full stack with RViz + DEBUG logging | `scripts/debug_robot.sh laptop` |
| `scripts/replay_bag.sh <bag_path> [profile]` | Replays a recorded bag through the full software stack (no hardware drivers) with `--clock` | `scripts/replay_bag.sh ~/bags/run1 laptop` |

Both use `profiles/laptop.env`-style single-machine ownership
(`use_cmd_vel_mux:=true`) since there's no second device to conflict with.

---

## 6. Recording

| Script | Records | Usage |
|---|---|---|
| `scripts/record_bag.sh [profile] [--label <tag>] [--topics ...]` | All SAD-required topics (lidar, IMU, GPS, camera, TF, cmd_vel chain) | `scripts/record_bag.sh jetson --label course_run1` |
| `scripts/record_zone_nav.sh [--label <tag>]` | Lightweight zone-nav-only topics (no lidar/camera — small bags) | `scripts/record_zone_nav.sh --label test1` |
| `scripts/record_trajectory.py [--out path] [--duration s]` | `/odom` → CSV + annotated trajectory plot (Ctrl+C to stop/save) | `python3 scripts/record_trajectory.py --duration 60` |
| `scripts/zone_nav_logger.py` | Standalone CSV logger of every zone-nav topic, for PlotJuggler/pandas | `python3 scripts/zone_nav_logger.py` |

---

## 7. Manual / teleop driving (separate from autonomy)

`driveStack` (a **different repo**, not part of `DIY-Challenge-Repo`) — a
tested, working manual-drive bringup: joystick → differential-drive kinematics
→ motors. Use for manual driving/mapping runs; never mix with
`challenge_master.launch.py` in the same session (see caveat #6).

---

## 8. Calibration

| Script | Calibrates | Status |
|---|---|---|
| `scripts/calibrate_imu.sh [profile] [--duration s]` | IMU noise (Allan variance) via `imu_utils` — 2-3 hr stationary recording | Output → paste into `fast_lio_hesai_qt64.yaml` |
| `scripts/calibrate_extrinsics.sh [profile] [--duration s]` | Lidar↔IMU rotation (`extrinsic_R`) via `lidar_imu_calib`, figure-8 motion | Fixed this session (was missing a launch file, hardcoded topics, no result-file output) — see `docs/reuse_plan_step1.md` |
| `scripts/verify_imu_gyr_unit.py [--topic /imu/data]` | On-robot spin test to resolve deg/s vs rad/s ambiguity | Already resolved for the current IMU — kept for future re-verification |
| `scripts/calibrate_cam_lidar.sh` | Camera↔lidar extrinsic (records a bag for offline Kalibr processing) | **Not updated for ZED2i** — still RealSense-specific, see caveat #2 |
| `scripts/calibrate_camera_intrinsics.sh` | Camera intrinsics via ROS `camera_calibration` checkerboard flow | **Not updated for ZED2i** — also the *approach* may need to change (ZED SDK has its own calibration tools) |

---

## 9. Maps & zones

| Script | Purpose |
|---|---|
| `scripts/generate_course_pgm.py [--diagram path] [--output prefix]` | Generates a Nav2 map + 10 zone waypoints from the official course diagram image |
| `scripts/preprocess_map.py --input <pgm> [--preview-only]` | Cleans/smooths a real LiDAR-scanned PGM map for Nav2 (removes noise blobs, smooths walls) |
| `scripts/register_zones_to_map.py --target-map <yaml> [--pick]` | Re-projects the diagram-derived zone waypoints onto a real, LIO-SAM-generated map (once available from an arena practice pass — expected ~1 week before competition) |

---

## 10. Runtime utilities

| Script | Purpose | Usage |
|---|---|---|
| `scripts/set_mux_mode.sh JOYSTICK\|AUTONOMOUS\|BLIND_DRIVE` | Switch `cmd_vel_mux` mode live (`ESTOP_LOCK` is hardware-only, cannot be set manually) | `scripts/set_mux_mode.sh AUTONOMOUS` |
| `scripts/trigger_green_light.sh` | Publishes a one-shot `/green_light` signal (bench-test substitute for the physical start signal) | `scripts/trigger_green_light.sh` |
| `scripts/deploy_bundle.sh <host> [--user u] [--dry-run]` | rsyncs calibration/config/map files from laptop → robot over SSH | `scripts/deploy_bundle.sh jetson.local` |

---

## 11. Simulation (Gazebo, no real hardware)

| Launch file | Purpose |
|---|---|
| `ros2 launch diy_sim sim_competition.launch.py [world:=...] [gui:=false]` | Full competition-arena Gazebo world + robot spawn |
| `ros2 launch diy_sim sim_simple_loop.launch.py` | All-in-one: Gazebo (simple loop track) + Nav2 + circuit_runner — the standard nav-pipeline regression test |
| `ros2 launch challenge_bringup sim_nav.launch.py` | Nav2 + zone_nav + circuit_runner against an *already-running* sim (run `sim_competition.launch.py` first). **Never use on the real robot.** |

Typical flow: launch `sim_competition.launch.py`, then in a second terminal
`sim_nav.launch.py`, then fire the start signal:
```bash
ros2 topic pub --once /green_light std_msgs/msg/Bool "data: true"
```

---

## 12. Launch files not wrapped by any script

These get included by other launch files above — you normally won't invoke
them directly, but useful to know they exist:

| File | Included by | Purpose |
|---|---|---|
| `src/diy_robot_description/launch/description.launch.py` | `challenge_master.launch.py` (BLOCK 2, always) | `robot_state_publisher` from the URDF — runs independently on both Jetson and RPi (safe to duplicate, see §2) |
| `src/diy_localization/launch/localization.launch.py` | `challenge_master.launch.py` (BLOCK 8), `test_step3`/`test_step4` | FAST-LIO2 (`fast_lio_ros2`) + single EKF (`ekf_odom.yaml`) + `map_localizer` (VGICP) — `mode:=runtime` always launches all three (plus a one-shot `trigger_map_relocalize.py` to load the map), there is no FAST-LIO2-only mode |
| `src/diy_localization/launch/offline_mapping.launch.py` | run directly, standalone | LIO-SAM offline prior-map generation from a recorded bag |
| `src/diy_ndt_localization/launch/ndt_localization.launch.py` | **not launched by anything anymore** — DEPRECATED | NDT-OMP map→odom scan matching; replaced by `map_localizer` after a real TF-composition bug was found (see caveat #4/reuse_plan_step1.md Step 11) — kept in the repo for reference/rollback only |
| `src/diy_zone_nav/launch/zone_nav.launch.py` | `challenge_master.launch.py` (BLOCK 12, via `_zone_nav_launch`) | Zone-aware nav state machine. **ORPHANED in the custom A*/PD architecture** (see caveat #9/reuse_plan_step1.md Step 13) — its `/speed_limit` and costmap-layer `SetParameters` calls target full-Nav2 services that don't exist in that setup; already fails gracefully (`service_is_ready()` checked), but nothing consumes its outputs either. Safe to not run at all with the custom controller stack. |
| `src/diy_motion_planner/launch/pd_navigation.launch.py` | run directly, standalone (not yet in `challenge_master.launch.py`) | `nav2_map_server` + `nav2_lifecycle_manager` (map serving ONLY, not full Nav2) + `a_star_planner_node` + `pd_motion_planner_node` → `/cmd_vel_nav`. `cmd_vel_topic` now defaults to `/cmd_vel_nav` (hardware) — pass `cmd_vel_topic:=/cmd_vel` for sim. |
| `src/diy_motion_planner/launch/pure_pursuit_navigation.launch.py` | run directly, standalone | Same map_server + A* stack, with `pure_pursuit_motion_planner_node` instead of PD. Same `cmd_vel_topic` default fix applied. |
| `src/diy_waypoint_sequencer/launch/waypoint_sequencer.launch.py` | run directly, standalone, **optional** | NEW — auto-publishes `/goal_pose` in sequence from a waypoints YAML, waits for `/green_light`, advances on `/pd/goal_reached`. Not running this is completely safe — `/goal_pose` just needs to be published manually instead (RViz "2D Goal Pose"). See reuse_plan_step1.md Step 13 for the full design rationale. |
| `src/challenge_bringup/launch/joystick_drive.launch.py` | `challenge_master.launch.py` (BLOCK 9), `test_step1`-`3` | `joy_node` + `teleop_twist_joy` only — **does not itself launch a motor node** |
| `src/challenge_bringup/launch/motion_plan_executor.launch.py` | `test_step4` (indirectly, via `plan_b`) | Standalone `plan_b` executor launch |
| `src/fast_lio_ros2/launch/lio_localizer.launch.py` | not used by this repo | The `fast_lio_ros2` package's own launch file (upstream) — this repo's `diy_localization/launch/localization.launch.py` launches the node directly with its own params instead |

**Not in this repo at all:** the ACEINNA IMU driver (`imu_can_interface`) is launched independently on the RPi as its own separate process, outside `DIY-Challenge-Repo` entirely — no vendored package, no launch file, no `DIY_USE_IMU` flag here. It just needs to publish `/imu/data` so FAST-LIO2/EKF on the Jetson can consume it over the Zenoh bridge.

---

## Maintenance note

This file is meant to drift as you actually run things and find gaps —
update the relevant section (or add a new numbered caveat to §0) as soon as
you discover something here is wrong, rather than waiting to batch edits.
