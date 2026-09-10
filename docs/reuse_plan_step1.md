# Reuse Plan Step 1 (From Betsybot-Software)

This document maps reusable legacy code to the 2026 SAD in [docs/diy-sad.html](docs/diy-sad.html) and defines the first migration baseline in this repository.

## What We Reuse Now

1. Motor/CAN control package from last year:
- Source: /home/vg1617/ros2_ws/src/Betsybot-Software/src/differential-drive
- Imported into this repo as: src/diy_motor_control_legacy
- Key behavior: subscribes `/cmd_vel`, drives TalonFX on `can1`, publishes wheel odom and joint states.

2. Joystick control launch pattern:
- Legacy reference: /home/vg1617/ros2_ws/src/Betsybot-Software/src/master_launch/launch/Joystick_drive.launch.py
- Implemented here in: src/challenge_bringup/launch/joystick_drive.launch.py
- Update: teleop output remapped to `/cmd_vel_joy` (SAD requires explicit mode gating).

3. Nav2 bringup/config skeleton:
- Legacy references:
  - /home/vg1617/ros2_ws/src/Betsybot-Software/src/betsybot/launch/bringup_launch.py
  - /home/vg1617/ros2_ws/src/Betsybot-Software/src/betsybot/config/nav2_params.yaml
- Implemented here in:
  - src/challenge_bringup/launch/challenge_master.launch.py
  - src/challenge_bringup/config/nav2_params.yaml
- SAD alignment updates:
  - Costmap observation source moved to `/hesai/points`
  - Planner set to Smac Hybrid
  - Controller set to MPPI

4. Collision monitor baseline:
- Legacy reference: /home/vg1617/ros2_ws/src/Betsybot-Software/src/betsybot/config/collision_monitor_params.yaml
- Implemented here in: src/challenge_bringup/config/collision_monitor_params.yaml
- Update: uses `/hesai/points`, outputs `/cmd_vel_safe`.

## What Exists In Legacy But Not Yet Reused

1. Hesai driver source tree exists in old repo and is already buildable via previous setup.
2. RealSense package exists in old repo but is usually better consumed from apt-installed `realsense2_ros`.
3. SuperOdom exists but SAD requires FAST-LIO2 for runtime odometry and LIO-SAM for offline mapping.

## Critical SAD Gaps (Must Build New)

1. E-stop path and fail-safe watchdog (STM32-centered, hardware priority).
2. Command arbitration node (joystick vs autonomous vs estop), producing `/cmd_vel_safe`.
3. Mission state machine and zone behavior switcher.
4. PCL obstacle classifier + map differencing.
5. EKF1/EKF2 configuration and navsat_transform wiring.

## First-Step Development Baseline Added Here

1. Reused motor package:
- src/diy_motor_control_legacy

2. New bringup package:
- src/challenge_bringup/package.xml
- src/challenge_bringup/CMakeLists.txt
- src/challenge_bringup/launch/challenge_master.launch.py
- src/challenge_bringup/launch/joystick_drive.launch.py
- src/challenge_bringup/config/nav2_params.yaml
- src/challenge_bringup/config/collision_monitor_params.yaml
- src/challenge_bringup/maps/static_map.yaml

## Action Plan (SAD-Aligned Next 5 Work Items)

1. Implement `diy_cmd_vel_mux` node:
- Inputs: `/cmd_vel_joy`, `/cmd_vel_nav`, `/estop_active`
- Output: `/cmd_vel_safe`
- Modes: JOYSTICK, AUTONOMOUS, ESTOP_LOCK

2. Integrate `diy_estop_controller` interface:
- Bridge STM32 status to ROS (`/estop_active`, heartbeat timeout)
- Force zero velocity on fail-safe.

3. Bring FAST-LIO2 + EKF online:
- FAST-LIO2 output to `/lidar_odometry`
- EKF1 local odom and EKF2 global map fusion with RTK.

4. Add mission scaffolding package:
- State transitions from SAD: WAIT -> NAVIGATE -> ZONE_ENTRY -> BEHAVIOR -> LAP_COMPLETE.

5. Add obstacle classifier package:
- RANSAC ground removal, clustering, class outputs to `/obstacle_class`.

## Notes

- Imported motor control package is legacy and tightly coupled to TalonFX/Phoenix6 and differential drive assumptions.
- It should be treated as a bootstrap implementation while the final STM32 + micro-ROS safety architecture is introduced.

---

## Reuse Plan Step 2 — FAST-LIO2 + IMU Integration (from teammate repos)

**Note:** parts of the original SAD (`docs/diy-sad.html`) referenced above are now
stale — the design has moved on since it was written (GPS/navsat_transform was
dropped in favor of NDT-OMP for `map→odom`, the controller moved from MPPI to
RPP, and the camera is now a ZED2i rather than the RealSense D435i described
in the SAD). Treat this section, `src/diy_localization/launch/localization.launch.py`'s
docstring, and direct teammate confirmation as more current than the SAD text.

1. FAST-LIO2, Hesai QT64-specific fork:
   - Source: teammate repo `Betsybots/LIO_Localization`, branch `develop`,
     `src/FAST_LIO_Hesai_ROS2` (package `fast_lio_ros2`, executable
     `fastlio_mapping`).
   - **Branch correction (2026-09-04):** this was first vendored from
     `LIO_Localization`'s `main` branch, which turned out to be stale —
     `develop` is the real, actively-tested branch (has a hardware-derived
     IMU extrinsic, ARM/Jetson build fix, a leaner QT64-only config/launch
     set, and a simplified `check_config.py`). Re-vendored from `develop`.
   - Imported into this repo as: `src/fast_lio_ros2`
   - Replaces: the generic `third_party_ws/src/FAST_LIO` submodule (hku-mars/FAST_LIO,
     `ROS2` branch) — that submodule has been removed. The generic build only
     supported `lidar_type: 2` (Velodyne-style parsing); this fork adds a
     dedicated `lidar_type: 3` case that parses the real Hesai QT64
     PointCloud2 layout (x/y/z/intensity/ring/timestamp).
   - Wired in `src/diy_localization/launch/localization.launch.py` (BLOCK 1) and
     `src/diy_localization/config/fast_lio_hesai_qt64.yaml` (synced verbatim
     from `develop`'s `config/qt64.yaml` — see that file's own header comment
     for the full list of open items found during the sync, summarized below).
   - **Real bug found and fixed:** BLOCK 1's `Node(...)` was passing
     `{"config_path": ..., "config_file": ...}` as inline parameter
     overrides — a scheme the old generic `third_party_ws/FAST_LIO` package
     used, but `fast_lio_ros2` does NOT support (it uses plain
     `declare_parameter()` + a normal ROS 2 params YAML, same as its own
     `launch/lio_localizer.launch.py`). Those two parameter names were never
     declared by the node, so they were silently ignored — FAST-LIO2 was
     running on 100% hardcoded defaults (wrong topics, wrong extrinsics)
     regardless of what `fast_lio_hesai_qt64.yaml` said, since the very first
     version of this integration. Fixed by passing the resolved yaml path
     directly in `parameters=[...]`.
   - Also ships `tools/check_config.py` / `check_input.py` / `check_map.py`
     for offline config/topic validation without needing ROS running.
     `develop`'s `check_config.py` is QT64-only (no more `--model`/`--ros`
     flags) — the `qt64` `MODEL_SPECS` entry I'd added to the stale `main`
     version isn't needed anymore.
   - **Open items surfaced by `check_config.py` against the real, synced
     config:**
     1. [RESOLVED 2026-09-04] `common.imu_gyr_unit` was `"deg"` — static
        analysis said this looked backwards (`imu_can_interface` converts
        gyro counts to rad/s before publishing, ~57x drift risk if so). A
        teammate with direct hands-on experience with this IMU confirmed
        the raw data is genuinely rad/s and changed the setting to `"rad"`
        (not from a spin test — see `scripts/verify_imu_gyr_unit.py` if this
        ever needs re-checking after a sensor/firmware change).
     2. `mapping.extrinsic_R` has determinant **-1.0** (a reflection, not a
        valid rotation — `check_config.py` FAILs this). The file's own
        comments derive a different, valid (det +1) matrix from a live
        accelerometer reading just above the active value, but the active
        value matches neither that derivation nor the "old/buggy" value the
        comments call out — a third, seemingly unresolved matrix. **Note:**
        the comment's premise that the IMU driver's own extrinsic rotation
        is "disabled" is now confirmed true (see ACEINNA item below — that
        driver's `develop` branch removed extrinsic handling entirely), so
        the reasoning path is sound; it's specifically the det-(-1) matrix
        itself that's still wrong and needs re-deriving. **Still open.**
     3. [RESOLVED 2026-09-04] `preprocess.scan_line: 32` is intentional —
        confirmed by a teammate as one half of a deliberate H-FOV/V-FOV
        reduction plan (64→32 channels, paired with `mapping.fov_degree`
        360°→180°, also now applied) to shrink point-cloud volume for
        faster/more-robust preprocessing on Jetson. `check_config.py`
        hardcodes `QT64_SPEC = {"scan_line": 64}` and will keep FAILing this
        specific check on every run — that's the checker being stale for
        this robot's chosen config, not a real problem.
     4. `common.lid_topic: "/lidar_points"` — but `challenge_bringup`
        (BLOCK 6) and `profiles/*.env` assume a `hesai_ros_driver` node
        publishing `/hesai/points`. `hesai_ros_driver` itself is not
        missing — per `docs/installation_setup.html` it already lives
        directly on the Jetson as a separate overlay workspace
        (`~/ros2_ws/src/Betsybot-Software/src/HesaiLidar_ROS_2.0`), outside
        all 4 git repos by design. The gap is purely the topic NAME the
        real driver actually publishes on the Jetson — `/hesai/points` or
        `/lidar_points`? Treating `/lidar_points` as ground truth since
        that's what `fast_lio_ros2` has actually been tested against;
        reconcile once the real topic name is confirmed on-robot.
        **Still open.**
     5. `mapping.map_file_path` is an absolute path on the original dev's
        machine — won't exist elsewhere. `check_config.py` WARNs on this.
        **Still open** (low priority — cosmetic/per-machine only).

2. ACEINNA IMU CAN driver:
   - Source: teammate repo `Betsybots/ACEINNA_IMU_ROS2`, branch `develop`
     (also corrected from a stale `main` vendor, same as FAST-LIO2 above),
     `imu_can_interface`.
   - Imported into this repo as: `src/imu_can_interface`
   - Publishes `/imu/data` (filtered) and `/imu/data_raw` — matches
     `fast_lio_ros2`'s `imu_topic` and EKF1's `imu0` input, no topic remap needed.
   - Runs physically on the Raspberry Pi 5's CAN bus alongside motor control;
     `/imu/data` reaches the Jetson over the Zenoh bridge (already configured
     outside this repo) rather than being launched locally by
     `challenge_master.launch.py`.
   - **`develop` vs `main` — two real fixes on the driver side** (commit
     "updated imu_timings and remove the extrinsics"):
     1. **CAN message decode bug fixed:** the angular-rate PGN was being
        parsed as the 19-bit "High-Resolution Angular Rate" message
        (1/1024 °/s per bit), but the CAN ID actually configured
        (`0x0CF02A82`) decodes to PGN 61482 — the STANDARD 16-bit Angular
        Rate message (1/128 °/s per bit) per the MTLT335D manual. Old scale
        `0.000976563` (1/1024) → new scale `0.0078125` (1/128), an ~8x
        correction. The slope-sensor offsets were also corrected
        (-200.0 → -250.0). Before this fix, `/imu/data` angular_velocity
        would have been ~8x too small in magnitude regardless of the
        deg/rad unit question above — worth keeping in mind if any earlier
        FAST-LIO2 tuning was done against the un-fixed driver.
     2. **Extrinsic frame rotation removed entirely** — `develop` deletes
        the `extrinsic:` block from `imu_params.yaml` and all matching code
        from `imu_node.cpp`/`imu_node.hpp`. `/imu/data` and `/imu/data_raw`
        are now unconditionally published in the raw, uncorrected IMU
        sensor frame. This **resolves** the contradiction flagged in item
        #2 above between what the FAST-LIO2 config's comments assumed
        (driver extrinsic "disabled") and what the previously-vendored
        `main` branch actually had (`enabled: true`) — `develop`'s behavior
        now matches the FAST-LIO2 config's assumption.
     3. Also renamed: `launch/imu.launch.py` → `launch/imu_sensors.launch.py`.
        Not referenced by anything in this repo (this driver runs on the
        RPi outside `challenge_master.launch.py`), so no wiring changes
        needed here.
   - See open item #1 above re: `imu_gyr_unit` — now **resolved** (set to
     `"rad"`, confirmed by a teammate's direct hands-on knowledge of this
     IMU). The scale-factor fix above is independent of and doesn't change
     that determination.

3. Not touched in this pass, and not a bug to fix (confirmed with the team):
   `src/diy_motor_control_legacy` and `driveStack`'s
   `differential-drive`/`plan_b`/`wall_follower` packages are intentionally
   separate. `driveStack` (and its newer `differential-drive` node,
   subscribing `/motor_cmd_vel`) is a **manual/teleop-operation stack only**
   -- it is not, and is not meant to be, wired into
   `challenge_master.launch.py`. The autonomy master launch continues to
   drive motors via `src/diy_motor_control_legacy` (subscribing the
   mux-arbitrated `/cmd_vel_safe`, per BLOCK 10). The `/cmd_vel_safe` vs
   `/motor_cmd_vel` topic names differing between the two is expected, not
   an inconsistency -- they're two independent bringups for two independent
   use cases (autonomous run vs. manual driving), not one pipeline.

## Reuse Plan Step 3 — No STM32: real e-stop is an RJ45 break-loop

**CONFIRMED 2026-09-04:** this robot has **no STM32 at all**. The real,
competition-compliant hardware e-stop is an **RJ45 break-loop** wired
directly into the motor power path — a pure hardware fail-safe (loop
open → motors de-energized), entirely independent of software/firmware.
This satisfies competition rule 1.2.7 ("must fail safe" / "must stop within
1 second") more directly than the STM32+micro-ROS design the original code
assumed ever could, since it works even if the entire Jetson/ROS stack
crashes or hangs.

**Bug found and fixed:** `diy_estop_controller` (the ROS-side advisory
mirror of an STM32's e-stop state) was launched **unconditionally** in
`challenge_master.launch.py` (no `condition=` guard at all), while
`micro_ros_agent` (the STM32 serial bridge) defaulted to enabled
(`use_micro_ros: true`). `diy_estop_controller`'s own documented fail-safe
logic treats "`/stm32/heartbeat` never received" as a **permanent** E-stop
condition — only clearable once a heartbeat *does* arrive. On a robot with
no STM32, that heartbeat can never arrive, so this node would permanently
latch `/estop_active = True` from the very first tick, and
`cmd_vel_mux`'s `ESTOP_LOCK` could never be exited — the master launch was
completely unusable end-to-end as shipped.

**Fix applied:**
- `use_micro_ros` default changed `true` → `false` in
  `challenge_master.launch.py` and in `profiles/jetson.env` /
  `profiles/raspi.env`.
- `diy_estop_controller`'s `Node(...)` in `challenge_master.launch.py` is now
  gated on `condition=IfCondition(use_micro_ros)` (previously unconditional)
  — consistent with `micro_ros_agent`'s existing gating.
- Confirmed this is safe, not just "quiet": `cmd_vel_mux_node.py` initializes
  `_estop_active = False` and only ever sets it `True` on an actual received
  `/estop_active` message (see its own source) — so simply never launching
  `diy_estop_controller` does not create a new failure mode, unlike a naive
  fix that might have tried to make the estop controller itself default
  differently.
- `scripts/health_check.sh` updated to only require `/micro_ros_agent`,
  `/estop_controller_node`, and a responding `/estop_active` topic when
  `DIY_USE_MICRO_ROS=true`; otherwise it now prints a warning to manually
  verify the RJ45 break-loop hardware instead of a false `_fail`.
- Updated `challenge_master.launch.py`'s module docstring, `profiles/*.env`
  header comments, and `src/challenge_bringup/README.md` to describe the
  real e-stop/motor-control architecture (RJ45 break-loop + direct CAN via
  `driveStack`'s tested `differential-drive`, no STM32) instead of the
  STM32+micro-ROS design that was never actually built/present.

**Left as-is (not needed for the fix, but noted for later):** the
`diy_estop_controller` package itself, `micro_ros_agent`, and the
`use_micro_ros` plumbing are not deleted — only disabled by default — in
case a real STM32 (or other MCU) safety architecture is added later, per the
team's stated "do the bigger rewrite later if needed" preference.

## Reuse Plan Step 4 — Camera: ZED2i, not RealSense D435i

This robot's actual camera is a **Stereolabs ZED2i**. `challenge_master.launch.py`
previously assumed an Intel RealSense D435i — no downstream node actually
consumed any RealSense-specific topic, but the block would have failed
outright on launch (no `realsense2_camera` driver exists for a ZED2i).

**Fixed:**
- `challenge_master.launch.py` BLOCK 7: replaced the `realsense2_camera`
  `Node(...)` with a conditional include of `zed_wrapper`'s
  `zed_camera.launch.py` (`camera_model:=zed2i`), with `publish_urdf`/
  `publish_tf`/`publish_map_tf` explicitly `false` — verified against the
  real [zed-ros2-wrapper source](https://github.com/stereolabs/zed-ros2-wrapper)
  that all three default `true` and would otherwise start a second,
  competing `robot_state_publisher` + TF broadcaster for frames already
  owned by `diy_robot_description` (static camera frame) and
  FAST-LIO2/EKF2/NDT-OMP (dynamic odom/map transforms).
- **Real bug found while verifying this:** a plain `IncludeLaunchDescription`
  + `IfCondition(use_zed)` still calls `get_package_share_directory('zed_wrapper')`
  unconditionally while `generate_launch_description()` builds its return
  list — `IfCondition` only gates whether the included launch actually
  *runs*, not whether that lookup *executes*. Since `zed_wrapper` only
  exists on the Jetson (not in this repo, not on a dev laptop), this would
  crash the **entire** master launch file with `PackageNotFoundError` on any
  machine without it installed — including `profiles/laptop.env`, which
  sets `DIY_USE_ZED=false` specifically to avoid needing camera hardware.
  Fixed using the same `OpaqueFunction` pattern already established in this
  file for `_zone_nav_launch` — see `_zed_launch()`. Verified with an
  isolated unit test (no ROS launch service, no `zed_wrapper` install
  needed): `generate_launch_description()` builds cleanly either way,
  `use_zed=false` never touches `zed_wrapper` at all, and `use_zed=true`
  reaches the lookup (and only there fails, since `zed_wrapper` isn't
  installed in the test sandbox).
- Renamed `use_realsense`/`DIY_USE_REALSENSE` → `use_zed`/`DIY_USE_ZED`
  throughout (`challenge_master.launch.py`, `profiles/*.env`,
  `scripts/{run_robot,debug_robot,replay_bag}.sh`).
- Topics corrected to the verified default namespace/node name
  (`/zed/zed_node/...`, not `/zed_node/...`) in `scripts/record_bag.sh` and
  this launch file's comments.
- README hardware table entry corrected.

**Left as a separate, larger follow-up (not fixed here):**
1. `scripts/calibrate_cam_lidar.sh` and `scripts/calibrate_camera_intrinsics.sh`
   are still RealSense-specific (hardcoded topics/driver calls); the
   intrinsics script's whole approach may also need to change since ZED2i
   normally uses Stereolabs' own SDK calibration tools, not the ROS
   `camera_calibration` checkerboard flow it currently wraps. Flagged
   in-file, not rewritten — needs a team decision on approach.
2. The currently-active URDF (`src/diy_robot_description/urdf/robot.urdf.xacro`)
   is missing `camera_link`, `lidar_link`, `imu_link`, and `gps_link`
   entirely, despite its own header comment documenting all four as
   required. (The old, no-longer-loaded `robot-old.urdf.xacro` still has a
   real `camera_link` definition — it was dropped when the active xacro was
   rewritten.) Independent of camera brand; needs its own dedicated pass.

## Reuse Plan Step 5 — Multi-machine device ownership (Jetson vs RPi)

`challenge_master.launch.py` is meant to be run on BOTH devices (via
`scripts/run_robot.sh <profile>`, sourcing `profiles/jetson.env` on the
Jetson and `profiles/raspi.env` on the RPi), with their ROS graphs bridged
together over Zenoh (set up outside this repo). Checking every block's
actual gating surfaced two real problems in how "run it on both machines"
was handled:

**Fixed — `cmd_vel_mux_node` had no on/off switch at all** (no `IfCondition`,
launched unconditionally). Running the master launch on both devices would
start two independent instances, both publishing `/cmd_vel_safe` — a real
command race for the motor driver, not just a cosmetic duplicate (unlike
`robot_state_publisher`, which is safe to duplicate since it only broadcasts
static, idempotent TF — deliberately left unconditional on both devices,
since gating it to one side would make the other's entire localization/nav
stack depend on the Zenoh bridge correctly forwarding `/tf_static`, an
external config this repo can't verify).
- Added a new `use_cmd_vel_mux` launch arg (default `true`) and gated
  BLOCK 5's `Node(...)` on it.
- **This robot: the RPi owns `cmd_vel_mux`** (co-located with the motor
  driver and e-stop wiring) — `DIY_USE_CMD_VEL_MUX=true` in
  `profiles/raspi.env`, `false` in `profiles/jetson.env`. `laptop.env` and
  `scripts/replay_bag.sh` set it `true` (single-machine, no bridge, no
  conflict).

**Fixed — `DIY_USE_MOTOR_DRIVER=true` was set on both `jetson.env` and
`raspi.env`.** The CAN bus + motor driver hardware physically lives on the
RPi only. Set to `false` on `jetson.env`.

**Left open — differential-drive package identity:** BLOCK 10's
`Node(package='differential-drive', ...)` is ambiguous by name alone —
`src/diy_motor_control_legacy` (in this repo) and `driveStack`'s
`differential-drive` (diverged fork: different wheel geometry, `can0` vs
`can1`, added safety watchdog) both declare the identical package name
`differential-drive`. Which one actually launches depends entirely on which
is built into the RPi's active workspace overlay — not verified in this
pass.

## Reuse Plan Step 6 — EKF1+EKF2 → single EKF collapse; two independent
## pre-existing bugs found only by actually running the config; missing IMU wiring

User's proposal: since FAST-LIO2 already runs its own internal EKF fusing
`/imu/data`, the old two-stage `EKF1(ekf_wimu.yaml: wheel+IMU → /wimu_odom)
→ EKF2(ekf_local.yaml: /wimu_odom+lidar → /odometry/filtered)` cascade
double-counts the same physical IMU (two independently-configured
`robot_localization` EKFs both assuming conditionally-independent inputs).
Agreed this is a real anti-pattern. User chose a 3-input design over dropping
IMU entirely: keep direct IMU angular-rate fusion (not full 6-DOF) so the
single EKF's own predict step stays accurate on wheel-slip terrain
(gravel/pothole/bumps) between FAST-LIO2's ~10 Hz lidar corrections.

**Change:** new `src/diy_localization/config/ekf_odom.yaml` — one
`ekf_filter_node_odom`, `odom0=/wheel_odom` (velocity only), `imu0=/imu/data`
(angular-rate fields only), `odom1=/lidar_odometry_gated` (position+yaw+
velocity), 50 Hz output, keeping EKF2's proven output-stage tuning.
`localization.launch.py` BLOCK 2 now launches this single node instead of
two. `ekf_wimu.yaml`/`ekf_local.yaml` marked SUPERSEDED but kept (still used
by `test_step2`/`test_step4` bench scripts — see below).

**Bug found #1 (real, pre-existing, affects ALL THREE ekf configs, not just
the new one) — mixed int/float YAML in covariance matrices crashes
`ekf_node` at startup.** ROS 2's strict params-file YAML parser rejects a
sequence containing both bare `0` (int) and `0.05` (float) — e.g.
`[0.05, 0, 0, ...]` — with `Sequence should be of same type. Value type
'integer' do not belong`. Python's `yaml.safe_load` parses this fine, which
is why it was never caught by inspection — only running the actual
`ros2 run robot_localization ekf_node --params-file ...` surfaces it. This
bug **pre-existed in `ekf_local.yaml` and `ekf_wimu.yaml`** — meaning the
entire EKF-based localization stack has apparently never been startable on
the real robot, independent of the architecture question above. Fixed all
three files with a scoped regex (`\b0\b(?!\.)` → `0.0`, applied only inside
the 15-line `process_noise_covariance`/`initial_estimate_covariance` blocks).
Verified: `ros2 run robot_localization ekf_node --params-file <each file>`
now starts cleanly with no parse errors.

**Bug found #2 (real, pre-existing, in `test_step2_fused_odom.sh` and
`test_step4_motion_plan.sh` only) — EKF node launched under the wrong name,
silently ignoring its entire params file.** Both scripts ran
`ros2 run robot_localization ekf_node --ros-args --params-file
ekf_local.yaml` with no `-r __node:=...` remap. The executable's default
node name is `ekf_filter_node`, but `ekf_local.yaml`'s top-level key is
`ekf_filter_node_odom` — since ROS 2 params-file keys must match the actual
node name exactly, none of the file's parameters (not `odom0`/`imu0`,
not `publish_tf`, nothing) were ever applied; the node silently ran on
100% defaults. Verified with `ros2 node info` before/after: without the
name remap, the node has zero sensor subscribers at all; with
`-r __node:=ekf_filter_node_odom` added, it subscribes to exactly the
configured topics. (`challenge_master.launch.py`'s own `Node(name=...)`
already set this correctly — only these two standalone bench scripts were
affected.)

**Bug found #3 (real, in `test_step2_fused_odom.sh` only) — stale remap
target.** The script's `-r /lidar_odometry:=/wheel_cmd_vel` remap doesn't
match anything: `ekf_local.yaml`'s actual keys are `odom0: /wimu_odom` /
`odom1: /lidar_odometry_gated` — no topic literally named `/lidar_odometry`
exists in that config. Leftover from an older single-stage EKF schema that
predates the EKF1/EKF2 split. The script's own status echo also claimed an
`imu0: /imu/data` input that `ekf_local.yaml` never had (only `ekf_wimu.yaml`
did) — same stale-schema mismatch.

**User decision:** rather than patch these two scripts' remaps against the
now-superseded `ekf_local.yaml`, migrate both to the new merged
`ekf_odom.yaml` (one less legacy config path to maintain). Done:
- `test_step2_fused_odom.sh`: `EKF_CONFIG` → `ekf_odom.yaml`; remap now
  `-r /wheel_odom:=/wheel_cmd_vel` (matches `ekf_odom.yaml`'s actual
  `odom0` key); status echo corrected to list all three real inputs.
  Verified via `ros2 node info` — subscribes to `/wheel_cmd_vel`,
  `/imu/data`, `/lidar_odometry_gated` as intended.
- `test_step4_motion_plan.sh`: found a deeper issue while migrating — its
  "fused" mode manually launched a *second* `ekf_node` under the same name
  (`ekf_filter_node_odom`) as the one `localization.launch.py`'s
  `mode:=runtime` **already** starts internally (true before today's change
  too — `docs/testing_guide.md` already documented `localization.launch.py`
  as launching "FAST-LIO2 + EKF1 + EKF2 + NDT-OMP" together, not FAST-LIO2
  alone). A duplicate node name is a real collision, not just a redundant
  process. **User's decision: drop the manual EKF launch entirely** — Node
  1 (`localization.launch.py mode:=runtime`) already provides the fused
  EKF output for free; `fastlio` vs `fused` now purely selects which
  already-published pose topic (`/lidar_odometry` vs `/odometry/filtered`)
  feeds the motion plan executor. Also removed the stale `use_gps:=false`
  arg passed by both `test_step3` and `test_step4` — `use_gps` hasn't been a
  declared argument on `localization.launch.py` since GPS/navsat_transform
  was dropped (Step... earlier NDT-OMP work); passing an undeclared arg to
  `ros2 launch` doesn't error, it's just silently ignored, so this was
  harmless but misleading.

**Bug found #4 (real, major, master-launch-level) — the IMU driver was
never wired into anything.** `src/imu_can_interface` (vendored from
`ACEINNA_IMU_ROS2` develop branch) has its own working
`imu_sensors.launch.py`, and builds/runs fine standalone — but it was never
included by `challenge_master.launch.py`, never referenced by any
`profiles/*.env`, and never referenced by `scripts/run_robot.sh` or any
other bringup script. Both FAST-LIO2 (internal IMU-aided prediction) and the
localization EKF (`imu0: /imu/data` input) depend on `/imu/data` actually
being published somewhere — with this gap, a real competition run would
have had FAST-LIO2 running IMU-less and the EKF's `imu0` input permanently
starved, with **no error, crash, or warning anywhere** (ROS 2 subscriptions
just silently never receive messages if nothing publishes).
Fixed: added a new `use_imu` launch arg to `challenge_master.launch.py`
(BLOCK 6b, right after the Hesai lidar block), including
`imu_can_interface`'s `imu_sensors.launch.py` under `IfCondition(use_imu)`.
Per the ACEINNA IMU being wired to the **RPi's** CAN bus (`can1`, per
`imu_can_interface/config/imu_params.yaml`) — same device as the motor
driver and `cmd_vel_mux`, NOT the Jetson — `DIY_USE_IMU=true` in
`profiles/raspi.env`, `false` in `profiles/jetson.env` and `laptop.env`.
The Jetson-side FAST-LIO2/EKF consume `/imu/data` over the Zenoh bridge from
the RPi, same pattern as every other RPi-owned topic in this architecture.
Wired through `scripts/run_robot.sh`, `debug_robot.sh`, and
`scripts/health_check.sh` (added `/imu_node` to `REQUIRED_NODES` when
`DIY_USE_IMU=true`).

**Verification performed this pass** (all with real tooling, not
inspection-only):
- `python3 -c "import yaml; yaml.safe_load(...)"` + actually running
  `ros2 run robot_localization ekf_node --params-file <file>` for all three
  EKF configs — no parse errors, clean start/stop.
- `ros2 node info /ekf_filter_node_odom` before/after the node-name-remap
  fix — confirmed zero sensor subscribers without the fix, exactly the
  intended three with it, for both the master-launch config (`ekf_odom.yaml`)
  and the migrated bench-test remap.
- `colcon build --symlink-install --base-paths src --packages-skip
  differential-drive` — full 11-package rebuild after all changes, clean.
- `ros2 launch challenge_bringup challenge_master.launch.py --show-args` —
  confirms `use_imu` is a recognized argument, default `true`.
- `bash -n` on every modified script — no syntax errors.
- Directly sourced `profiles/raspi.env`/`profiles/jetson.env` — confirmed
  `DIY_USE_IMU=true`/`false` respectively (an unrelated pre-existing quirk
  in `scripts/env.sh` under `set -u` prevented testing via that wrapper
  specifically in this sandbox shell — reproduced identically with the
  older `DIY_USE_HESAI` var, confirming it's not new).
- Actually ran `ros2 launch diy_localization localization.launch.py
  mode:=runtime` end-to-end — confirmed FAST-LIO2 and the EKF start cleanly;
  confirmed NDT-OMP's pre-existing `GlobalMap.pcd`-not-found crash is real
  and immediate (already flagged red in the pipeline diagram, not new).

**Still open / not addressed this pass:**
- NDT-OMP's `GlobalMap.pcd` path mismatch (crashes on start) — pre-existing,
  tracked separately, needs either the real map generated or the hardcoded
  path in `ndt_localizer.yaml` corrected.
- `imu_sensors.launch.py`'s own `can_interface` launch arg is declared but
  never actually used by the node (it reads `can.interface` from the params
  YAML directly) — a minor bug in the teammate's own package, not fixed
  here (out of scope, upstream).
- `extrinsic_R`, `/hesai/points` vs `/lidar_points`,
  `differential-drive` package identity, URDF missing TF links — all still
  open from earlier steps. (ZED2i driver source — see below — is now
  resolved.)
- **Cosmetic-only, not fixed:** the "EKF1"/"EKF2" naming still appears in
  code comments (not functional logic) across several *other* packages this
  pass didn't touch — `diy_zone_nav/src/lidar_odom_gate_node.cpp`,
  `zone_nav_manager_node.cpp`, `diy_ndt_localization/src/ndt_localizer_node.cpp`,
  `diy_robot_description`'s URDF headers, `record_bag.sh`/`record_zone_nav.sh`.
  All still functionally correct (the topics they reference — `/odometry/
  filtered`, `/lidar_odometry_gated` — haven't changed), just describes the
  old two-EKF mental model. Low priority; flagging so a future pass doesn't
  need to re-discover it.

## Reuse Plan Step 7 — ZED2i camera driver source confirmed

User confirmed `stereolabs/zed-ros2-wrapper`
(https://github.com/stereolabs/zed-ros2-wrapper) is the actual ZED2i driver
in use on the real robot — resolves the caveat raised in Step 4 (the
`zed_wrapper` integration was built against this public repo's verified
launch args/topics, but a teammate had flagged it might not be the real
driver in use, so it was treated as not-yet-trustworthy until now).

**No functional code changes** — `challenge_master.launch.py`'s `_zed_launch`
already targeted the correct package/launch file/args; only documentation
and status markers updated to reflect the confirmation:
- `challenge_master.launch.py` BLOCK 7 comment: "NOT YET VERIFIED" →
  "CONFIRMED this is the actual driver in use", with the GitHub link.
- `docs/testing_guide.md` caveat #2: rewritten from "unconfirmed, treat
  use_zed:=true as not-yet-trustworthy" to "confirmed"; kept the still-open
  sub-issue (calibration scripts remain RealSense-specific) under the same
  caveat number so no other cross-references needed renumbering.
- `docs/pipeline_diagram.dot`: ZED2i node moved from red/dashed
  ("driver source unconfirmed") to yellow/solid ("CONFIRMED ... never run
  end-to-end yet") — package identity is no longer in question, but it
  still hasn't actually been run as part of this pipeline on hardware, so
  it isn't green yet.

**Still not done:** actually running `use_zed:=true` on the Jetson and
verifying the documented topics (`/zed/zed_node/rgb/image_rect_color`,
`/zed/zed_node/imu/data`, etc.) really appear via `ros2 topic list` — this
confirms the *source repo*, not yet the *live integration*.

## Reuse Plan Step 8 — Found a native map→odom alternative to NDT-OMP (BACKLOG, not implemented)

User asked whether FAST-LIO2 has anything usable instead of NDT-OMP for
map→odom localization, given `diy_ndt_localization` is confirmed broken
(hardcoded `GlobalMap.pcd` path that doesn't exist — see Step 6 verification
run). FAST-LIO2 itself has nothing built-in (pure LIO odometry, no map
anchor) — but investigating `LIO_Localization` (the source repo
`fast_lio_ros2` was vendored from) turned up a purpose-built companion
toolchain that has never been vendored into this repo:

- **`map_localizer`** — package.xml literally says "Coarse-to-fine ICP
  relocalization against a saved/refined map for FAST_LIO_Hesai_ROS2".
  Consumes FAST-LIO2's own native topics directly (`/Odometry`,
  `/cloud_registered_body`) instead of subscribing to raw lidar like
  NDT-OMP does. Map path + initial pose guess (`x,y,z,yaw,pitch,roll`) are
  passed via a runtime `/relocalize` service call
  (`slam_interfaces/srv/Relocalize`) — NOT hardcoded in a YAML, which
  sidesteps the exact class of bug that broke `diy_ndt_localization`.
  Two-stage coarse-to-fine `pcl::registration::icp` (rough: 0.25m voxel/5
  iterations; refine: 0.1m voxel/10 iterations).
- **`map_hba`** (Hierarchical Bundle Adjustment) — **confirmed to be the
  actual tool that produced `maps/refined_map.pcd`**, the exact file
  already shipped in this repo (`hba_node.cpp` hardcodes
  `m_maps_path / "refined_map.pcd"` as its output filename — matches
  verbatim). Strong evidence this toolchain (not NDT-OMP) is what the
  original map was built for.
- **`loop_pgo`** — loop closure / pose graph optimization (GTSAM iSAM2),
  used during mapping, not real-time localization.
- **`slam_interfaces`** — shared custom srv definitions (`Relocalize`,
  `IsValid`, `SaveMaps`, `SavePoses`, `RefineMap`).

None of these four packages are vendored into `DIY-Challenge-Repo` or
referenced anywhere in it today — this would be net-new integration work
(same pattern as the `fast_lio_ros2`/`imu_can_interface` vendoring earlier
this session), not a small patch.

**User decision: defer.** Keeping this documented as a backlog item rather
than implementing now. **Next step whenever this is picked up:** vendor
`map_localizer` + `slam_interfaces` (mirroring the existing vendoring
process — sync from `LIO_Localization`'s `develop` branch, verify against
real topic/param names, wire a gated block into
`localization.launch.py`/`challenge_master.launch.py`), decide whether to
run it alongside or instead of `diy_ndt_localization`, and update
`docs/pipeline_diagram.dot` once done.

## Reuse Plan Step 9 — REVERTED: ACEINNA IMU is launched independently, not integrated into this repo

User decision (reversing Step 2/Step 6's IMU vendoring/wiring): the
ACEINNA IMU driver will be launched independently on the RPi as its own
separate process — no need to integrate it into `DIY-Challenge-Repo` at
all. Removed everything that vendoring/wiring added this session:

- **Deleted `src/imu_can_interface`** entirely (it was untracked/never
  committed, so a plain `rm -rf` was sufficient — no git history to clean up).
- **`challenge_master.launch.py`**: removed BLOCK 6b (the
  `IncludeLaunchDescription` for `imu_sensors.launch.py`), the `use_imu`
  `DeclareLaunchArgument`, and the `use_imu` `LaunchConfiguration`. Replaced
  with a plain comment noting the IMU is launched independently and
  FAST-LIO2/EKF still consume `/imu/data` as an external input over Zenoh.
  Docstring's STARTUP ORDER/argument table updated to match.
- **`profiles/{jetson,raspi,laptop}.env`**: removed `DIY_USE_IMU` from all
  three; replaced with plain comments explaining the IMU is external.
- **`scripts/run_robot.sh`, `debug_robot.sh`**: removed `use_imu:=...` from
  the `ros2 launch` invocations and the status echo.
- **`scripts/health_check.sh`**: removed the `/imu_node` entry from
  `REQUIRED_NODES` (that node no longer belongs to this repo's launch
  responsibility). Kept the unconditional `/imu/data` topic-rate check —
  still valid regardless of where the driver runs.
- **`scripts/test_step2_fused_odom.sh`**: comments updated to describe the
  IMU driver as "the independent RPi IMU driver" rather than referencing a
  vendored package path.
- **`README.md`**: removed the `imu_can_interface/` line from the package
  tree; also fixed a stale "dual EKF fusion" description (from before
  Step 6's single-EKF collapse) while in the area.
- **`docs/testing_guide.md`**: caveat #7 rewritten (dropped the "IMU driver
  was never wired in" framing — no longer applicable since it's not wired
  in here at all, by design); removed the `use_imu` row from the argument
  table and both `imu_can_interface` rows from the launch-file inventory,
  replaced with one clear "Not in this repo at all" note.
- **`docs/pipeline_diagram.dot`/`.png`/`.svg`**: `imu_can_interface` node
  restyled from yellow ("just wired into master launch") to green+dashed
  ("launched independently on this device, outside this repo") — dashed
  per the diagram's own legend ("external / not yet integrated"), green
  because the driver itself is real, tested hardware code, just not
  something this repo is responsible for starting.

**Net effect:** `/imu/data` remains the expected topic name/contract
between the RPi (wherever it actually launches from) and the Jetson's
FAST-LIO2 + EKF over the Zenoh bridge — nothing about the EKF/FAST-LIO2
configs changed. Only the "who launches the driver" responsibility moved
back out of this repo. Verified: `colcon build --symlink-install
--base-paths src --packages-skip differential-drive` → 10 packages (was
11), clean; `ros2 launch challenge_bringup challenge_master.launch.py
--show-args` no longer lists `use_imu`.
