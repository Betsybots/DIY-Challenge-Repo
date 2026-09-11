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

**Update: a newer branch exists — `localizer_v2.2` (superseding `develop`
for this evaluation).** User switched `LIO_Localization`'s local checkout
to `origin/localizer_v2.2` (latest commit `7ffaab0` — "updated
maplocaliser with VGICP and Tuen IMU timesyncs"). Diffed against `develop`
and confirmed real, relevant improvements to the exact `map_localizer`
package flagged above:
- **ICP algorithm swap**: `pcl::IterativeClosestPoint` → `fast_gicp::FastVGICP`
  (voxelized generalized ICP) for both the rough and refine passes in
  `icp_localizer.h`/`.cpp` — generally more robust than plain point-to-point
  ICP, especially on sparse/structured lidar returns. New `fast_gicp`
  dependency, **already vendored** at `LIO_Localization/src/third_party/fast_gicp`
  (self-contained, not an extra external fetch needed).
- New tunable params in `map_localizer.yaml`: `rough_vgicp_resolution: 1.0`,
  `refine_vgicp_resolution: 0.5`, `num_threads: 4`.
- IMU timestamp-sync tuning also touched in `FAST_LIO_Hesai_ROS2` on this
  branch (per the commit message) — not yet individually diffed/verified.

**Still deferred, not implemented** — but if/when this backlog item is
picked up, vendor from `localizer_v2.2` instead of `develop`, and note the
extra `fast_gicp` dependency needs pulling in alongside `map_localizer` +
`slam_interfaces`.

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

## Reuse Plan Step 10 — Pulled real FAST-LIO2 fixes from LIO_Localization `localizer_v2.2`

User asked to integrate `localizer_v2.2`'s changes into our vendored
`fast_lio_ros2`. Diffed `develop` (what was originally vendored) against
`localizer_v2.2` (commit `7ffaab0`) scoped to `src/FAST_LIO_Hesai_ROS2` —
exactly 2 files changed, both applied here:

**1. `extrinsic_R` fix (config/qt64.yaml)** — resolves the long-flagged
determinant -1 (invalid rotation) bug. Teammate's committed fix changes the
matrix to plain identity:
```
[1,0,0; 0,1,0; 0,0,-1]  →  [1,0,0; 0,1,0; 0,0,1]
```
Verified with `python3 -c "import numpy as np; ..."`: old det=-1.0 (a
reflection), new det=+1.0 (a valid rotation). Also confirmed via the
vendored checker: `python3 src/fast_lio_ros2/tools/check_config.py --config
src/diy_localization/config/fast_lio_hesai_qt64.yaml` — extrinsic_R line
flips from FAIL to `[PASS] mapping.extrinsic_R valid rotation (det=1.0000)`.
**Important open caveat, documented inline in the YAML and in
testing_guide.md caveat #4**: this new value is plain IDENTITY, which does
NOT match the Rx(-90°) this same file derives in its own comments from a
live accelerometer + physical-mounting analysis. Applied the teammate's
real, committed value anyway (more authoritative than our own from-scratch
derivation), but flagged clearly that this discrepancy needs on-robot
re-verification (live accelerometer reading) before fully trusting it for
autonomous nav — not blindly trusted just because it's a valid rotation.
- Applied to both the actively-used
  `src/diy_localization/config/fast_lio_hesai_qt64.yaml` and the vendored
  reference copy `src/fast_lio_ros2/config/qt64.yaml` (kept in sync, per
  this repo's existing convention of treating the latter as a faithful
  vendored mirror).

**2. `blind: 0.3 → 0.5`** (config/qt64.yaml) — LiDAR blind-zone distance
tuning from the same teammate branch. Applied to both config copies.

**3. Threading/callback-group fix (src/laserMapping.cpp)** — real,
well-motivated fix, not a cosmetic change. Previously `sub_pcl_pc_`/
`sub_imu_` shared the node's single default callback group with the scan-
processing `timer_callback` (~100-450ms per scan doing ICP/map_incremental),
and `rclcpp::spin()` ran everything on one thread — so a long-running scan
callback could starve `imu_cbk` long enough to trip the `frontend.
max_imu_gap` shock-detection gate already active in our own config (`0.05s`
— see `fast_lio_hesai_qt64.yaml`'s `frontend:` block), causing false scan
rejections that looked like real IMU dropouts but weren't. Fixed by:
  - Giving both sensor subscriptions their own `Reentrant` callback group
    (shared buffers are already `mtx_buffer`-protected, so concurrent
    callbacks are safe).
  - Deepening the IMU subscription queue from `10` to `rclcpp::QoS(200)`.
  - Switching `main()` from single-threaded `rclcpp::spin()` to a
    `MultiThreadedExecutor` with 2 threads.
  Applied verbatim to `src/fast_lio_ros2/src/laserMapping.cpp` (our vendored
  copy needed the actual C++ patch, not just a config sync, since this is
  compiled code).

**Verification performed** (real tooling, not inspection-only):
- `python3 src/fast_lio_ros2/tools/check_config.py --config
  src/diy_localization/config/fast_lio_hesai_qt64.yaml` — extrinsic_R now
  PASSes (det=1.0000); pre-existing scan_line=32 FAIL and map_file_path WARN
  unchanged (both already-documented, intentional/known items, not
  regressions from this change).
- `colcon build --symlink-install --base-paths src --packages-select
  fast_lio_ros2` — clean rebuild, no new warnings/errors from the threading
  change.
- `colcon build --symlink-install --base-paths src --packages-skip
  differential-drive` — full 12-package workspace rebuild, clean.
- `ros2 run fast_lio_ros2 fastlio_mapping --params-file
  fast_lio_hesai_qt64.yaml` — starts cleanly, no crashes, clean shutdown on
  SIGTERM (smoke test only — no real lidar/IMU hardware in this sandbox, so
  this does not confirm the threading fix's actual runtime behavior under
  load; that needs the real robot).

**Docs updated:** `docs/testing_guide.md` caveat #4 rewritten (was "may
still be wrong, teammate says a fix exists but may need pulling" → now
describes the applied fix and the still-open identity-vs-Rx(-90°)
discrepancy). `docs/pipeline_diagram.dot`/`.png`/`.svg`: `fast_lio_ros2`
node annotated with the fix + "not yet re-tested on robot" caveat; also
fixed a pre-existing mislabel where the NDT-OMP node incorrectly carried an
"extrinsic_R fix pending" note (NDT-OMP's own config has no `extrinsic_R`
parameter at all — that flag belonged on the FAST-LIO2 node, not NDT-OMP).

**Not brought in from `localizer_v2.2`** (out of scope for this pass, per
Step 8's backlog decision): the `map_localizer`/VGICP/`fast_gicp` changes —
still deferred as documented in Step 8, since that's a net-new package
vendoring decision, not a patch to an already-vendored one like this
FAST-LIO2 update was.

**Still open:** the identity-vs-Rx(-90°) extrinsic_R discrepancy (needs a
live accelerometer re-verification on the real robot); everything else
already flagged as open from earlier steps (`/hesai/points` vs
`/lidar_points`, ZED2i live-topic confirmation, `differential-drive`
package identity, URDF TF links, `GlobalMap.pcd` path).

## Reuse Plan Step 11 — Replaced NDT-OMP with map_localizer (VGICP) for map→odom TF

User asked what NDT-OMP needs to provide `map→odom` TF to a downstream
controller (noting the controller may not be Nav2 — a teammate is working
on a different one), then separately asked whether `map_localizer` could
provide this more easily. Investigated both by reading the actual TF-
publishing code in each, not by inspection/assumption.

**Real, previously-undiscovered bug found in `diy_ndt_localization`
(NDT-OMP):** its `publishTF()` broadcasts the raw NDT scan-matched pose
(`current_pose_matrix_`, which is actually `map→base_link` — where the
robot IS right now) directly as `map→odom` TF:
```cpp
tf_msg.header.frame_id = map_frame_;   // "map"
tf_msg.child_frame_id = odom_frame_;   // "odom"
tf_msg.transform = /* pose_matrix, straight from NDT scan-matching */
```
There is no `tf2_ros::Buffer`/`TransformListener` anywhere in the class —
confirmed by grepping the whole file — despite both headers being
included (dead code, likely started and never finished). Since
`odom→base_link` is ALSO published separately by the EKF
(`ekf_odom.yaml`), composing `map→odom→base_link` via this bug would
double-apply the odom offset, corrupting the robot's true position in the
map — worse the more `odom` drifts. The function's own comment even
states the correct intended semantics ("the transform represents: odom
origin's pose in map frame") but the code doesn't compute that at all.
This bug has never manifested in practice because the node also can't even
start yet (hardcoded `GlobalMap.pcd` path issue, confirmed broken back in
Step 6/8's verification runs) — but it needed fixing regardless of the
controller question, since a real map→odom TF has to be composed
correctly no matter who consumes it downstream.

**Verified `map_localizer`'s TF math is correct, term-by-term:**
```cpp
// current_local_r/t = odom→body pose, from FAST-LIO2's own /Odometry
// map_body_r/t = map→body pose, from ICP-aligning the scan against the map
m_state.last_offset_r = map_body_r * current_local_r.transpose();
m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
```
Worked through the rotation/translation algebra and confirmed this is
exactly `map→odom = map→body × inverse(odom→body)` — the textbook-correct
composition (same pattern AMCL/every real localization node uses).

**Answer given for the "different controller" question:** `map→odom` TF
is fully consumer-agnostic — once broadcast correctly, ANY node (Nav2 or a
custom controller) reads it the same standard way via
`tf2_ros::Buffer`/`TransformListener::lookupTransform("map", "base_link",
...)`. No Nav2-specific plumbing needed regardless of which node publishes
it or who's downstream.

**User decision: switch to `map_localizer` now** (bringing Step 8's
backlog item off the shelf). Full integration performed:

1. **Vendored** `src/map_localizer` and `src/slam_interfaces` (plain copy,
   matching the `fast_lio_ros2` vendoring pattern) from `LIO_Localization`
   branch `localizer_v2.2` (commit `7ffaab0`).
2. **Added `fast_gicp`** (the VGICP dependency `map_localizer` needs) as a
   REAL git submodule at `third_party_ws/src/fast_gicp`, pointing to the
   actual public upstream (`https://github.com/SMRT-AIST/fast_gicp.git`,
   `master`) rather than copying LIO_Localization's local snapshot —
   confirmed via diff that LIO_Localization's copy only differs in a
   CUDA-specific Jetson compute-capability flag (inside an
   `if(BUILD_VGICP_CUDA)` block, OFF by default), so pointing at public
   upstream is equivalent for our CPU-only build and matches this repo's
   existing submodule convention (`ndt_omp_ros2`, `lidar_imu_calib`, etc.).
   Confirmed `BUILD_VGICP_CUDA` defaults OFF and the CUDA/nvbio/Eigen
   thirdparty submodule dirs are only referenced inside that CUDA-only
   CMake block — not needed for our sandbox (no CUDA toolkit here) or
   likely even on the Jetson unless CUDA acceleration is deliberately
   enabled later.
3. **Fixed a real config bug before it could bite**: `map_localizer.yaml`'s
   vendored default is `odom_topic: /Odometry`, but this repo's
   `localization.launch.py` remaps FAST-LIO2's `/Odometry` to
   `/lidar_odometry` for naming consistency. Since `map_localizer` syncs
   cloud+odom via `message_filters::ApproximateTime` (needs BOTH topics
   before its callback ever fires), leaving the vendored default would
   have meant `map_localizer` silently never processed a single scan — no
   error, just permanently zero output (same failure shape as the EKF
   node-name-mismatch bug found in Step 6). Fixed by changing
   `odom_topic` to `/lidar_odometry` in the active runtime config.
   `/cloud_registered_body` (the other sync input) was confirmed
   unaffected by any remap.
4. **Split into vendored-reference vs. active-runtime configs**, matching
   this repo's established convention (`qt64.yaml` vs
   `fast_lio_hesai_qt64.yaml`): `src/map_localizer/config/map_localizer.yaml`
   stays an untouched, faithful mirror of upstream; the real, customized
   config with all of the above fixes and reasoning lives at
   `src/diy_localization/config/map_localizer.yaml`.
5. **Wrote `trigger_map_relocalize.py`** (new,
   `src/diy_localization/scripts/`): `map_localizer` does NOT auto-load a
   map at startup — confirmed by reading `localizer_node.cpp`'s
   constructor — it only loads a `.pcd` (and sets the initial pose guess)
   in response to a `slam_interfaces/srv/Relocalize` service call. Without
   something to make that call, `map_localizer` would start, subscribe to
   its topics, and just never do anything, forever, with no error (same
   "silently does nothing" shape as several other bugs found this
   session). This script waits for the `/relocalize` service to become
   available, then calls it once with a configurable `pcd_path` +
   initial pose (defaults to map origin, matching NDT-OMP's documented
   assumption), then exits — it is not a long-lived pipeline node.
6. **Added a `map_pcd_path` launch argument** to `localization.launch.py`,
   defaulting to `$DIY_ROS_WS/src/DIY-Challenge-Repo/maps/refined_map.pcd`
   when `DIY_ROS_WS` is set (reusing the existing profile-level env var
   convention already established for other machine-specific paths,
   rather than inventing a new one) — avoids the exact hardcoded-path
   anti-pattern that broke NDT-OMP's `GlobalMap.pcd` reference.
7. **Deprecated `diy_ndt_localization`** in place rather than deleting it:
   added a prominent deprecation header to both `package.xml`'s
   `<description>` and the top of `ndt_localizer_node.cpp` explaining the
   TF bug and pointing here. The package still builds (no reason to break
   it) but is no longer included by `localization.launch.py`.

**Bug found and fixed DURING integration, via actually launching, not just
building:** the first end-to-end launch attempt crashed —
`trigger_map_relocalize.py` raised `InvalidParameterTypeException` on
`initial_x`. Root cause: `LaunchConfiguration(...).perform(context)` always
returns a Python `str`, but the script declares these as `double`
parameters — ROS 2 rejects setting a double parameter from a string value
at the type level. Fixed by wrapping each in `float(...)` in the launch
file. This is the same general class of type-mismatch bug as the EKF
YAML int/float issue from Step 6 — caught only by actually running it.

**Verification performed (real tooling throughout):**
- `fast_gicp`, `slam_interfaces`, `map_localizer` all build clean in this
  sandbox (`colcon build --packages-select ...`), confirming the CPU-only
  VGICP path needs no CUDA toolkit.
- Full workspace rebuild: 14 packages clean (`colcon build --symlink-install
  --base-paths src --packages-skip differential-drive`).
- Real end-to-end launch: `ros2 launch diy_localization
  localization.launch.py mode:=runtime map_pcd_path:=<real path to
  maps/refined_map.pcd>` — `fastlio_mapping`, `ekf_node`,
  `map_localizer_node`, and `trigger_map_relocalize.py` all start cleanly;
  `trigger_map_relocalize.py` logs `/relocalize succeeded: relocalize
  success` against the real map file, then exits (as designed).
- Synthetic-data test: manually published fake `/cloud_registered_body`
  (4-point cloud) + `/lidar_odometry` (Odometry, translation (2,1,0))
  messages via a throwaway rclpy script, then confirmed with `ros2 run
  tf2_ros tf2_echo map odom` that a `map→odom` TF is genuinely being
  broadcast under the correct frame names, continuously. The observed
  transform was identity/zero (the trivial synthetic cloud couldn't
  produce a real ICP match against the actual loaded map, so the code
  correctly held its default fallback rather than crashing) — this
  confirms the TF *mechanism* end-to-end (service call → map load → sync
  → align → broadcast, no crashes) but does NOT confirm real alignment
  accuracy, which needs actual matching lidar scan data (real hardware).

**Found and fixed a second, unrelated regression while investigating:**
`third_party_ws/COLCON_IGNORE` — a file that blocks `colcon list`/`build`
from seeing ANYTHING under `third_party_ws` (not just `fast_gicp` — also
`ndt_omp_ros2`, `lidar_imu_calib`, `LIO-SAM`, `imu_utils_ros2_humble`).
Confirmed empirically (`colcon list` found 0 packages with it present, all
6 with it removed). This is a re-introduction of the *exact* bug already
found and fixed earlier this session (see the "Critical build gotcha" note
in this doc's technical history) — it came back in via the teammate's
commit `eb85d42` during the `git pull --rebase` from earlier today, which
this session's own commits didn't touch either way (nothing removed it,
their commit added it, so the rebase result kept it). Removed again
(`git rm`). Verified: `cd third_party_ws && colcon list` now shows all 6
packages; `colcon build --packages-skip code_utils imu_utils lio_sam`
succeeds for `ndt_omp_ros2`/`fast_gicp`/`lidar_imu_calib` (the 3 skipped
packages fail for unrelated, pre-existing reasons — missing system
packages `elfutils/libdw.h` and `GTSAM`, nothing to do with today's work).

**Docs updated:** `docs/testing_guide.md` (new caveat #8, launch-file
inventory row for `ndt_localization.launch.py` marked deprecated,
`map_localizer` references throughout). `docs/pipeline_diagram.dot`/`.png`/
`.svg`: replaced the `ndtomp` node with `maplocalizer` + a new
`reloc_trigger` satellite node, corrected edges (map_localizer consumes
FAST-LIO2's `/lidar_odometry`+`/cloud_registered_body`, not raw
`/hesai/points`; NDT-OMP's old direct hesai-points edge removed).

**Still open:**
- Whether `maps/refined_map.pcd` is genuinely the real course map (vs. a
  bench/test map) — not verified this pass.
- Real VGICP alignment accuracy against actual lidar data — needs real
  hardware, impossible to verify further in this sandbox.
- All previously-flagged open items unaffected by this change:
  `/hesai/points` vs `/lidar_points`, ZED2i live-topic confirmation,
  `differential-drive` package identity, URDF TF links, the
  identity-vs-Rx(-90°) `extrinsic_R` discrepancy from Step 10.

## Reuse Plan Step 12 — setup.sh missing runtime packages (found while writing Jetson quickstart)

While answering "how do I run this on a freshly-cloned Jetson", tested
`setup.sh`'s documented build step literally (removed `install/` for the
relevant packages, ran the exact `colcon build --packages-select ...` list
setup.sh uses) and found it was missing packages needed at runtime:

- **`diy_zone_nav`** — referenced directly by `challenge_master.launch.py`
  (`Node(package='diy_zone_nav', executable='lidar_odom_gate_node', ...)`,
  always launched whenever `use_localization=true`, i.e. every normal run)
  but was never in `setup.sh`'s `--packages-select` list, at any point
  before today's changes either — a pre-existing gap, not something
  today's work introduced.
- **`fast_lio_ros2`, `slam_interfaces`, `map_localizer`** — all needed at
  runtime by `diy_localization/launch/localization.launch.py` (today's
  Step 11 changes), same gap: colcon's `--packages-select` does NOT
  auto-include a package's `exec_depend`s the way `--packages-up-to` does,
  so listing only `diy_localization` builds its launch/config files fine
  but never builds what those launch files actually try to run.

Net effect: a fresh `bash setup.sh jetson` followed by `scripts/run_robot.sh
jetson` (exactly the documented workflow) would have failed at `ros2
launch` time with "package not found" for whichever of these was missing —
setup.sh itself would report success, masking the problem until the actual
launch attempt.

**Fixed:** added all four to `setup.sh`'s explicit `--packages-select`
list, with a comment explaining why (and how this class of bug is found —
by removing a package's `install/` dir and actually re-running the
documented build+launch sequence, not just reading the list and assuming
it's complete). Verified by literally doing that: removed
`install/{fast_lio_ros2,slam_interfaces,map_localizer,diy_zone_nav}`, ran
the corrected package list, confirmed all 9 packages build, then re-ran
the full `ros2 launch diy_localization localization.launch.py` end-to-end
test from Step 11 again to confirm nothing broke.

**Also fixed while in this file:** `challenge_bringup/package.xml` still
declared `<exec_depend>realsense2_camera</exec_depend>` from before the
ZED2i swap (should be `zed_wrapper`) — stale leftover from the camera
migration a few steps back that was never updated in the package.xml
itself (only the launch file was fixed at the time). Also:
`map_localizer/package.xml` never declared `yaml-cpp`/`eigen`/
`libpcl-all-dev` as `<depend>`s despite `CMakeLists.txt` hard-requiring
all three via `find_package(... REQUIRED)` — meaning `rosdep install`
(what `setup.sh` runs) would never know to install `libyaml-cpp-dev` on a
genuinely fresh machine that doesn't already have it. Confirmed
`libyaml-cpp-dev` is the correct resolved package name via `rosdep resolve
yaml-cpp`; added all three (matching the exact same dependency names
`fast_gicp`'s own package.xml already uses for `eigen`/`libpcl-all-dev`,
for consistency). Rebuilt `map_localizer` clean after the fix.

**Still not fixed / flagged for later:** this exposed a broader pattern —
`challenge_bringup`'s `package.xml` doesn't declare `exec_depend` on ANY
of the repo-local packages its own launch file `Node()`-launches
(`diy_cmd_vel_mux`, `diy_estop_controller`, `diy_robot_description`,
`diy_zone_nav`, `diy_localization`). Switching `setup.sh` to
`colcon build --packages-up-to challenge_bringup` instead of an explicit
list would be more robust long-term (confirmed empirically that
`--packages-up-to` correctly resolves `exec_depend` — that's how this
pass's fix was verified) — but doing that properly requires first fixing
every affected package.xml's exec_depend declarations across the whole
repo, which is a larger, separate cleanup pass than what was in scope
here. Documented as a known gap; the explicit list in `setup.sh` is
correct and complete for today's purposes but requires manual updating
again if a future change adds another new runtime-only package dependency.

## Reuse Plan Step 13 — Decoupled the custom A*/PD controller from diy_zone_nav

User's teammate is building a custom navigation stack (nav2_map_server-only
+ a custom A* planner + PD/pure-pursuit path follower — NOT full Nav2) that
already exists in this repo via an earlier rebase (`src/diy_planning`,
`src/diy_motion_planner`) but was never investigated until now. User asked:
design things so `diy_zone_nav` can be removed entirely with zero impact,
since the plan is for the controller to consume "info from the zone
navigator" and they want that dependency to be optional/safe-to-remove
from day one, not bolted on and only discovered to be load-bearing later.

**Investigated the actual code (not assumed) before designing anything:**
- Grepped `diy_planning`/`diy_motion_planner` for any reference to
  `zone_nav`, `/nav_mode`, `/speed_limit`, `SpeedLimit` — **zero hits**.
  The custom controller already has NO existing coupling to zone_nav at
  all. `a_star_planner_node`'s `goal_callback` idles gracefully (early
  return, no crash/hang) whenever `/goal_pose` hasn't been received —
  confirmed by reading the guard clauses directly.
- Confirmed `diy_cmd_vel_mux`'s AUTONOMOUS mode reads `/cmd_vel_nav`
  unconditionally with its own staleness watchdog, completely independent
  of zone_nav — BLIND_DRIVE mode only activates via an explicit
  `SetParameters` service call that only `zone_nav_manager_node` makes, so
  removing zone_nav simply means that mode is never entered; everything
  else is unaffected. No mux changes needed.
- Teammate's own diagrams (shared as images) confirm the controller
  publishes on `/cmd_vel_nav` — the exact topic name `cmd_vel_mux` already
  reads for AUTONOMOUS mode. Zero wiring changes needed there.
- Teammate confirmed (via relayed message) they use ONLY `nav2_map_server`
  + `nav2_lifecycle_manager` (map serving), not the full Nav2 stack (no
  `planner_server`/`controller_server`/costmap layers). This means
  `zone_nav`'s `/speed_limit` output (designed to throttle Nav2's
  `controller_server`) and its costmap-layer `SetParameters` calls have
  **no service to call at all** in this architecture — confirmed
  `zone_nav_manager_node.cpp` already checks `service_is_ready()` before
  every such call and skips gracefully with a warn log if the service
  isn't there, so this was already safe, just newly-relevant.
- **Net finding: `diy_zone_nav` is already fully orphaned/optional in this
  new architecture** — nothing in the custom controller path consumes any
  of its outputs, and it already fails gracefully when its own targets
  (Nav2 services) don't exist. No code changes were needed to achieve
  "can take zone_nav out with no issues" for the EXISTING pipeline — that
  was already true. Documented this clearly (pipeline diagram, this entry)
  so it doesn't need re-discovering.

**Two real, separate bugs found and fixed while investigating this stack:**
1. **`pd_navigation.launch.py` / `pure_pursuit_navigation.launch.py`**:
   `cmd_vel_topic` launch argument defaulted to `/cmd_vel` (the simulation
   value) even though its sibling arguments (`base_frame`, `use_sim_time`)
   already default to their HARDWARE values (`base_link`, `false`) — an
   inconsistent default mix. A bare hardware launch with no override would
   have the PD/pure-pursuit controller silently publish into `/cmd_vel`,
   which `cmd_vel_mux` never subscribes to — robot simply never moves
   under autonomous test, no error anywhere. Fixed both launch files'
   `cmd_vel_topic` default to `/cmd_vel_nav`, matching the already-correct
   hardware-default pattern of the other args.
2. **Neither `pd_motion_planner_node.py` nor
   `pure_pursuit_motion_planner_node.py` published any signal when a goal
   was reached** — only a log line (`get_logger().info('Goal reached!...')`),
   no ROS topic at all. This blocks any future automated waypoint sequencing
   (nothing to know "we're done, send the next goal"). Added a
   `/pd/goal_reached` (`std_msgs/Bool`) publisher to both controller nodes,
   published at the exact point each already detects goal completion — a
   minimal, purely additive change (no existing behavior altered).

**New package: `diy_waypoint_sequencer`** — built per the user's explicit
design ask (an automated `/goal_pose` publisher for competition runs where
no human clicks RViz goals), designed from the start to be safely
removable:
- Deliberately a brand-new, separate, minimal package — NOT added inside
  `diy_zone_nav` or `diy_planning`/`diy_motion_planner`. This was a real
  design decision: putting it inside `diy_zone_nav` would re-couple "can I
  remove zone_nav" with "do I still get automated goal sequencing",
  exactly the ambiguity this whole task was about avoiding.
- `waypoint_sequencer_node`: loads a simple waypoints YAML (`label, x, y,
  yaw` list — deliberately NOT reusing `zone_waypoints.yaml`'s much richer
  schema, which encodes Nav2-costmap/BLIND_DRIVE concepts this simpler
  stack doesn't use), waits for `/green_light` (same competition
  start-trigger topic/convention already established for `zone_nav`),
  publishes `/goal_pose` for the current waypoint, advances on
  `/pd/goal_reached`, optionally loops back to the first waypoint at the
  end (`loop: true` in the YAML or launch arg).
- If this node is not run: `/goal_pose` is simply never auto-published —
  a human can still publish it manually (RViz "2D Goal Pose", or `ros2
  topic pub`) with zero code changes anywhere else, confirmed by design
  (the A* planner doesn't care who publishes `/goal_pose`).

**Verification performed (real tooling, not just written and assumed
correct):**
- `colcon build --packages-select diy_waypoint_sequencer` (and full
  15-package workspace rebuild) — clean.
- `ros2 launch diy_waypoint_sequencer waypoint_sequencer.launch.py
  --show-args` — confirms all 5 launch arguments resolve correctly,
  including the default `waypoints_file` path resolving to the installed
  package share directory.
- **Real functional test** (not just syntax/build): wrote a throwaway
  rclpy test harness (`tf_transformations` isn't installed in this sandbox
  and there's no `sudo` access to add the tiny apt package — stubbed just
  that one function with equivalent pure-math since `diy_motion_planner`
  has the exact same pre-existing sandbox dependency gap) that actually
  ran the node and verified, via real published/subscribed ROS messages:
  no premature publish before `/green_light`; first waypoint published
  correctly on green light; `/pd/goal_reached` correctly advances the
  index; sequence stops cleanly with no extra publish after the last
  waypoint when `loop:false`; loop correctly wraps back to waypoint 0 when
  `loop:true`; `wait_for_green_light:false` auto-starts after
  `start_delay_s` with no green-light message at all; duplicate
  `/green_light` messages after start are safely ignored (idempotent).
  All 7 behavioral assertions passed.

**Docs updated:** `docs/pipeline_diagram.dot`/`.png`/`.svg` — replaced the
generic "Nav2 (never run)" placeholder with the real architecture
(`nav2_map_server`+`nav2_lifecycle_manager`, `a_star_planner_node`,
`pd`/`pure_pursuit_motion_planner_node`, the new `waypoint_sequencer`), and
`diy_zone_nav` re-drawn as explicitly ORPHANED with no consumers, with an
explanatory label so this doesn't need re-investigating later.

**Still open / not addressed this pass:**
- The custom controller stack (`diy_planning`, `diy_motion_planner`,
  `diy_waypoint_sequencer`) is not yet wired into
  `challenge_master.launch.py` at all — currently only launchable
  standalone via `pd_navigation.launch.py`/`pure_pursuit_navigation.launch.py`.
  Master-launch integration (a `use_custom_nav`-style flag, deciding how it
  coexists with/replaces the `diy_zone_nav`/Nav2 blocks already there) is a
  separate, larger decision not made in this pass.
- Whether `diy_zone_nav` should eventually be deleted outright (vs. kept
  orphaned-but-present) wasn't decided — no urgency since it's already
  proven harmless to leave in place.
- `tf_transformations` is not installed in this sandbox (no `sudo`) —
  affects verifying this AND the pre-existing `diy_motion_planner` package
  identically; not a new gap introduced by this pass.

## Reuse Plan Step 14 — Re-verified URDF status: lidar/imu genuinely fixed, camera still missing, no GPS at all

User asked whether a recently-updated xacro is "the URDF" and whether it
resolves the long-flagged "URDF missing lidar_link/imu_link/camera_link/
gps_link" item. Read `src/diy_robot_description/urdf/robot.urdf.xacro` in
full (all ~440 lines, not just the header comment) — this is confirmed to
be the actual active URDF (`description.launch.py` loads it directly).

**Corrected finding — the old flag was too coarse, partially stale:**
- **`lidar_link`, `imu_link`: genuinely fixed**, not just claimed-fixed in
  a comment. Real `<xacro:include>` of `lidar.urdf.xacro`/`imu.urdf.xacro`,
  each defining a real `<link>` + fixed `<joint parent="base_link">`, with
  real non-zero measured offsets (`sensor_xoff_from_AR = 0.0568325` /
  2.2375in, `Lidar_base_height_from_AR = 0.149225` / 5.875in,
  `imu_height_from_AR = 0.0333375` / 1.3125in) — not placeholder zeros.
  Verified by actually running `xacro robot.urdf.xacro` and confirming
  `<link name="lidar_link">`/`<link name="imu_link">` appear in the real
  generated URDF output.
- **`camera_link`: still genuinely missing** — no include, no macro, no
  inline definition anywhere in the active file. The header comment's
  "fixed" claim was false for this one. Confirmed the actual link/joint
  definitions DO already exist, fully written, in two OTHER files in this
  repo (`robot.urdf` — a static, non-xacro file not loaded by anything —
  and the deprecated `robot-old.urdf.xacro`) — they were just never ported
  into the active `robot.urdf.xacro` when it was reworked. Both existing
  definitions reference "RealSense D435i" in their comments, which is
  itself stale (confirmed earlier this session: the real camera is ZED2i).
- **`gps_link`: user confirmed there is no GPS hardware on this robot at
  all** — so this was never actually a gap, just a stale assumption
  carried in multiple docs/configs (see below). Removed from every
  "required frame" list rather than left as a to-do.

**Real, consequential bug found while re-checking this:**
`challenge_master.launch.py`'s `_zed_launch()` forces `zed_wrapper`'s own
`publish_urdf`/`publish_tf`/`publish_map_tf` to `false`, with the
justification (in-code comment) *"diy_robot_description publishes the
static camera_link transform"*. That premise is false — confirmed
`camera_link` doesn't exist in the active URDF at all. **Net effect:
`base_link→camera_link` currently has NO publisher anywhere in the running
system** — not the URDF (doesn't define it), not zed_wrapper (deliberately
disabled). Anything needing that transform (e.g. projecting ZED2i
image/depth data into the robot frame) would silently fail a TF lookup.
Not fixed in this pass (fix requires the camera_link port decision below,
still pending) — but the finding itself is real and now documented in the
pipeline diagram directly on both the `robot_state_publisher` and `zed`
nodes so it isn't lost.

**GPS cleanup performed** (real, since the user confirmed no hardware
exists at all): `DIY_USE_GPS=true` was set on both `profiles/jetson.env`
and `profiles/raspi.env` — meaning `scripts/health_check.sh` would check
for a `/gps/fix` topic that can never exist, producing a false pre-flight
"failure" during actual competition prep. Fixed:
- `profiles/{jetson,raspi,laptop}.env`: `DIY_USE_GPS=false` everywhere,
  with a comment explaining there's no GPS hardware at all (was already
  `false` on laptop.env, just had a slightly misleading comment).
- `src/diy_robot_description/package.xml`: description no longer claims
  `camera_link (RealSense D435i)` and `gps_link (RTK antenna)` as defined
  frames — corrected to describe the real current state (lidar/imu real,
  camera not yet ported + wrong brand in the old reference copies, no GPS
  at all).
- `robot.urdf.xacro`'s header comment: fully rewritten — dropped the
  stale `navsat_transform / EKF2` reference (map→odom is `map_localizer`
  now; there's a single EKF, not "EKF2"), corrected the lidar_link/
  imu_link status to REAL, marked camera_link as not-yet-defined with the
  ZED2i note, and removed gps_link from the required tree entirely instead
  of listing it as a to-do.
- `docs/pipeline_diagram.dot`/`.png`/`.svg`: `robot_state_publisher` node
  updated to show lidar_link/imu_link as fixed (green check) and only
  camera_link as the real remaining gap, with GPS explicitly noted as
  not-applicable rather than missing. `zed` node recolored red (from
  yellow) and annotated with the base_link→camera_link-has-no-publisher
  finding, since this is now a confirmed real gap, not just "untested."

**Verification performed:** `xacro src/diy_robot_description/urdf/
robot.urdf.xacro` (the real xacro processor, not just an XML well-formed
check) — exits 0, and the generated URDF was grepped to confirm exactly
`lidar_link`/`imu_link` exist and `camera_link`/`gps_link` do not, matching
the corrected documentation exactly.

**Still open:**
- Whether to port `camera_link`/`camera_optical_link` into
  `robot.urdf.xacro` now (definitions + real ZED2i-corrected comments
  ready to go from `robot.urdf`) — asked the user, not yet answered/acted
  on as of this entry.
- Once `camera_link` exists for real, revisit whether `zed_wrapper`'s
  `publish_tf`/`publish_urdf` should stay forced `false` (correct, once
  the URDF really does own that frame) or whether the OpaqueFunction
  comment just needs updating to stop citing a false premise.
- All previously-flagged open items unaffected by this pass: `/hesai/
  points` vs `/lidar_points`, ZED2i live-topic confirmation,
  `differential-drive` package identity, `extrinsic_R` identity-vs-
  Rx(-90°) discrepancy, `GlobalMap.pcd`/`refined_map.pcd` naming/course
  authenticity question.

## Reuse Plan Step 15 — camera_link resolved (not needed); new pcd_to_pgm.py tool

**camera_link question resolved:** user confirmed the ZED2i is not used
for odometry, VSLAM, AprilTag, or any other spatial-fusion purpose in this
repo — confirmed empirically by grepping for any consumer of
`/zed/zed_node/{left,right}/image_rect_color` or any VSLAM/AprilTag/
perception node anywhere in the repo (none found). Since nothing needs
`base_link->camera_link` to resolve, this is not a gap needing a fix
(same treatment as `gps_link` in Step 14) — `camera_link` does NOT need to
be ported into `robot.urdf.xacro`, and `zed_wrapper`'s `publish_tf=false`
is fine to leave as-is. Downgrades the "no publisher for
base_link->camera_link" item from a warning to a non-issue; still worth a
follow-up pass to update `docs/pipeline_diagram.dot`/`jetson_bringup_guide.md`
§6 to reflect this (not done in this entry).

**New tool: `scripts/pcd_to_pgm.py`** — while answering "how do I run just
FAST-LIO2 to build a course map", found a real, previously-undocumented
gap: FAST-LIO2's own `/map_save` service already works (saves a 3-D
`.pcd`), but nothing in this repo converts that `.pcd` into the 2-D
occupancy-grid PGM that `nav2_map_server`/the A* planner need.
`scripts/generate_course_pgm.py` only builds a PGM from a hand-drawn
course diagram image (not real sensor data); `scripts/preprocess_map.py`
only cleans up a PGM that already exists. Built a new, dependency-free
(numpy + opencv only, no open3d/pcl) converter:
- Minimal ascii/binary PCD reader (mirrors `fast_lio_ros2/tools/
  check_map.py`'s own `_load_pcd_minimal()`, reimplemented rather than
  cross-package-imported since `tools/` isn't on the Python path).
- `--preview-only` prints the point cloud's real x/y/z bounding box and a
  z-histogram, so the height slab used to isolate walls from floor/
  ceiling (`--z-min`/`--z-max`) is chosen from real data, not guessed.
- Projects the chosen z-slab to a 2-D grid at `--resolution`, writes a
  PGM (trinary convention: 0=occupied/black, 254=free/white) sized to the
  point cloud's own bounding box, plus a companion YAML with the origin
  computed from that same bounding box (NOT assumed `[0,0,0]` — unlike
  the diagram-derived maps, a real lidar scan is centered wherever
  FAST-LIO2 happened to start).
- Deliberately does zero wall-smoothing/denoising itself — designed to
  feed directly into the existing, already-tested `preprocess_map.py` for
  that step, avoiding duplicated logic.

**Verified via a real synthetic test** (not just written and assumed
correct): built a synthetic binary PCD (4x4m square room wall loop at
z=0.1-1.0m, plus floor-plane noise at z~0 and stray ceiling points at
z~2.5-3.0m) and ran the full pipeline end-to-end:
1. `--preview-only` correctly showed the floor spike, wall band, and
   sparse ceiling stray points in the z-histogram.
2. Real conversion with `--z-min 0.05 --z-max 1.1` produced a clean binary
   (0/254 only) PGM showing an exact, correctly-scaled closed square wall
   loop — verified by rendering it to PNG and visually inspecting it.
3. Piped that PGM straight into `scripts/preprocess_map.py --keep-walls
   1` with zero errors — the smoothed output was the same clean square,
   confirming the two scripts compose correctly.
4. Also tested: ASCII-format PCD parsing, missing-file error path,
   missing `--z-min`/`--z-max` error path, and an out-of-range z-slab
   (zero points survive) error path — all fail with clear messages and
   exit code 1 rather than crashing or silently producing garbage output.

**Docs updated:** `docs/testing_guide.md` §9 — added `pcd_to_pgm.py` to
the maps/zones table, plus a new "Building a course map from a real
FAST-LIO2 scan" walkthrough (the exact `fastlio_mapping` standalone launch
command, why not to use `fast_lio_ros2`'s own `lio_localizer.launch.py`
[stale bundled `qt64.yaml`], the `/map_save` service call, and the full
`pcd_to_pgm.py` -> `preprocess_map.py` chain).

**Still open:**
- No "FAST-LIO2-only" launch file exists — `localization.launch.py`'s
  `mode:=mapping` is dead code (every block is gated on
  `mode == "runtime"` only); the standalone `ros2 run` command documented
  above is a workaround, not a proper fix. Worth eventually adding a real
  `mode:=mapping` implementation to that launch file.
- `pcd_to_pgm.py` has not been run against a REAL FAST-LIO2-saved `.pcd`
  yet, only a synthetic hand-built one — the real point type
  (`pcl::PointXYZINormal`) and any real-world scan noise/density patterns
  haven't been exercised. Re-verify the first time a real scan is
  available.
- `preprocess_map.py`'s existing `--keep-walls`/`--close-kernel` tuning
  guidance was written for diagram-derived maps; real lidar scans may
  need different defaults (noisier, more fragmented walls) — expect some
  iteration the first time this runs on real course data.

## Reuse Plan Step 16 — new pick_waypoints.py tool for controller testing

User asked how to generate automatic waypoints for controller testing.
Clarified this meant an interactive click-to-pick tool (a human still
chooses where, but the tool handles pixel->world conversion, heading
computation, and schema-correct output), as opposed to a fully algorithmic
generator (random free-space sampling or a fixed patrol pattern) — the
latter was not built.

**New tool: `scripts/pick_waypoints.py`** — same click-to-pick pattern
already established by `scripts/register_zones_to_map.py`'s `--pick` mode
(same `px_to_world()` convention, same matplotlib click-handling
approach), purpose-built for `diy_waypoint_sequencer`'s simpler schema:
- Click waypoints in visit order on a real map image; each click is
  checked against the map's own occupancy value immediately (warns if the
  point lands on an occupied/unknown pixel, so a bad pick is caught before
  it ever reaches the robot); `u` undoes the last point.
- Heading (`yaw`) is computed automatically as the bearing toward the
  next waypoint (a sane default a path-following controller test actually
  wants) — `--uniform-yaw` overrides this with a fixed value if preferred.
- Writes `waypoints.yaml` in `diy_waypoint_sequencer`'s exact schema
  (`frame_id`/`loop`/`waypoints:[{label,x,y,yaw}]`) — confirmed by reading
  `waypoint_sequencer_node.py`'s own `_load_waypoints()` parser, not just
  assumed.
- `--overlay` saves a PNG with numbered markers + heading arrows for a
  visual sanity check before running on the robot (same idea as
  `register_zones_to_map.py`'s own `--overlay`).

**Verified (the parts that can be, in a sandbox with no display):** all
non-interactive logic — `px_to_world()` round-trip and occupancy lookup
against the REAL `maps/global_map_2_smooth.pgm` (not synthetic data),
`compute_headings()` bearing math against a known square path (including
the single-point edge case), the output YAML schema round-tripped and
checked key-for-key against `waypoint_sequencer_node.py`'s actual parser
expectations, and `--overlay` rendering (headless/Agg backend) against
real free-space points sampled from the actual test map — the overlay
correctly showed all points inside the real ring-shaped course corridor
with correct path order and heading arrows. Also verified the
missing-map-file error path exits cleanly (added a check — the first
version raised a raw traceback here, same as `register_zones_to_map.py`
still does today; fixed only in the new script).

**Not verified (inherent to being an interactive GUI tool):** the actual
mouse-click event loop itself was not exercised end-to-end in this
sandbox (no display) — only every function it calls was. Same caveat
already applies to the existing `register_zones_to_map.py --pick`.

**Docs updated:** `docs/testing_guide.md` §9 (new table row) and
`docs/custom_nav_stack_design.md` §5 (+ regenerated
`Custom_Nav_Stack_Design.pdf`, now 7 pages, both new pages visually
verified) — new "Generating real waypoints for controller testing"
subsection.

## Reuse Plan Step 17 — pick_waypoints.py: loop-closing heading fix + initial-pose hint

Two follow-ups from real usage of `scripts/pick_waypoints.py` on an actual
23-point course (`maps/test_waypoints.yaml`):

**1. Fixed a real bug in `compute_headings()`:** it was ignoring `--loop`
entirely — the last waypoint's heading always just repeated the previous
segment's direction, even when the path loops back to wp1. Now, if
`loop=True`, the last waypoint's heading is computed as the bearing
toward point[0] instead (the actual direction the robot will drive next
after wrapping around). Verified with a 4-point square: `loop=False`
gives the last point 180°/same as previous segment; `loop=True` correctly
gives -90° (bearing toward point 0) instead — a real, different value,
not just a no-op change. Also patched the user's already-picked
`maps/test_waypoints.yaml` in place (no re-clicking needed — positions
were correct, only `loop: false→true` and wp23's yaw needed recomputing)
and regenerated its overlay to visually confirm the loop closes cleanly.

**2. Added an `initial_x`/`initial_y`/`initial_yaw` hint, printed after
every run:** discovered while explaining "is wp1 (0,0,0) in the robot
frame" that `map_localizer`'s own relocalization initial pose (see
`trigger_map_relocalize.py`'s `initial_x`/`initial_y`/`initial_yaw`
parameters, wired from `localization.launch.py`'s launch args) defaults
to `(0.0, 0.0, 0.0)` — i.e. it assumes the robot starts at the MAP
FRAME's own origin, which is generally NOT the same point as wp1 (in the
real 23-point course, wp1 is at `(9.761, 5.698)`, ~11m from the map
origin). If the robot is physically placed at wp1 before start but this
override isn't passed, VGICP relocalization has to converge from a large,
wrong initial guess — risking a failed or wrong convergence rather than a
clean one. `pick_waypoints.py` now prints the exact, ready-to-copy
override matching wp1 after every run, so nobody has to compute or
copy these numbers by hand.

**Verified:** re-ran the full non-interactive regression suite (px/world
round-trip against real map data, heading computation incl. the new
loop-closing behavior, YAML schema round-trip, overlay rendering) — all
still pass after both changes. The new hint's printed values were checked
against the real `test_waypoints.yaml`'s wp1 entry and match exactly
(`x=9.7610, y=5.6980, yaw=-1.3259`).

**Docs updated:** `docs/custom_nav_stack_design.md` §5 (+ regenerated
`Custom_Nav_Stack_Design.pdf`, still 7 pages, new content visually
verified — the added note box spans a page break cleanly via ReportLab's
normal flow, no content lost) and the script's own module docstring.

## Reuse Plan Step 18 — test_step5_full_autonomy.sh: consolidate 5 terminals into 1

User asked how many terminals a full end-to-end autonomy test (localizer +
controller + waypoint sequencer) needs. Counted precisely: 4 long-running
foreground processes on the Jetson (`localization.launch.py`,
`pd_navigation.launch.py`, `cmd_vel_mux_node`,
`waypoint_sequencer.launch.py`) + 1 more for one-shot/verification
commands + 1 on the RPi for driveStack = 6 total. Confirmed this repo
already has an established pattern for exactly this problem
(`test_step3_fastlio.sh`/`test_step4_motion_plan.sh` background multiple
nodes with `&` + a `trap cleanup INT TERM` that does `kill 0`), so built
`scripts/test_step5_full_autonomy.sh` following that same convention —
reduces the Jetson side to 1 terminal (+ 1 still needed on the RPi for
driveStack, which stays independent per the Zenoh cmd_vel_safe design).

**Design decisions:**
- `initial_x`/`initial_y`/`initial_yaw` for `map_localizer`'s relocalization
  are auto-derived by reading wp1 directly out of `--waypoints-file` (same
  numbers `pick_waypoints.py`'s `print_initial_pose_hint()` already prints
  — just read programmatically here instead of requiring a copy-paste
  step) — overridable via `--initial-x/--initial-y/--initial-yaw` if the
  robot isn't actually placed at wp1.
- Pre-flight checks all three input files (`map_pcd_path`, `map_yaml`,
  `waypoints_file`) exist before launching anything, failing fast with a
  clear message rather than partially starting the stack.
- Does NOT auto-fire `/green_light` — that stays a deliberate, separate
  action (`scripts/trigger_green_light.sh`) taken only after manually
  confirming localization actually converged, so a bad relocalization
  can't silently start the robot moving.
- Does NOT launch any motor driver (unlike `test_step1`-`test_step4`,
  which use `diy_motor_control_legacy` locally) — driveStack stays on the
  RPi in its own terminal, independent, per the existing Zenoh-bridge
  `cmd_vel_safe` design.

**Verified (everything testable without real ROS hardware/build in this
sandbox):** bash syntax (`bash -n`), `shellcheck` (only info/style-level
findings, matching `test_step3_fastlio.sh`'s own identical "unused PID
variable" pattern — cleanup uses `kill 0`, not individual PIDs, by
design), the wp1-auto-derivation Python/bash logic against the REAL,
current `maps/test_waypoints.yaml` (confirmed it correctly picks up
whatever the file's current wp1 is — the file had actually changed to a
new 22-point set since the previous turn, and the script picked that up
correctly, not a stale cached value), the pre-flight file-check failing
correctly on a missing file and passing on real existing files, and the
CLI argument-parsing loop (flags + positional profile argument, plus the
unexpected-extra-argument error path).

**Not verified (needs real hardware/build, not possible in this
sandbox):** the actual `ros2 launch`/`ros2 run` invocations were not run
end-to-end — they are the exact same commands already given directly to
the user in this conversation and described as the correct invocations,
just consolidated into one backgrounded script now.

**Docs updated:** `docs/testing_guide.md` §4 — table header changed to
"`test_step1`-`test_step5`", new row added, caveat #1 note updated to
clarify `test_step5` doesn't use the legacy motor package at all (doesn't
launch any motor driver locally).

## Reuse Plan Step 19 — CRITICAL: global_map_2_smooth and refined_map.pcd are NOT the same coordinate frame

User asked how the robot knows its starting pose relative to the PGM's
top-left-corner-as-(0,0) convention, correctly pointing out the robot
obviously did not start recording at that corner. Investigating this
surfaced a serious, previously-undetected bug affecting every waypoint
picked so far.

**Root cause, confirmed with hard numbers, not assumption:**
`maps/global_map_2_smooth.yaml` has `origin: [0.0, 0.0, 0.0]`. This is
NOT a real, measured tie to any physical location — it is
`scripts/preprocess_map.py`'s own hardcoded fallback (`write_yaml()`)
used whenever no source `.yaml` is passed in. The user confirmed why: a
teammate handed over `global_map_2.pgm`/`global_map_3.pgm` directly, with
NO accompanying `.yaml` at all, so `preprocess_map.py` had nothing real
to inherit an origin from and silently defaulted to `[0,0,0]`.

Meanwhile `maps/refined_map.pcd` (what `map_localizer` actually loads for
its `map->odom` TF) has its own real coordinate frame, derived from
FAST-LIO2/LIO-SAM's actual odometry during recording — completely
unrelated to the PGM's assumed `[0,0,0]`.

**Verified this is a real, current problem, not a hypothetical:**
projected `refined_map.pcd` with `scripts/pcd_to_pgm.py` and checked
every waypoint in the user's actual `maps/test_waypoints.yaml` (picked
against `global_map_2_smooth`) against `refined_map.pcd`'s real bounding
box (x:[-6.15,4.82], y:[-6.75,7.63]) — **18 of 22 waypoints fall
completely outside the area the lidar ever scanned.** The
`initial_x/y/yaw` override set up two turns ago (matching wp1) was
therefore ALSO wrong — it would have told map_localizer's relocalization
to seed VGICP ~10+ metres from any real scan data.

**Investigated and ruled out a shortcut fix:** found that
`LIO_Localization/src/map_del/` (the teammate's own repo, outside
DIY-Challenge-Repo) contains BOTH `global_map.pgm`+`refined_map_1.pcd`
(byte-identical to this repo's `global_map_2.pgm`) AND
`global_map_1.pgm`+`refined_map.pcd` (byte-identical to this repo's
`global_map_3.pgm` and the currently-active `maps/refined_map.pcd`) —
i.e. confirmed real, same-mapping-run PAIRS exist. Tried swapping
`maps/refined_map.pcd` to be `refined_map_1.pcd` (the file that pairs
with `global_map_2.pgm`, the actual source of `global_map_2_smooth`)
expecting this would fix the frame mismatch — it did NOT: re-checked the
same 22 waypoints against `refined_map_1.pcd`'s real bounding box
(x:[-6.28,7.05], y:[-7.53,5.09]) and still found 14/22 out of bounds.
**Conclusion: even a genuinely paired PGM+PCD from the same mapping run
do not automatically share a coordinate origin** unless whatever tool
produced the PGM explicitly derived it from the point cloud (confirmed
`maps/global_map.yaml` — an older, different map family — DOES have a
real non-zero origin `[-5.25,-7.35,0.0]`, so this has been done
correctly before, just not for the `global_map_2`/`_3` family). Reverted
`maps/refined_map.pcd` back to its original file (matching
`global_map_3.pgm`) since the swap didn't help.

**Only reliable fix:** generate the 2D map DIRECTLY from the same `.pcd`
`map_localizer` will load, using `pcd_to_pgm.py` (which computes origin
from the point cloud's own real bounding box, not a placeholder). Did
this: `maps/course_from_pcd.pgm`/`.yaml` (z-slab -0.3 to 1.4m, resolution
0.05, `--no-smooth` mode). This is frame-correct by construction, but
visually rough — `refined_map.pcd` is genuinely sparse (12,685 points
total across the whole course), and no combination of
`--close-kernel`/`--keep-walls`/z-range tried (several combinations,
0.025 and 0.05 resolution, smooth and no-smooth modes) produced a fully
clean closed ring; real wall-point gaps remain even in the best result.
Confirmed a higher `--z-max` (tried 1.4/1.5/2.0/2.2) does NOT help —
occupied pixel count barely changes (1001->1005 px), meaning the extra
height range only catches a handful of stray points, not more real wall
structure — sparsity is the genuine limiting factor, not z-window choice.

User chose to proceed with the rough-but-frame-correct
`maps/course_from_pcd.pgm`/`.yaml` as-is rather than spend more time
tuning preprocessing or waiting for denser source data.

**Still open / next steps:**
- `maps/test_waypoints.yaml` needs to be re-picked entirely against
  `maps/course_from_pcd.yaml` (NOT `global_map_2_smooth.yaml`) — the
  current file is invalid (18/22 points out of the real scanned area).
- The `initial_x/y/yaw` override for `map_localizer`'s relocalization
  must be re-derived from the NEW wp1 once re-picked (same
  `pick_waypoints.py` auto-hint mechanism, just needs a fresh run).
- `docs/custom_nav_stack_design.md`/`docs/testing_guide.md` still
  reference `maps/global_map_2_smooth.yaml` as the example test map in
  several places — needs updating to `maps/course_from_pcd.yaml` (or
  whatever the team settles on) once this is finalized, to avoid the
  same mistake recurring.
- Longer-term: if a denser mapping pass becomes available (more lidar
  points, e.g. via LIO-SAM's loop-closure-refined map rather than raw
  FAST-LIO2 output), regenerating `course_from_pcd` from that would
  likely produce a visually cleaner map without changing this fix's
  approach.

## Reuse Plan Step 20 — pcd_to_pgm.py: point-splatting + outlier removal + region exclusion for clean walls from sparse scans

User asked whether the rough `course_from_pcd.pgm` walls could be
smoothed like the (frame-mismatched, not usable as-is) `global_map_3.pgm`
teammate map, and separately flagged a stray "antenna" spike protruding
from the top of the outer wall in the rendered overlay as noise that
should be removed.

**Investigated why the raw wall-selection/contour pipeline failed on
this sparse point cloud:** `refined_map.pcd` only has 12,685 total points
across the whole course. At native resolution, individual wall points
are frequently too far apart to touch as single pixels, so
`preprocess_map.py`'s connected-components step saw dozens of tiny
fragments (~1300px components) instead of one real wall, and no
`--close-kernel` value (tried 15-100) could bridge the real gaps without
either leaving fragments or over-merging unrelated structure.

**Root-fix added to `scripts/pcd_to_pgm.py` (new, reusable capability,
not a one-off hack for this map):**
1. **`--point-radius-px N`**: splats each occupied pixel into a filled
   disk of N pixels before writing the PGM — the same technique real
   occupancy-grid converters (octomap, PCL grid projection) use for
   sparse point clouds. Verified with a synthetic test (a line of points
   spaced wider than the raster resolution originally rasterized to 17
   isolated pixels; with `--point-radius-px 3` the same points produced
   513 connected occupied pixels — a continuous line). On the real data,
   `--point-radius-px 3` grew wall components from ~1300px to
   96000-150000px — genuinely continuous walls, verified visually
   (closed ring, both outer and inner boundaries solid).
2. **`--outlier-min-neighbors N` / `--outlier-radius-factor`**:
   statistical outlier removal (same concept as PCL's
   StatisticalOutlierRemoval) — drops points whose 2D neighborhood (via
   `scipy.spatial.cKDTree`) is too sparse to be a real densely-scanned
   wall, applied BEFORE rasterizing/splatting (so splatting can't weld an
   isolated noise point onto the real wall the way it would if outlier
   removal ran after). Verified on real data: of 7854 slab points, 87
   have <=3 neighbors within 0.15m (3x resolution) vs. 7316 with >10 —
   a clear, real bimodal separation between isolated noise and dense wall
   structure. Verified with a synthetic dense-wall + isolated-3-point-
   cluster test: outlier removal correctly dropped the isolated cluster,
   kept the dense wall.
3. **`--exclude-region X_MIN,X_MAX,Y_MIN,Y_MAX`** (repeatable): manually
   drops every point inside a given world-space x/y box before
   rasterizing. Added specifically because the user-flagged spike turned
   out NOT to be simple isolated noise — investigated its actual point
   distribution and found ~296-337 points in a locally coherent (not
   isolated) cluster around x=[-2.2,1.2] y=[2.2,3.63], meaning neither
   `--outlier-min-neighbors` (tried 3/5/8 — no visible change, since the
   cluster is locally dense enough to pass any reasonable neighbor
   threshold) nor morphological opening (tried k=15/25 on the pre-contour
   filled mask — either left the spike mostly intact or, at a strong
   enough kernel to remove it, also fragmented the real wall, since the
   spike and the real wall are similar thickness) could tell it apart
   from real structure automatically. Manual region exclusion was the
   only approach that worked cleanly, verified before/after
   side-by-side: the long finger-like protrusion visible in the original
   render is now a small, much-reduced bump.

**Not fully resolved:** a small residual bump remains at the excluded
region's boundary (visible in the final `maps/test_waypoints_preview.png`
overlay near the top of the outer wall) — likely a contour-smoothing
edge effect where the algorithm draws through the now-sparser boundary
of the excluded zone. Cosmetic only; does not affect any of the 23
waypoints (re-verified in-bounds and in free space against the final
map after every iteration in this step).

**Final map**: `maps/course_from_pcd_smooth.pgm`/`.yaml`
(resolution 0.025, origin `[-5.6551, -6.9249, 0.0]` — same origin
throughout every variant tried in this step, confirming none of
splatting/outlier-removal/region-exclusion disturbed frame correctness).
Generated via:
```
python3 scripts/pcd_to_pgm.py --input maps/refined_map.pcd \
    --output /tmp/final2 --z-min -0.3 --z-max 1.4 --resolution 0.05 \
    --point-radius-px 3 --exclude-region="-2.5,1.5,1.6,10.0"
python3 scripts/preprocess_map.py --input /tmp/final2.pgm \
    --yaml /tmp/final2.yaml --close-kernel 5 --keep-walls 2 --no-display
```
All 23 existing waypoints in `maps/test_waypoints.yaml` re-verified
in-bounds and in free space against this final map (0 out of bounds, 0
on occupied/unknown) — no re-picking needed, only the map's wall
rendering changed, not its coordinate frame.

**Still open:**
- The residual small bump at the exclusion boundary could likely be
  removed with a wider/differently-shaped exclusion region, or by
  accepting it as cosmetic (current recommendation, given the boundary
  doesn't intersect any real waypoint or corridor path).
- These new `pcd_to_pgm.py` flags are generically reusable for any future
  sparse-scan map, not tied to this specific course/point cloud — worth
  documenting in `docs/testing_guide.md`'s map-generation section as the
  new recommended workflow for sparse LIO2/LIO-SAM point clouds.

## Reuse Plan Step 21 — preprocess_map.py: mark enclosed inner-ring interior unavailable

User pointed out the inner ring's interior was still shown as free (white)
in `course_from_pcd_smooth.pgm`, even though it's fully enclosed by a
wall and genuinely unreachable — should be unavailable, same treatment
as the exterior outside the outer wall.

**Root cause:** `preprocess_map.py`'s `_flood_and_assemble()` only
flood-fills from the 4 IMAGE BORDERS inward to find the true exterior.
Since both the real corridor AND the inner ring's interior are fully
enclosed by walls, NEITHER touches the image border — the border flood
fill correctly marks the true exterior unavailable, but has no way to
also reach and mark the separately-enclosed interior island.

**Fix:** added `isolate_largest_free` parameter to `_flood_and_assemble()`
(wired through `smooth_pipeline()`/`simple_pipeline()`/new CLI flag
`--isolate-largest-free`, default OFF). After the usual border flood
fill, if enabled, runs connected-components on the remaining free-space
regions and keeps only the SINGLE LARGEST one as free — any other
enclosed free blob (the inner-ring interior) gets marked
unavailable/unknown too. Deliberately opt-in, not the new default: this
technique can't distinguish "unreachable enclosed island" from "a second
genuinely separate navigable room" — it just assumes the largest
connected free blob is the only real one, which is correct for a single
closed-loop/corridor course but would silently break a map with multiple
legitimately separate rooms.

**Verified with a synthetic test** (two square wall outlines: an outer
boundary + an inner island, matching the real course's topology) before
touching real data: confirmed the old behavior incorrectly left the
island's interior FREE; the new `isolate_largest_free=True` correctly
marks it UNKNOWN while the real corridor between the two walls stays
FREE and the true exterior stays UNKNOWN in both cases.

**Applied to the real course map**: regenerated
`maps/course_from_pcd_smooth.pgm`/`.yaml` with `--isolate-largest-free`
added to the existing recipe (same z-slab/point-radius/exclude-region
from Step 20) — log confirmed a 151,776 px enclosed blob (the inner ring
interior) was correctly marked unavailable; verified visually (both
exterior and inner-ring interior now consistently grey, only the
corridor between them white). Origin unchanged (`[-5.6551, -6.9249]`) —
confirms this didn't disturb frame correctness. Re-verified all 26
waypoints in `maps/test_waypoints.yaml` still in-bounds and in free space
against the updated map (0 out of bounds, 0 on occupied/unknown) — no
re-picking needed, since the change only affects space the waypoints
were never placed in anyway.

## Reuse Plan Step 22 — new draw_map_borders.py: manual wall tracing for real scan gaps

User asked for an interactive tool to open the raw PGM (from `pcd_to_pgm.py`)
and hand-draw the outer/inner wall borders, explicitly to solve the real
gap found in Step 20's `refined_map_1.pcd` generalization test — a genuine
scan gap (zero points in one stretch) that no `--close-kernel`/
`--inner-close-kernel` value could close, since those tools can only
bridge gaps BETWEEN existing points, not invent data that was never
scanned.

**New tool: `scripts/draw_map_borders.py`** — same interactive-picker
pattern as `pick_waypoints.py`/`register_zones_to_map.py` (matplotlib
click handling, explicit backend fallback for the known `QT_API`
misconfiguration bug), purpose-built for manual wall tracing:
- Shows the raw/messy PGM dimmed as a background reference so the user
  traces directly over real scan data.
- `'o'` starts/restarts the OUTER wall trace; `'i'` finishes the current
  polygon and starts a new INNER obstacle (repeatable for multiple
  separate inner obstacles); `'u'` undoes the last point; closing the
  window finalizes and rasterizes.
- Traced polygons are resampled to evenly-spaced points then
  Gaussian-smoothed in periodic (wrap) mode — same smoothing technique
  `preprocess_map.py` already uses on its own auto-extracted contours,
  so hand-clicked points don't look like jagged straight-line segments.
- Rasterization: FREE only inside the outer polygon AND outside every
  inner polygon; UNKNOWN everywhere else (true exterior AND every inner
  obstacle's interior, consistent treatment — same idea as Step 21's
  `--isolate-largest-free`, but driven by an exact hand-drawn shape
  instead of an automatic largest-blob heuristic, so it works even when
  the automatic wall detection can't close a real gap).
- Output PGM/YAML preserves the INPUT map's resolution/origin exactly
  (not re-derived) — this only re-draws walls on an already-correct
  frame, it does not touch frame correctness at all.

**Verified (everything testable without a live GUI in this sandbox):**
- `resample_and_smooth()`: a synthetic square resamples to the requested
  point count; smoothing measurably rounds a corner (moves it away from
  the exact clicked corner point) vs. no smoothing, confirming the
  Gaussian pass has a real, visible effect.
- `rasterize()`: a synthetic outer square + inner square obstacle
  correctly classifies exterior=UNKNOWN, corridor=FREE,
  inner-obstacle-interior=UNKNOWN, and the traced boundary itself=WALL.
  Re-verified the zero-inner-polygon case (a course with only an outer
  wall, no obstacle) correctly leaves the whole interior FREE.
- `write_output()`: confirmed the output YAML's resolution/origin exactly
  match the input meta dict, byte-for-byte on the printed values — not
  just "close enough."
- `draw_overlay()`: renders without error on real PGM dimensions.
- **End-to-end proof against the actual real gap found in Step 20**:
  loaded the real raw projection of `refined_map_1.pcd`
  (`pcd_to_pgm.py --z-min -0.3 --z-max 1.2 --point-radius-px 3`),
  simulated a realistic hand-trace (a plausible sequence of click points
  a person would place, including bridging the exact real gap region
  identified in Step 20's point-density check) through `rasterize()`
  directly (not the live GUI, which can't be exercised headlessly) —
  produced a fully closed, clean ring with both the exterior and the
  inner obstacle's interior correctly marked unavailable, and the
  corridor correctly free — the exact result the automatic pipeline
  could NOT produce for this file no matter which parameters were tried.

**Not verified (inherent to being an interactive GUI tool, same caveat as
`pick_waypoints.py`/`register_zones_to_map.py --pick`):** the actual
mouse-click event loop and key-binding handlers were not exercised
end-to-end in this sandbox (no display) — every function they call was
tested directly instead.

**Docs updated:** `docs/testing_guide.md` §9 — new table row, positioned
as the manual alternative to `preprocess_map.py` specifically for when
automatic wall detection can't cleanly close a real scan gap.

## Reuse Plan Step 23 — draw_map_borders.py: save/reload traced points + real-world wall thickness

Two real usability gaps found while iterating with the user on
`draw_map_borders.py`:

**1. `cv2.polylines(thickness=N)` does not draw exactly N pixels wide** —
confirmed empirically (requesting 3 draws 5px; requesting 8 draws 9px;
the relationship isn't a clean formula). This meant a user-requested
"~5 inch wall" via a naive pixel-count guess came out ~2x too thick.
Fixed by adding `--wall-thickness-in`/`--wall-thickness-m`: these
empirically calibrate the real drawn width AT RUNTIME (draw real test
lines at increasing requested thicknesses on a throwaway canvas, measure
the actual result, pick the closest match) rather than trusting a
hardcoded formula that could be wrong on a different OpenCV build.
Verified: targeting 5in at 0.05 m/px resolution correctly resolves to
cv2 thickness=2, which empirically draws 3px = 5.91in (the closest
achievable value — pixel granularity at this resolution limits
precision to ~2in steps, can't hit exactly 5.00in without a
higher-resolution source map).

Also re-measured `preprocess_map.py`'s (the automatic pipeline)
already-generated `course_from_pcd_smooth.pgm` for comparison: its
default `--wall-thickness=8` (a DIFFERENT script with different internal
4x-scale-then-2x-downsample math, not directly comparable to
`draw_map_borders.py`'s parameter) produces an actual real-world
thickness of ~4.3-4.5 inches, measured the same way (distance-transform
ridge method).

**2. User asked why they had to re-trace every time** — a completely
valid complaint; the tool had no way to persist a trace. Added:
- `--points-file PATH` (default `<output>_borders.yaml`): saves the
  traced outer/inner polygon pixel coordinates + the source image shape
  after every trace.
- On a subsequent run, if this file already exists, it's loaded
  automatically and the interactive tracer is SKIPPED ENTIRELY — trace
  once per input map, then freely re-run with different
  `--wall-thickness-in`/`--smooth-sigma`/etc. with zero re-tracing.
- `--retrace` forces the interactive tracer to open again (overwriting
  the saved file) if the user genuinely wants to redraw.
- If the saved points' recorded image shape doesn't match the current
  input PGM's shape, prints a clear warning (pixel coordinates are only
  meaningful for the exact image they were traced against) but doesn't
  hard-fail, since the user may know what they're doing.

**Verified:**
- `resolve_wall_thickness_px()`: tested at two different resolutions
  (0.05 and 0.025 m/px) for the same 5in target, confirming it correctly
  adapts the resolved cv2 parameter per-resolution rather than using a
  fixed value.
- Mutual-exclusivity CLI validation: `--wall-thickness` +
  `--wall-thickness-in` together correctly errors (exit code 2) rather
  than silently picking one.
- `save_points_yaml()`/`load_points_yaml()`: exact round-trip verified
  (saved then reloaded points match byte-for-byte); shape-mismatch case
  verified to warn but not crash.
- **Full end-to-end simulation of the real two-run workflow**: saved a
  realistic trace (matching `course_from_pcd.pgm`'s actual dimensions),
  then simulated exactly what a second script invocation's `main()`
  would do (load from the points file, skip `interactive_draw()`
  entirely) — produced a correct, clean rasterized map purely from the
  saved file, confirming the "trace once, reuse forever" behavior
  actually works end-to-end, not just at the unit level.

**Docs:** module docstring's ARGUMENTS AT A GLANCE section updated with
all new flags.

## Reuse Plan Step 24 — real-world wall thickness for preprocess_map.py + optional pipeline wrapper

Two follow-ups to streamline the pcd->final-map workflow:

**1. Added `--wall-thickness-in`/`--wall-thickness-m` to `preprocess_map.py`**
(mirroring `draw_map_borders.py`'s identical fix from Step 23) —
`smooth_pipeline()`'s raw `--wall-thickness` parameter draws at an
internal 4x-upscaled canvas then downsamples 2x, so it never mapped
1:1 to real-world size; empirically calibrated at runtime instead
(replicate just the draw-at-4x + downsample-2x steps on a synthetic
line, measure the real result, pick the closest match) — verified
against a real ground-truth data point (measured, via a genuine
straight 18-column wall segment on `course_hand_drawn_smooth.pgm`: the
default `--wall-thickness=8` produces 5px final = 4.92in, not the
stale docstring's claimed "~0.05m"): the new calibration function
correctly resolves a 4.92in target to `--wall-thickness=7`, landing on
the identical actual 5px/4.92in result (off by 1 in the underlying
parameter vs. the empirical ground truth, attributable to the
calibration deliberately skipping blur/close-kernel steps that add
marginal extra thickness in the full real pipeline — functionally
accurate for its purpose).

**2. Found and fixed a real, pre-existing, unrelated bug while touching
this code**: `smooth_pipeline`'s output resolution was HARDCODED to
`0.025` in `main()`, not computed from the actual input resolution —
this only ever happened to be correct in all of today's testing because
every map used exactly `0.05` m/px input by coincidence. Fixed to
`input_resolution / 2.0` (the real, always-true relationship given
`smooth_pipeline`'s fixed 4x-upscale/2x-downsample math). Verified the
fix is genuinely active, not still coincidentally masked: regenerated a
map from `refined_map.pcd` at a deliberately different resolution
(`0.08` m/px input) and confirmed the output yaml correctly reports
`0.04` (not the old hardcoded `0.025`).

**3. New optional convenience script: `scripts/build_course_map.sh`** —
chains `pcd_to_pgm.py` -> `draw_map_borders.py` -> `preprocess_map.py`
into one command with the defaults validated across this session
(`--point-radius-px 3`, `--keep-walls 2`, `--isolate-largest-free`,
`--wall-thickness-in 5`). Explicitly presented to the user as OPTIONAL —
they confirmed they're fine running the 3 steps manually and asked not
to over-engineer this; built as a lightweight best-effort convenience,
not a hard requirement. `--preview-only` runs just the z-histogram step
(picking a z-slab still requires human judgment, can't be automated).
Re-running with the same `--output` prefix auto-reuses any previously
saved trace (via `draw_map_borders.py`'s existing points-file feature),
so re-tuning `--wall-thickness-in`/`--keep-walls` afterward needs no
re-tracing. Verified: bash syntax check, missing-required-arg error
path, and the `--preview-only` path all work correctly.

**Docs:** `preprocess_map.py`'s `--wall-thickness` help text corrected
(was a stale/wrong "~0.05m" claim); `--isolate-largest-free`-related tips
already documented in Step 21 remain accurate.

## Reuse Plan Step 25 — RESOLVED: real lidar topic is `/lidar_points`, not `/hesai/points`

**Context:** Several files (comments and, in a couple of cases, actual
functional parameters) assumed the raw Hesai QT64 driver publishes on
`/hesai/points`. This was flagged as an open item as far back as Step
4-ish of this log, since the *actual* running driver's topic name had
never been confirmed against real hardware. User has now confirmed
directly: **the real topic is `/lidar_points`**.

**What was already correct (verified via grep before touching anything):**
`fast_lio_hesai_qt64.yaml`'s functional `common.lid_topic` and
`nav2_params.yaml`'s functional `topic:` values were already
`/lidar_points` — likely fixed opportunistically in an earlier segment
without the header comments/other files being updated to match.

**What was wrong and got fixed (functional, not just cosmetic):**
- `src/challenge_bringup/config/collision_monitor_params.yaml` —
  `topic:`/`observation_sources:` actually said `/hesai/points`. Fixed.
  (Currently dormant/unlaunched per user: local costmap + collision
  avoidance are not implemented yet — only controller + localizer are
  being tested right now — but this is still a real latent bug for
  whenever that stack is turned on.)
- `scripts/health_check.sh` — lidar-publishing health check subscribed
  to the wrong topic, meaning a health-check run before today would
  have falsely reported the lidar as dead. Fixed.
- `scripts/record_bag.sh` — bag-recording topic list would have
  silently recorded nothing for lidar (no error, just an empty/missing
  topic in the bag). Fixed.
- `scripts/calibrate_extrinsics.sh` — prerequisite comment + 2
  functional occurrences (used for cam-lidar calibration data capture).
  Fixed.
- `scripts/calibrate_cam_lidar.sh` — prerequisite comment, `ros2 topic
  hz` health check, and `ros2 bag record` topic list (3 occurrences).
  Fixed.

**Cosmetic-only fixes** (comments/docstrings/echo statements, no
functional effect either way, fixed for accuracy anyway):
`src/diy_localization/config/fast_lio_hesai_qt64.yaml` header comment
(open item rewritten as RESOLVED with today's date),
`src/challenge_bringup/launch/challenge_master.launch.py` BLOCK 6
comment, `src/diy_localization/launch/localization.launch.py` and
`offline_mapping.launch.py` docstring data-flow diagrams,
`scripts/test_step3_fastlio.sh` echo statement.

**Deliberately left alone:** `src/diy_ndt_localization/*` — confirmed
via grep that this package is already deprecated (replaced by
`map_localizer`, per comments in `localization.launch.py`) and not
referenced by any active launch file; not worth fixing a dead package.
`docs/generate_*.py` PDF-generator scripts were swept and found to
contain no `/hesai/points` references needing correction in the current
docs set (already-generated PDFs were not touched here since no source
`.md` content referenced the stale topic).

**Verification:** `bash -n` on every edited shell script, `python3 -c
"import ast; ast.parse(...)"` on every edited `.py` launch file,
`yaml.safe_load` on every edited `.yaml` file — all pass. Final `grep
-rn "hesai/points"` across the whole repo (excluding
`diy_ndt_localization`, `docs/generate_*.py`, and vendored/build dirs)
returns only the 3 intentional "this used to be assumed /hesai/points"
historical-note comments left in place for context — no remaining
functional or accidental references.

**Still open (unchanged by this step):** whether `extrinsic_R` in
`fast_lio_hesai_qt64.yaml` is correct is still not 100% proven (see
Step prior discussion — strong circumstantial timestamp evidence, not
a live accelerometer re-check). None of today's fixes touch that value.

**Deferred by user request:** 6 more `docs/generate_*.py` PDF
generators (`generate_arena_debug_guide.py`,
`generate_camera_guide.py`, `generate_competition_playbook.py`,
`generate_docs.py` (SAD), `generate_mapping_guide.py`,
`generate_nav_design.py`) still contain numerous `/hesai/points`
references in body text and some diagram graphics (matplotlib/graphviz
labels) — purely cosmetic/documentation, not used by anything run on
the robot. User explicitly chose to leave these for later since they
don't affect robot testing. Revisit before finalizing docs for
competition handoff.
