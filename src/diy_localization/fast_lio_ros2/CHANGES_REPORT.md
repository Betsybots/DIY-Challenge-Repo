# fast_lio_ros2 changes — localization robustness session

**Status**: built and verified with `colcon build --symlink-install --packages-select fast_lio_ros2` on this machine (Ubuntu/ROS 2 Humble). Not yet validated against real hardware logs — the shock/gyro thresholds below are reasoned defaults, not field-tuned.

This report covers changes made to the **active** `fast_lio_ros2` package during a session focused on hardening the FAST-LIO2 + `map_localizer` localization stack (which replaces AMCL for `map -> odom` correction while FAST-LIO2 owns `odom -> base_link`).

---

## Background: what this package is for

FAST-LIO2 (`laserMapping.cpp`) fuses IMU + LiDAR to publish high-rate `odom -> base_link` odometry and a `/cloud_registered_body` point cloud. `map_localizer` (separate package, see its own `CHANGES_REPORT.md`) periodically registers that cloud against a saved map to correct `map -> odom`. Both run simultaneously so the robot has a continuously-updated pose even while moving fast or between map corrections.

---

## 1. Shock-rollback fix (`laserMapping.cpp`)

**Problem**: this fork already had a scan-quality gate (`reject_low_quality_scans`) that flags a scan as unreliable during/just after an IMU "shock" (bump, low feature count, high residual, IMU gap). But the gate only kept a bad scan **out of the persistent map** — the LiDAR correction from that same bad scan was **still published as real odometry/TF**, because `publish_odometry()` ran unconditionally regardless of the gate's verdict. A shock-damaged correction could still slip into `odom -> base_link` and appear as a sudden pose jump/teleport downstream (RViz, Nav2, `map_localizer`, the motion controller).

**Fix**: in the main processing loop, right after IMU-only forward propagation (`p_imu->Process(...)`) but **before** the LiDAR measurement update is applied, the filter's state and covariance are snapshotted:

```cpp
state_ikfom predicted_state_for_rollback = state_point;
esekfom::esekf<state_ikfom, 12, input_ikfom>::cov predicted_cov_for_rollback = kf.get_P();
```

If the scan is later flagged `scan_is_low_quality`, the filter is rolled back to that pre-update snapshot via the existing IKFoM `change_x()`/`change_P()` API before `publish_odometry()` runs, so a low-quality scan now publishes the safe, IMU-only **predicted** pose instead of an already-flagged-unreliable LiDAR correction.

- **Function changed**: the scan-processing lambda inside `LaserMappingNode`'s main loop in `src/laserMapping.cpp` (around the `scan_is_low_quality` block).
- **No new config** — reuses the existing `frontend.reject_low_quality_scans` gate and its thresholds.

---

## 2. Gyro-rate shock criterion (`IMU_Processing.hpp` + `laserMapping.cpp`)

**Problem**: the existing shock detector (`ImuProcess::hadShock()`) only looks at **linear acceleration** deviation from gravity — it catches bumps, but a **fast rotation with little net linear acceleration change** (e.g. an in-place spin) was never flagged at all, even though that's exactly the kind of motion most likely to destabilize a LiDAR-inertial filter.

**Fix**: added an independent, OR'd trigger based on angular velocity magnitude.

- `ImuProcess::set_shock_detection(...)` gained a 4th parameter, `gyro_threshold` (rad/s).
- Each IMU interval now computes `gyro_shock_this_step = angvel_avr.norm() > shock_gyro_threshold_`, in addition to the existing `accel_shock_this_step`. `shock_this_step = accel_shock_this_step || gyro_shock_this_step` — either one inflates that predict step's process noise and marks the scan as shock-affected, feeding into the rollback fix above.
- Added separate diagnostic flags so logs can say *why* a scan was flagged: `hadAccelShock()` / `hadGyroShock()`, in addition to the existing combined `hadShock()`.
- New parameter: `frontend.shock_gyro_threshold` (default **2.5 rad/s**). Chosen above `motion_controller`'s `max_angular_velocity` default (1.0 rad/s) so normal commanded in-place turns don't false-positive.
- The "Rejecting low-quality scan" `RCLCPP_WARN` in `laserMapping.cpp` now prints `shock=%d(accel=%d gyro=%d)` instead of just a combined flag.

**Functions/files changed**:
- `IMU_Processing.hpp`: `ImuProcess::set_shock_detection()` signature, `UndistortPcl()`'s per-interval shock computation, new members `shock_gyro_threshold_`, `last_had_accel_shock_`, `last_had_gyro_shock_`.
- `laserMapping.cpp`: new global `shock_gyro_threshold`, its `declare_parameter`/`get_parameter_or` calls, updated `p_imu->set_shock_detection(...)` call, updated warning log.

**Known limitation** (documented, not fixed here): this is still a single fixed threshold, not a rigorous geometric degeneracy check (see `map_localizer`'s report, item C.7) — a real gyro-based test on hardware is needed to confirm 2.5 rad/s is the right cutoff for this robot.

---

## Config surface added (`config/qt64.yaml`)

```yaml
frontend:
    shock_gyro_threshold: 2.5   # rad/s -- see reasoning above
```

## What was intentionally *not* touched

Everything else already active in this package (the `Log/` directory crash-guard, `publish_tf` toggle, extrinsic-calibration debug print, `/Odometry -> /odom` remap, tuned `extrinsic_T`) was left as-is — see the workspace session history for why the `referance_fast_lio_ros2` snapshot (an older branch with an isolated fix bolted on) was *not* merged wholesale.

---

# Session 2: robustness + multithreading + dead-code removal (QT64 @ 32 lines, 3-5 m)

**Status**: built and verified with `colcon build --symlink-install --packages-select fast_lio_ros2` (ROS 2 Humble). Config values are reasoned from FAST-LIO upstream defaults and community short-range-indoor guidance; validate on hardware.

## 1. Removed the `filtered_cloud_` pipeline entirely

It was disabled in `qt64.yaml`, nothing in the repo subscribed to `/cloud_registered_body_filtered`, and when enabled it ran a **second full preprocessing pass and a second full undistortion pass per scan** — pure overhead. Deleted:

- `laserMapping.cpp`: all `filtered_cloud_*` globals/parameters, `keep_filtered_body_point()`, `publish_frame_body_filtered()`, the duplicate `p_pre_filtered` preprocessor, `lidar_buffer_filtered`, `feats_undistort_filtered`, and the `/cloud_registered_body_filtered` publisher.
- `IMU_Processing.hpp`: the optional filtered-cloud copy/sort/undistort path in `Process()`/`UndistortPcl()`.
- `common_lib.h`: `MeasureGroup::lidar_filtered`.
- `qt64.yaml`: `filtered_cloud_pub_en` and the commented-out `filtered_cloud_*` keys.

## 2. Multithreading / speedup

- **CMakeLists.txt**: OpenMP parallel sections were x86-only and hard-capped at 3 threads — ARM boards (Jetson) ran the scan-to-map matching fully single-threaded. Now enabled on every architecture with >3 cores, using `min(cores - 2, 6)` threads (2 cores reserved for the ROS executor threads).
- **`map_incremental()`**: restructured into a parallel phase (world transform + per-point add/downsample decision, independent per index) and a short serial collection phase. This was a fully serial per-scan hot loop.
- **`publish_frame_world()` / `publish_frame_body()` / PCD accumulation loop**: per-point transforms parallelized (`#pragma omp parallel for`, independent index writes).
- **Executor**: `MultiThreadedExecutor` bumped 2 → 3 threads so the heavy scan-processing timer plus *both* sensor callbacks (reentrant group) can run concurrently; with 2 threads a long scan update left only one thread for LiDAR *and* IMU callbacks.
- Removed the dead `Python.h`/matplotlibcpp/PythonLibs dependency (unused since the plotting code was removed upstream) — faster builds, fewer deps. Deleted unused `include/matplotlibcpp.h` and `include/Exp_mat.h`.

## 3. Config tuning (`qt64.yaml`) — the actual "flying odometry" fixes

| Param | Before | After | Why |
|---|---|---|---|
| `acc_cov` / `gyr_cov` | 0.001 | **0.1** | 100x over-trust of the consumer-grade ZED2i MEMS IMU; the standard FAST-LIO value is 0.1 and for noisy IMUs you only ever *raise* it. Over-trusting IMU propagation is a textbook cause of odometry "flying away" and slow LiDAR correction. |
| `reject_low_quality_scans` | false | **true** | The entire scan-quality gate + shock rollback built in session 1 was switched off, so shock-corrupted scans went straight into odometry/TF and the map. |
| `max_mean_residual` | 0.1 | **0.3** | 0.1 rejected healthy scans during normal motion (map starvation); 0.3 still catches mis-registrations. |
| `max_iteration` | 7 | **5** | Own measurement note said "5 and 10 working good/best"; 7 only added latency. |
| `cube_side_length` | 500 | **100** | Short-range indoor robot; smaller ikd-tree working volume. |
| `det_range` | 10 | **5** | Matches the application's stated 3-5 m useful detection range. |

## 4. Known remaining limitation (by design)

FAST-LIO2 is **odometry only — it has no loop closure**, so a long loop will never exactly re-close by itself; that is what `loop_pgo` / `map_localizer` in this workspace are for. If drift stays too high, the research survey (see session notes) recommends DLIO (`vectr-ucla/direct_lidar_inertial_odometry`, `feature/ros2` branch) or Point-LIO (`dfloreaa/point_lio_ros2`) as the most robust drop-in alternatives for an unsynced external IMU.

---

# Session 3: jump-gated rollback + rollback cap + degeneracy detection

**Status**: built with `colcon build --packages-select fast_lio_ros2` (Humble). Thresholds are reasoned defaults — field-tune `max_pose_jump`/`max_rot_jump` against real logs.

## Why odometry was still "bad with LIO" (root cause found in the recovery design)

The session-1 gate rolled the filter back to the **IMU-only predicted pose for ANY suspect scan** (shock, cooldown, low features, high residual, IMU gap), with **no cap**. On a consumer MEMS IMU (ZED2i) IMU-only dead reckoning diverges within 1-2 scan periods, so:

> reject scan → pose drifts on IMU → next scan's residual is even higher → rejected too → … → odometry flies away permanently.

A single bump or brief occlusion could trigger this death spiral — exactly the observed "easily losing odometry / TF flying over the map". The shock_cooldown made it worse by forcing ≥3 consecutive IMU-only cycles at 10 Hz.

## Fix 1: jump-gated rollback (`laserMapping.cpp`)

The pose decision now looks at the **magnitude of the LiDAR correction itself** (updated state vs. IMU-predicted prior, position and `Log(R_predᵀ R_upd)` rotation):

- *Suspect scan, sane correction* → **keep** the LiDAR update (held out of the map only). A slightly distorted correction is far better than open-loop IMU.
- *Implausible jump* (`> max_pose_jump` 0.10 m or `> max_rot_jump` 0.15 rad per scan) → roll back as before.

## Fix 2: rollback cap (`frontend.max_consecutive_rollbacks`, default 3)

After N consecutive rollbacks the (large) LiDAR correction is accepted anyway: at that point the *prior* is what's wrong, and snapping back to the map is the only way to re-converge. This bounds worst-case open-loop time to N scan periods and makes the recovery mode actually recover.

## Fix 3: degeneracy / localizability detection (Zhang & Singh, ICRA 2016)

Per scan, the smallest eigenvalue of the normalized plane-normal Gramian `Σ n nᵀ / N` (translation block of the point-to-plane GN Hessian) is computed from the effective correspondences. Below `frontend.degeneracy_min_eig` (default 0.05) the scan does not constrain translation along that eigenvector (corridor / dominant single plane) and a throttled warning prints the weak direction; `frontend.degeneracy_hold_map: true` additionally keeps such scans out of the map so the sliding direction doesn't get smeared into it. Same approach as FASTLIO2-DCReg (github.com/Shidabot/FASTLIO2-DCReg).

The rejection log now prints `correction(pos=…m rot=…rad) jump=… min_eig=…` and whether the scan was rolled back or kept, so field logs can directly tune the thresholds.

## Config surface added (`qt64.yaml`)

```yaml
frontend:
    max_pose_jump: 0.10
    max_rot_jump: 0.15
    max_consecutive_rollbacks: 3
    degeneracy_min_eig: 0.05
    degeneracy_hold_map: false
```

## Note on "loop doesn't re-close"

Pure LIO drift means a loop never re-closes exactly by itself — that correction belongs to `loop_pgo`/`map_localizer` (map→odom). These fixes reduce the odom-frame drift those layers must absorb; they don't eliminate it.

---

# Session 4: lio_ekf.yaml restructured LIO-primary (outdoor arena)

Review found the old file made ZED visual odometry the ungated absolute-pose backbone (lever-arm error + relocalization teleports went straight into odom->base_footprint) while demoting FAST-LIO2 to differential x/y behind a **dead gate** — robot_localization ignores `pose_rejection_threshold` when `differential: true` (only the twist threshold applies), so the documented "very tight gate (1.0)" never existed.

New structure (`config/lio_ekf.yaml`, launch docstring updated to match):

| Input | Topic | Fused | Mode / gate |
|---|---|---|---|
| odom0 | `/Odometry` (FAST-LIO2) | x, y, yaw | absolute, ungated (LIO self-gates via qt64.yaml `frontend:`) |
| odom1 | `/zed/zed_node/odom` | x, y | **differential**, twist gate 2.0 |
| odom2 | `/wheel_odom` | vx, vy | twist gate 2.5 |
| imu0 | ZED IMU | vyaw only | twist gate 3.0 (absolute yaw dropped — ZED-frame, fights LIO/VO) |

Rationale: FAST-LIO2 (hardened in sessions 2-3) is the only sensor whose pose is expressed directly in the odom frame against the LiDAR map; VO/wheel/gyro now stabilize it as increments and bridge LIO-degenerate stretches (open outdoor ground with few returns inside the QT64's used 3-5 m range). Also fixed the stale "not wired into any launch file" header (launch/lio_ekf.launch.py exists).
