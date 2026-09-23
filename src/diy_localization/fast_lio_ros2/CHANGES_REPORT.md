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
