# laserMapping.cpp changes — investigation report

**Branch**: `navigation_pipeline_develop` (local only — not committed, not pushed)
**File changed**: `src/fast_lio_ros2/src/laserMapping.cpp`
**Status**: NOT YET BUILT OR TESTED — this Windows environment has no ROS 2 /
PCL toolchain, so this change has only been manually reviewed (brace-balance
checked, types/APIs cross-checked against `esekfom.hpp`). Build with `colcon
build` on the robot/dev Linux machine and test before trusting this.

---

## 1. Context

This is part of the same "robot loses localization / appears to fly in
RViz" investigation covered in `src/map_localizer/CHANGES_REPORT.md`. That
report covers a separate, already-implemented fix in `map_localizer`
(`map → odom`). This report covers a second, independent issue found in
FAST-LIO2 itself (`odom → base_link`), agreed with the user to implement
without first confirming it on hardware logs (a faster-but-higher-trust
path than the log-first approach used for `map_localizer`).

## 2. Root cause found

This fork already has a scan-quality gate (`reject_low_quality_scans`) that
flags a scan as unreliable when it's captured during/just after an IMU
"shock" (a sudden linear-acceleration deviation — e.g. a bump), or has too
few reliable point-to-plane correspondences, or too high a residual, or too
large a gap in IMU coverage:

```cpp
const bool scan_is_low_quality = reject_low_quality_scans &&
    (effct_feat_num < min_effective_features ||
     res_mean_last > max_mean_residual ||
     p_imu->hadShock() ||
     in_shock_cooldown ||
     p_imu->getMaxImuGap() > max_imu_gap);

if (scan_is_low_quality) { /* ...log it... */ }

/******* Publish odometry *******/
publish_odometry(pubOdomAftMapped_, tf_broadcaster_);   // <- always runs!

...
if (!scan_is_low_quality) map_incremental();            // <- only THIS was gated
```

**The measurement update (`kf.update_iterated_dyn_share_modified(...)`) had
already been applied to the filter, and its result was always published as
real odometry/TF — regardless of `scan_is_low_quality`.** The gate only kept
a bad scan out of the persistent ikd-tree map and off the published point
clouds. The pose actually handed to RViz, Nav2, `map_localizer`, and the
motion planner (via `odom → base_link`) had **no such protection** — meaning
the exact scans this code identifies as shock-damaged/unreliable were still
the ones being trusted for the published pose. That's a strong candidate for
sudden, uncommanded pose jumps ("flying") that are independent of robot
speed, since a shock is about acceleration deviation/feature quality, not
commanded velocity.

## 3. Fix implemented

When a scan is flagged low-quality, the filter is now rolled back to its
pre-update (pure IMU-propagated) state/covariance **before** odometry is
published, instead of publishing the just-computed (and already-flagged-
unreliable) correction.

### Mechanism
- Right after `p_imu->Process(...)` (i.e. immediately after IMU-only forward
  propagation, before the LiDAR measurement update is applied), the state
  and covariance are snapshotted:
  ```cpp
  state_ikfom predicted_state_for_rollback = state_point;
  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov predicted_cov_for_rollback = kf.get_P();
  ```
- If `scan_is_low_quality` is later found true, the filter is restored to
  that snapshot via the existing IKFoM `change_x()`/`change_P()` API (already
  used elsewhere in this codebase, e.g. `IMU_Processing.hpp`'s `IMU_init`),
  and `state_point`/`euler_cur`/`pos_lid`/`geoQuat` are recomputed from it —
  the same recomputation already done right after a normal successful
  update, just now sourced from the rolled-back state:
  ```cpp
  kf.change_x(predicted_state_for_rollback);
  kf.change_P(predicted_cov_for_rollback);
  state_point = kf.get_x();
  euler_cur = SO3ToEuler(state_point.rot);
  pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
  geoQuat.x = state_point.rot.coeffs()[0];
  geoQuat.y = state_point.rot.coeffs()[1];
  geoQuat.z = state_point.rot.coeffs()[2];
  geoQuat.w = state_point.rot.coeffs()[3];
  ```
- `publish_odometry()` (and `publish_path()`, which is unconditional) read
  these same global variables plus `kf.get_P()` directly, so no other call
  site needed to change — they automatically now publish the rolled-back,
  IMU-only pose for a low-quality scan.
- `map_incremental()` and the various `publish_frame_*`/`publish_effect_*`
  calls remain skipped entirely for a low-quality scan exactly as before
  (unaffected by this change) — they never see `state_point` in either its
  pre- or post-rollback form, since they simply don't run.

### Logging
The existing `"Rejecting low-quality scan #%d: ..."` warning now also notes
`"- rolling odometry back to pre-update predicted state"`, so the log makes
clear that odometry was actively protected for that scan, not merely that
the map update was skipped.

## 4. Why this is believed safe

- Uses the filter's own existing, purpose-built state/covariance
  getter/setter API (`get_x`/`get_P`/`change_x`/`change_P`) — no new state
  representation or math introduced.
- Only activates on the exact same condition this codebase already uses to
  protect the map (`scan_is_low_quality`) — no new threshold or heuristic
  added, so behavior is unchanged for every scan that isn't already being
  flagged.
- The snapshot is taken unconditionally every cycle (cheap: one state copy +
  one covariance matrix copy, at LiDAR scan rate), so there's no ordering
  dependency risk — it's always available by the time the quality gate
  decision is made, regardless of which early-return path a given scan may
  or may not take before reaching that point.

## 5. How to verify this actually helps

1. Build this branch (`navigation_pipeline_develop`, with these local
   changes, plus the separate `map_localizer` fix) via `colcon build` and
   deploy to the robot.
2. Reproduce the original scenario.
3. Watch `fastlio_mapping`'s console for `"Rejecting low-quality scan #N: ...
   shock=1 ..."` lines. If the "flying" symptom used to correlate with these
   (especially `shock=1` cases), it should no longer produce a visible pose
   jump — `odom → base_link` should instead hold steady at the IMU-predicted
   pose through that scan, then resume normal tracking once a good scan
   returns.
4. `ros2 topic echo /Odometry` (or `/lidar_odometry`, per the remap in
   `diy_localization/launch/localization.launch.py`) during a repro — check
   that no discontinuous jump in `pose.pose.position`/`orientation` lines up
   with a `"Rejecting low-quality scan"` log line.

## 6. Known caveats / things NOT addressed here

- **Not compiled or run** — see the top of this report.
- The shock detector this fix relies on (`IMU_Processing.hpp`) only
  triggers on **linear acceleration magnitude** deviation from gravity. A
  fast rotation with little net linear acceleration change (e.g. the
  original "goal behind the robot" in-place spin that started this whole
  investigation) would **not** trigger `hadShock()` at all, and so would
  not benefit from this rollback. This is a separate, larger change (would
  need a second, gyro-rate-based shock criterion) and was intentionally
  left out of this change set.
- No true geometric degeneracy detection (eigenvalue analysis of the
  point-to-plane Jacobian) exists here either — only the existing total
  feature-count/mean-residual gate. A feature-rich but directionally
  degenerate scene (e.g. a long corridor) would still not be caught by
  `scan_is_low_quality` at all, so this rollback would never trigger for
  that failure mode.
