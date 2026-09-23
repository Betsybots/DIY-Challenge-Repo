# map_localizer changes — investigation report

**Branch**: `navigation_pipeline_develop` (local only — not committed, not pushed)
**Files changed**: everything under this package (`src/map_localizer/`) plus the
active runtime config at `src/diy_localization/config/map_localizer.yaml`.
**Status**: NOT YET BUILT OR TESTED — this Windows environment has no ROS 2 /
PCL / `fast_gicp` toolchain, so these changes have only been manually reviewed
(brace-balance checked, math cross-checked against the existing offset-
composition pattern already used elsewhere in this file). Build with
`colcon build` on the robot/dev Linux machine and test before trusting this.

---

## 1. Symptom being investigated

Robot sometimes loses localization mid-run and appears to "fly"/teleport in
RViz. Originally reproduced by sending a goal pose behind the robot; later
confirmed to also happen with goals ahead of it, after several goal poses had
already been sent (i.e. not tied to any specific rotation or robot speed).

## 2. How we got here (investigation summary)

1. Ruled out: pure-pursuit commanding an instant full-speed in-place spin
   (fixed separately on `behzad_develop`, in `motion_planner`'s
   `pure_pursuit_motion_planner_node.py`) — retested with a slowed robot and
   the issue persisted, so this wasn't the (sole) cause.
2. Confirmed via `tf2_echo`: `map → odom` was **not changing at all** while
   the robot still drifted/"flew" in the map view.
3. Per REP 105, `map → odom` exists specifically to correct FAST-LIO2's
   (`odom → base_link`) natural, unavoidable drift against the fixed,
   pre-built map. A `map → odom` that never updates means **all of FAST-
   LIO2's drift is going completely uncorrected** — which explains both the
   lack of rotation/speed dependence and why it only becomes visible after a
   few goals (i.e. after enough distance for the drift to be noticeable).
4. Traced *why* `map → odom` might never update, by reading
   `localizer_node.cpp`'s actual `align()`/`timerCB()`/`relocCB()` logic (see
   root causes below).

## 3. Root causes found in this package's original code

### 3a. Silent freeze on ICP failure (no visibility at all)
`timerCB()` only updates the broadcast `map → odom` offset when
`ICPLocalizer::align()` succeeds. On failure, it silently keeps re-
broadcasting the last known-good offset forever, with **zero logging** —
there was no way for an operator to tell "the correction is stale" from
"everything is fine."

### 3b. A failed *first* alignment attempt could strand the node permanently
`relocCB()` (the `/relocalize` service handler) sets `service_received =
true` and returns `"relocalize success"` as soon as the requested PCD map
**file loads from disk** — this response does **not** wait for or confirm
that ICP actually converges against that map. The real lock is
`m_state.localize_success`, which is only set `true` later, asynchronously,
inside `timerCB()`, if and when `align()` succeeds.

Critically, `service_received` was only ever reset to `false` on **success**
(the reset-on-failure line was commented out:
`// m_state.service_received = false;`). So if the very first alignment
attempt after a `/relocalize` call failed (e.g. because the robot wasn't
*exactly* at the assumed map origin `(0,0,0,0)` — see
`diy_localization/launch/localization.launch.py`'s own
`initial_x/y/z/yaw/pitch/roll` defaults and comment: *"Robot assumed to
start at map origin"*), every subsequent retry reused the **exact same
static, non-moving pose** as its ICP warm-start guess, forever. There was no
fallback to the normal (odometry-tracking) warm-start path while
`service_received` remained `true`. Net effect: `map_localizer` could get
permanently stuck at its default identity `map → odom` offset from the very
first second of the run — matching the observed "`map → odom` never
changes" — while the `/relocalize` service call itself still reported
`"success"`.

### 3c. Instantaneous "snap" when a stale correction finally recovers
Even when `align()` *does* eventually succeed again after a run of failures,
the old code applied the newly computed offset **instantly** — any drift
accumulated during the failed stretch would appear as a single, large,
one-tick jump in the broadcast `map → odom` TF (visible everywhere
downstream: RViz, Nav2, the motion planner).

## 4. Changes made (all in `localizer_node.cpp` unless noted)

### Fix for 3b — service-provided guess now tracks live odometry
- Added `NodeState::service_ref_r` / `service_ref_t`: a snapshot of the
  robot's FAST-LIO2 odometry pose at the moment `/relocalize` was received
  (set in `relocCB`).
- `timerCB()`'s `service_received` branch no longer reuses the static
  requested pose verbatim. It now composes the originally requested pose
  with however far the robot's own odometry has moved *since* the request
  (`req_offset_r/t`, following the exact same offset-composition pattern
  already used elsewhere in this file for the non-service warm-start path).
  Every retry therefore gets a fresh, physically-consistent guess instead of
  an increasingly stale one. Once alignment finally succeeds, it transitions
  to the normal `target_offset`-based tracking exactly as before.

### Fix for 3c — smoothed corrections instead of instant snaps
- Added `NodeConfig::offset_smoothing_time_constant` (seconds; default
  `0.5`, `<= 0` restores the old instant-snap behavior).
- Split the previously single `last_offset_r/t` into:
  - `target_offset_r/t` — the latest *raw*, unsmoothed correction from a
    successful `align()` (also used as the ICP warm-start guess, since ICP
    needs the best available estimate, not a lagging one).
  - `last_offset_r/t` — the *smoothed* value actually broadcast over TF,
    which exponentially chases `target_offset_r/t` (translation: linear
    blend; rotation: quaternion `slerp`) via the new `advanceSmoothedOffset()`
    method, called every 10 ms tick regardless of `update_hz`.
  - An explicit `/relocalize` call still **snaps immediately** (no prior
    localized state to stay continuous with, so smoothing would only add
    unnecessary lag there).

### Fix for 3a / general diagnosability — logging added throughout
- `relocCB()`: logs the incoming request (pcd path + initial pose), and logs
  explicitly on rejection (`pcd file not found` / `failed to load pcd map`)
  and on acceptance — including an explicit note that **this response alone
  only confirms the map file loaded, not that ICP ever converged**.
- `timerCB()`:
  - On failure: `RCLCPP_WARN` with the rough/refine fitness scores *and*
    thresholds, current `localized` state, and a running
    `consecutive_align_failures` counter (`NodeState::consecutive_align_failures`)
    so a stuck run is immediately visible as an ever-growing attempt count.
  - On the first success after a run of failures: `RCLCPP_INFO` reporting
    how many consecutive failures preceded the recovery.
  - On actually acquiring the lock for the first time
    (`localize_success` flips `false → true`): `RCLCPP_INFO` with the
    resulting `map → odom` translation — this is the log line that
    disambiguates "map file loaded" from "localization is actually
    trustworthy now."

### `icp_localizer.h` / `icp_localizer.cpp`
- `ICPLocalizer` now records the rough/refine ICP fitness scores from the
  most recent `align()` call (`lastRoughFitness()` / `lastRefineFitness()`),
  regardless of whether that call succeeded, so `localizer_node.cpp` can log
  *why* an alignment failed instead of only whether it did.

### `src/diy_localization/config/map_localizer.yaml` (the **active** runtime
config — the untouched upstream mirror at `src/map_localizer/config/` was
deliberately left alone, per its own header comment)
- Added `offset_smoothing_time_constant: 0.5` with an explanatory comment.

## 5. How to verify this actually fixes the problem

1. Build this branch (`navigation_pipeline_develop`, with these local
   changes) via `colcon build` on the robot/dev Linux machine and deploy.
2. Reproduce the original scenario (send several goal poses, including at
   least one behind the robot).
3. Watch `map_localizer_node`'s console (it runs with `output="screen"`) for:
   - `"relocalize ACCEPTED: ..."` right after startup.
   - Either `"Map lock ACQUIRED: ..."` shortly after (good), or a run of
     `"ICP map alignment failed (attempt #N since last lock, ...)"` messages
     (tells you it's still retrying, and why — check the fitness numbers
     against the thresholds also printed).
   - If it does fail initially, confirm it eventually logs `"ICP map
     alignment RECOVERED after N consecutive failure(s)"` — if this happens
     early in the run, that confirms fix 3b actually mattered here.
4. `ros2 run tf2_ros tf2_echo map odom` — should now show real, changing
   values (not stuck at translation `(0,0,0)` for the whole run).
5. If the earlier symptom (a large instantaneous jump right as the robot
   recovers from a stale correction) still occurs even with a confirmed
   `"Map lock ACQUIRED"`/`"RECOVERED"` log, that would point at fix 3c
   (`offset_smoothing_time_constant`) not being effective enough — consider
   raising it (e.g. to `1.0`–`2.0`) rather than reverting it.

## 6. Known caveats / things NOT addressed here

- **Not compiled or run** — see the top of this report. Please build and
  test before relying on this.
- A separate, still-open lead from this investigation: FAST-LIO2's own
  shock/low-quality-scan detector (`src/fast_lio_ros2/src/laserMapping.cpp`)
  gates the persistent map/point-cloud publishing on scan quality, but
  **still publishes odometry/TF from a scan it has already flagged as
  shock-damaged or low-quality**. That is a separate, not-yet-implemented
  fix (would require rolling the EKF state back to its pre-update/predicted
  value via `change_x()`/`change_P()` when a scan is rejected) — out of
  scope for this change set, tracked separately.
- The shock detector in `IMU_Processing.hpp` only triggers on linear
  acceleration magnitude deviation, not on angular velocity/rotation — a
  fast rotation with little net linear acceleration change would not be
  caught by it at all. Also out of scope here.
- No true geometric degeneracy detection (e.g. eigenvalue analysis of the
  point-to-plane Jacobian) exists on the FAST-LIO2 side; only a total
  feature-count/mean-residual gate. A feature-rich but directionally
  degenerate scene (e.g. a long corridor) would not be caught. Also out of
  scope here.
