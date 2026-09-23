# map_localizer changes — localization robustness session

**Status**: `map_localizer` builds cleanly (last verified together with the rest of the 13-package workspace). The concurrency change in section C.5 has **not** been build-verified yet in this exact revision — build it before trusting it (`colcon build --symlink-install --packages-select map_localizer`).

This report covers every change made to `map_localizer` (`src/localizer_node.cpp`, `src/localizers/`, `config/map_localizer.yaml`, `package.xml`) in a session focused on hardening the FAST-LIO2 + `map_localizer` localization stack, which replaces AMCL: FAST-LIO2 owns high-rate `odom -> base_link`, `map_localizer` periodically corrects `map -> odom` by registering `/cloud_registered_body` against a saved map with `fast_gicp` (VGICP, rough-then-refine).

Steps are labeled to match the working session (Step A was the `fast_lio_ros2` shock-rollback fix, documented in that package's own report).

---

## Step B — ported/adapted fixes, robustness against known failure modes

### B.2 — TF smoothing (no more "teleport" on recovery)

**Problem**: ICP only re-registers at `update_hz` (e.g. 2 Hz). When it fails to converge for a stretch (fast maneuver, feature-poor geometry) it silently holds the last known-good `map -> odom` offset. The next successful alignment used to apply the **entire accumulated correction in one TF tick** — a sudden, visible jump in RViz/Nav2/the motion controller.

**Fix**: split the single `last_offset_r/t` into two:
- `target_offset_r/t` — the raw, latest ICP result (also used as ICP's warm-start guess — needs the true best estimate, not a lagging one).
- `last_offset_r/t` — the *smoothed* value actually broadcast over TF and used in `/amcl_pose`.

`advanceSmoothedOffset()` runs every 10 ms tick (the wall timer moved from `update_hz`-period to a fixed 10 ms so smoothing has enough resolution; **ICP itself still only runs at `update_hz`**, gated by an early-return in `timerCB`) and exponentially blends `last_offset` toward `target_offset` with time constant `offset_smoothing_time_constant` (default 0.5 s; `<= 0` restores instant-snap).

An explicit `/relocalize` or `/initialpose` **snaps immediately** instead of blending — there's no prior localized state to stay continuous with, so blending would only add lag to a correction the operator explicitly asked for.

**Config added**: `offset_smoothing_time_constant: 0.5`.

### B.3 — `/relocalize` retry tracks live odometry

**Problem**: `relocCB`'s response only confirms the map file *loaded* — not that ICP ever converged (`m_state.localize_success` flips true asynchronously, later, in `timerCB`). If the very first alignment attempt from the requested pose failed (e.g. the robot wasn't exactly at the assumed pose), every retry reused the **exact same static requested pose** forever, since `service_received` is only cleared on success.

**Fix**: `applyInitialGuess()` (called by both `relocCB` and `initialPoseCB`) now snapshots the robot's live odometry pose into `m_state.service_ref_r/t` at request time. `timerCB`'s `service_received` branch composes the originally-requested pose with how far the robot's odometry has moved *since* the request, so every retry gets a fresh, physically-consistent guess instead of an increasingly stale one.

### B.4 — Diagnostics

- `relocCB` now logs the incoming request, explicit rejection reasons (pcd not found / load failed), and an "ACCEPTED" log clarifying that acceptance only means the file loaded, not that ICP has converged.
- `timerCB`'s success branch logs `"ICP map alignment RECOVERED after N consecutive failure(s)"` and, on first lock acquisition, `"Map lock ACQUIRED: map->odom translation=(...)"` — the line that actually confirms localization is trustworthy (previously the only signal was silence).
- The pre-existing throttled failure warning was kept and enriched, not replaced.

### B.6 — `package.xml` hygiene

Added `<depend>` entries for `yaml-cpp`, `eigen`, `libpcl-all-dev` — `CMakeLists.txt` already `find_package()`s/links them directly so this didn't affect the build, it's rosdep metadata only.

---

## Step C — additional robustness passes

### C.1 — Consistent odometry/cloud snapshot for ICP

**Problem**: the ICP warm-start guess and the odometry/cloud actually fed to `align()` were read from `m_state` in two separate lock acquisitions in `timerCB`. A `syncCB` update landing between them could pair the warm-start guess with slightly different (newer) odometry than what was actually aligned. (Note: the resulting `map->odom` offset math itself was *not* corrupted by this — it's self-consistent — only the warm-start guess could be marginally stale.)

**Fix**: one snapshot (`current_local_r/t`, `current_time`, `current_target_offset_r/t`, and — after C.4 — the scan history) taken under a single `message_mutex` lock at the top of `timerCB`; both the warm-start guess and the post-`align()` offset math are derived from that same snapshot.

### C.2 — `/localization_status` topic

**New message**: `slam_interfaces/msg/LocalizationStatus` (`msg/LocalizationStatus.msg`) — `header`, `localized` (bool), `consecutive_align_failures` (int32), `rough_fitness`/`refine_fitness` (float64). Published every ICP cycle from `publishLocalizationStatus()` on topic `localization_status`. Lets other nodes (e.g. `motion_controller`) throttle speed or react when the map lock degrades, instead of only finding out indirectly via a pose jump.

### C.3 — Auto-recovery via map-derived grid hypotheses

**Problem**: if ICP fails against the last-known offset for a long stretch (robot picked up, drove somewhere unmapped and back, drift beyond the jump-gate's tolerance), the node holds the stale offset forever with no path back to a correct lock.

Two designs were tried and rejected before landing on the current one:
1. *Waypoint-file based* (like `diy_waypoint_sequencer`'s `waypoints.yaml`) — rejected because no real course waypoints file exists yet in this repo (only a generic placeholder), and it would couple `map_localizer` to another package's config.
2. *Waypoints generated from the course PGM occupancy grid* — worked, but still an external-file dependency and course-specific regeneration step.

**Final design — self-contained, no external file**: `computeRecoveryHypotheses()` runs whenever a map (re)loads (startup `map_path` or a later `/relocalize`). It reads the *already-loaded* map cloud (`m_localizer->roughMap()`), lays a grid over its XY bounding box at `recovery_grid_spacing` (default 2.0 m), and keeps only grid cells with actual map structure nearby (an occupancy check against the map cloud voxelized at half that spacing) — so hypotheses aren't wasted on open space outside the mapped footprint.

`attemptRecoverySweep()` is triggered from `timerCB`'s failure branch every time `consecutive_align_failures` reaches a multiple of `recovery_after_failures` (0 = disabled, the default). It tries each grid hypothesis at `recovery_yaw_samples` evenly-spaced yaws (default 4, i.e. every 90°, since a grid point — unlike a hand-picked waypoint — carries no heading information), returning on the first hypothesis that converges. On success, computes `map->odom` the same way the normal path does, **snaps** `last_offset` (no continuity to preserve — we were lost), and resets the failure counter.

**Config added** (all disabled/conservative by default):
```yaml
recovery_after_failures: 0     # 0 = disabled
recovery_grid_spacing: 2.0     # meters
recovery_yaw_samples: 4
```

### C.4 — Scan accumulation before ICP

**Problem**: each ICP alignment ran against a single `/cloud_registered_body` scan — potentially thin given the QT64's ~1 m blind zone.

**Fix**: `syncCB` now allocates a **fresh** `CloudType::Ptr` per callback (previously it mutated the same object in place, which would have corrupted any retained history) and keeps a bounded `scan_history` deque (size = `accumulate_scans`, clamped to `[1, 10]`). `buildAccumulatedCloud()` merges the history into the most recent scan's body frame using the same rigid-composition math already used for the `map->odom` offset elsewhere in the file, and is a zero-cost no-op (returns the single most recent scan) when `accumulate_scans <= 1` (the default).

**Config added**: `accumulate_scans: 1` (disabled by default).

### C.5 — Dedicated ICP callback group (concurrency)

**Problem**: under the default single-threaded executor, a slow `align()` call — or worse, a full recovery sweep (C.3) — blocked *every* callback, including `syncCB`'s TF broadcast and the 10 ms smoothing tick, for its entire duration.

**Fix**: `timerCB` now runs in its own `MutuallyExclusive` callback group (`m_icp_callback_group`), separate from the default group (`syncCB`, the services, `/initialpose`). `main()` switched from `rclcpp::spin()` (single-threaded) to a 2-thread `rclcpp::executors::MultiThreadedExecutor` so the groups actually run concurrently.

This required a genuine concurrency audit, not just moving the timer:
- **`m_localizer` and `m_recovery_hypotheses`** are the only state genuinely reachable from both groups (ICP group: `timerCB`/`attemptRecoverySweep`/`publishLocalizationStatus`/`publishMapCloud`; default group: `relocCB`/constructor via `loadMap()`). Added `std::mutex m_localizer_mutex` guarding every access. Convention: the four "entry point" methods each acquire the lock themselves for their own scope; `computeRecoveryHypotheses()` and `attemptRecoverySweep()` are documented as requiring the caller to already hold it (avoids self-deadlock on the non-recursive mutex, since both are only ever called from within an already-locked section).
- **`message_received`, `service_received`, `localize_success`** (in `NodeState`) converted from `bool` to `std::atomic<bool>` — these are read/written from both groups without another mutex consistently covering every access site. `std::atomic<bool>` supports the existing `=`/implicit-bool-conversion call sites unchanged (no call-site edits needed).
- Everything else already confined to a single callback group (e.g. `last_r/t`, `scan_history`, `target_offset_r/t` — only ever touched by the timer group; `m_map_loaded` — only ever touched by the default group) needed no new locking.
- Verified lock-acquisition order is consistent everywhere it nests (`m_localizer_mutex -> message_mutex -> service_mutex`) to rule out deadlock from lock-order inversion.

**Not done in this pass**: this doesn't include the full X-ICP-style Jacobian-eigenvalue degeneracy detection (C.7 below) — evaluated and deliberately deferred (see rationale there).

### C.6 — Gyro-based shock criterion

Implemented in `fast_lio_ros2`, not this package — see that package's own `CHANGES_REPORT.md`. Included here only because it was part of the same session and affects what scans `map_localizer` ends up receiving.

### C.7 — Jacobian-eigenvalue degeneracy gate — evaluated, not implemented

**The gap**: ICP can converge to a confident-looking wrong answer when scan geometry doesn't fully constrain all 6 DOF — classic example: a long, straight, feature-poor corridor, where position *along* the corridor axis is barely constrained even though the fitness score looks fine. The existing jump-gate (`max_offset_jump_dist/yaw`) only catches a *sudden* bad jump, not a slow, silent drift where each individual update looks small and plausible.

**Why it was not implemented this session**:
1. `fast_gicp`'s `FastVGICP::align()` doesn't expose the per-correspondence Jacobians or final Hessian needed to compute directional confidence — would require patching `fast_gicp` or re-deriving an approximate information matrix by hand after the fact.
2. Eigenvalue thresholds are environment/map-density-specific and need real field data to tune, not a one-shot guess.
3. What to *do* on detection (treat as a failure vs. a partial/projected update) is itself a nontrivial design decision touching the core offset math.
4. No test harness exists in this repo to validate a hand-rolled information-matrix computation before trusting it on the real robot.

**Recommendation on record**: skip unless the course's actual environment has long feature-poor corridors *and* testing shows a slow silent drift (not a sudden jump — those are already caught) in exactly that kind of geometry. If it does come up, revisit with real log data in hand rather than guessing thresholds blind. A cheaper (but less rigorous) fallback discussed: PCA on the matched scan points themselves as a proxy for directional degeneracy, avoiding the need to touch `fast_gicp` internals.

---

## Config surface added (`config/map_localizer.yaml`)

```yaml
offset_smoothing_time_constant: 0.5
recovery_after_failures: 0        # 0 = disabled
recovery_grid_spacing: 2.0
recovery_yaw_samples: 4
accumulate_scans: 1               # 1 = disabled
```

## New files

- `slam_interfaces/msg/LocalizationStatus.msg` (new message type, in the `slam_interfaces` package).

## What was intentionally *not* ported from `referance_map_localizer`

That reference branch (an older snapshot with one isolated fix bolted on) removed several things this active package already has and deliberately keeps: `/initialpose` support, `/amcl_pose` publishing with fitness-based covariance, the `max_offset_jump_dist/yaw` gate, and `map_path`/start-pose auto-load. It also re-added a `hasConverged()` hard-gate that this package's own code comments explain was deliberately dropped (observed to disagree with fitness score in practice), and used unnamed-temporary `std::lock_guard<std::mutex>(mutex);` calls that don't actually hold the lock — not reproduced here.
