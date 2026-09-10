# Custom Navigation Stack — Design & Zone Nav Decoupling

Companion doc to [jetson_bringup_guide.md](jetson_bringup_guide.md) — covers
the custom A* + PD/pure-pursuit navigation stack (`diy_planning`,
`diy_motion_planner`) built by the teammate, why it is already decoupled
from `diy_zone_nav`, the two real bugs found while verifying that, and the
new `diy_waypoint_sequencer` package built for automated goal sequencing.

Full investigation and verification detail: `docs/reuse_plan_step1.md` Step 13.

---

## 1. Architecture Overview

This robot uses **only `nav2_map_server` + `nav2_lifecycle_manager`** from
the Nav2 ecosystem — map serving only, not the full Nav2 navigation system
(no `planner_server`, `controller_server`, or costmap layers). Path
planning and control are fully custom.

**Planning pipeline:**

```
/map (from nav2_map_server)  ──╮
/goal_pose (from user/RViz)  ──┼──►  A* Planner  ──►  /a_star/path
robot pose (TF: map→base_link) ─╯
```

**Control pipeline:**

```
/a_star/path (from A* planner)     ──╮
robot pose (TF: odom→base_link)    ──┼──►  PD Controller  ──►  /cmd_vel_nav
                                                                     │
                                                                     ▼
                                                            cmd_vel_mux_node
                                                                     │
                                                                     ▼
                                                              /cmd_vel_safe
                                                                     │
                                                                     ▼
                                                              motor driver
```

`/cmd_vel_nav` is the **same topic name** `cmd_vel_mux_node` already reads
for AUTONOMOUS mode — no mux changes were needed to plug this controller in.

---

## 2. The Zone Nav Question — What Was Actually Verified

The ask: design so `diy_zone_nav` can be removed entirely with no impact,
since the plan is for the controller to eventually consume "info from the
zone navigator." Investigated the real code before designing anything —
did not assume.

| Check | Method | Result |
|---|---|---|
| Does `diy_planning`/`diy_motion_planner` reference `/nav_mode`, `/speed_limit`, or `zone_nav` anywhere? | `grep -rn` across both packages | **Zero hits** — no existing coupling at all |
| Does the A* planner crash/hang with no `/goal_pose`? | Read `goal_callback`'s guard clauses directly | No — early-return, idles safely, no timeout/crash |
| Does `cmd_vel_mux`'s AUTONOMOUS mode depend on zone_nav? | Read `cmd_vel_mux_node.py`'s mode logic | No — reads `/cmd_vel_nav` unconditionally; BLIND_DRIVE only activates via an explicit service call only `zone_nav_manager_node` makes |
| Does zone_nav crash if Nav2's costmap/controller services don't exist? | Read `zone_nav_manager_node.cpp`'s service-call sites | No — already checks `service_is_ready()` before every call, skips gracefully with a warn log |

**Conclusion: `diy_zone_nav` was already fully orphaned and safely
removable in this architecture before any code was changed.** Its
`/speed_limit` output and costmap `SetParameters` calls have no consumer
and no valid service target respectively, in a stack that only runs
`nav2_map_server`. Nothing needed to be built to satisfy "can take zone
navigator out" for the pipeline as it already existed.

---

## 3. Two Real Bugs Found and Fixed While Verifying This

**Bug 1 — `cmd_vel_topic` launch default mismatch.**
`pd_navigation.launch.py` and `pure_pursuit_navigation.launch.py` declared
`cmd_vel_topic` defaulting to `/cmd_vel` (the simulation value), while
their sibling arguments (`base_frame`, `use_sim_time`) already defaulted to
hardware values (`base_link`, `false`). A bare hardware launch with no
override would silently publish velocity commands into `/cmd_vel`, which
`cmd_vel_mux` never subscribes to — the robot simply would not move, with
no error anywhere. **Fixed:** both launch files now default
`cmd_vel_topic` to `/cmd_vel_nav`, consistent with their other hardware
defaults.

**Bug 2 — no ROS signal for "goal reached."**
Neither `pd_motion_planner_node.py` nor `pure_pursuit_motion_planner_node.py`
published anything when the robot reached its goal — only an internal log
line. This blocks any future automated sequencing (nothing to react to).
**Fixed:** added a `/pd/goal_reached` (`std_msgs/Bool`) publisher to both
controller nodes, published at the exact point each already detects goal
completion — purely additive, no existing behavior changed.

---

## 4. New Package: `diy_waypoint_sequencer`

Built to satisfy the actual open design question: something needs to
auto-publish `/goal_pose` in sequence for a competition run where no human
is clicking RViz goals.

**Key design decision:** built as a brand-new, standalone, minimal
package — **not** added inside `diy_zone_nav`, `diy_planning`, or
`diy_motion_planner`. Putting it inside `diy_zone_nav` would have
re-coupled "can I remove zone_nav" with "do I still get automated
sequencing" — exactly the ambiguity this whole task was about avoiding.

**How it works:**
1. Loads a simple waypoints YAML (`label, x, y, yaw` — deliberately NOT
   reusing `zone_waypoints.yaml`'s much richer zone/radius/mode schema,
   which encodes Nav2-costmap concepts this simpler stack doesn't use).
2. Waits for `/green_light` (same competition start-trigger topic
   convention already used by `zone_nav`) — or starts immediately after a
   configurable delay for bench testing.
3. Publishes the current waypoint on `/goal_pose`.
4. Advances to the next waypoint when `/pd/goal_reached` fires `True`.
5. Optionally loops back to the first waypoint after the last
   (`loop: true`).

**If this node is not run:** `/goal_pose` is simply never auto-published —
a human can still publish it manually (RViz "2D Goal Pose", or
`ros2 topic pub`) with zero code changes anywhere else in the stack.

**Verification performed** (a real functional test, not just written and
assumed correct — `tf_transformations` isn't installed in this sandbox and
there's no `sudo` access to add it, so its one function was stubbed with
equivalent pure math for the test only):

| # | Behavior tested | Result |
|---|---|---|
| 1 | No goal published before `/green_light` | PASS |
| 2 | First waypoint published correctly on green light | PASS |
| 3 | `/pd/goal_reached` advances to the next waypoint | PASS |
| 4 | Sequence stops cleanly after the last waypoint (no loop) | PASS |
| 5 | Duplicate `/green_light` after start is a no-op | PASS |
| 6 | `wait_for_green_light:=false` auto-starts with no green light at all | PASS |
| 7 | `loop:=true` wraps back to waypoint 0 after the last | PASS |

Also confirmed the package builds clean in a full 15-package workspace
rebuild, and its launch file's arguments resolve correctly
(`ros2 launch diy_waypoint_sequencer waypoint_sequencer.launch.py --show-args`).

---

## 5. How to Launch / Test This Stack

```bash
# Hardware, PD controller, with automated sequencing:
ros2 launch diy_motion_planner pd_navigation.launch.py \
    map_yaml:=/absolute/path/to/map.yaml

ros2 launch diy_waypoint_sequencer waypoint_sequencer.launch.py \
    waypoints_file:=/absolute/path/to/waypoints.yaml

# Manual goal testing (no sequencer needed):
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}"

# diy_zone_nav is NOT required for any of the above to work.
```

### Switching maps during testing

The course map (2D occupancy grid, `.pgm`/`.yaml` — feeds `nav2_map_server`
→ `/map` → the A\* planner) is expected to change often as testing
progresses. `map_yaml` in both `pd_navigation.launch.py` and
`pure_pursuit_navigation.launch.py` now defaults to, in priority order:

1. **`$DIY_MAP_YAML`** env var, if set
2. **`$DIY_ROS_WS/src/DIY-Challenge-Repo/maps/global_map_2_smooth.yaml`**
   otherwise (the map currently used for initial localizer/controller
   testing)

So switching maps needs no launch-file edits — just one of:

```bash
# Once per test session, before launching anything:
export DIY_MAP_YAML=/absolute/path/to/new_map.yaml

# Or override for a single run without exporting anything:
ros2 launch diy_motion_planner pd_navigation.launch.py \
    map_yaml:=/absolute/path/to/new_map.yaml
```

This mirrors how `diy_localization`'s `map_pcd_path` already works for
`map_localizer`'s map (see `jetson_bringup_guide.md` §2/§3) — but note
these are two **separate, unrelated maps**: `map_yaml` is a 2D occupancy
grid consumed only by `nav2_map_server`/the A\* planner; `map_pcd_path` is
a 3D point cloud consumed only by `map_localizer` for the `map→odom` TF.
A `.pgm`/`.yaml` pair cannot be used as a `map_pcd_path` value or vice
versa.

---

## 6. Still Open

- The custom controller stack is not yet wired into
  `challenge_master.launch.py` — currently launched standalone only.
  Master-launch integration (a `use_custom_nav`-style flag, and how it
  coexists with the existing `diy_zone_nav`/Nav2 blocks already there) is
  a separate, larger decision not made in this pass.
- Whether `diy_zone_nav` should eventually be deleted outright (vs. kept
  orphaned-but-present) was not decided — no urgency since it is already
  proven harmless to leave in place.
- `tf_transformations` is not installed in this sandbox (no `sudo`) —
  affects verifying this and the pre-existing `diy_motion_planner` package
  identically; not a new gap introduced by this work.
