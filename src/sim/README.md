# diy_sim — Simulation Package

Ignition Gazebo Fortress simulation for the DIY Robot Challenge 2026.
This package is used to test the navigation pipeline on a **laptop** before touching the real robot.

---

## What's in this package

| File / Directory | Purpose |
|---|---|
| `worlds/simple_loop.sdf` | Rectangular test track (1.05 m corridors, no obstacles) |
| `worlds/competition_arena.sdf` | Full competition course with obstacles, ramp, tunnel |
| `models/` | Gazebo models (grass, asphalt, straw bales) |
| `sim_simple_loop.launch.py` | **All-in-one launch** — Gazebo + Nav2 + circuit_runner |
| `sim_competition.launch.py` | Gazebo only (sim world + robot spawn + bridge) |
| `scripts/sim_telemetry.py` | Live 4-panel dashboard (trajectory, velocity, mode, pose) |

---

## Prerequisites

### ROS 2 Humble + Ignition Fortress

```bash
# ROS 2 Humble
sudo apt install -y ros-humble-desktop

# Ignition Fortress
sudo apt install -y ignition-fortress

# ROS-Ignition bridge + simulation packages
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-tf2-ros \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

### Python dependencies (for telemetry scripts)

```bash
pip3 install matplotlib numpy
```

---

## Build

```bash
cd ~/DIY-Challenge-Repo

# Install ROS deps (first time only)
rosdep install --from-paths src --ignore-src -r -y

# Build (symlink-install avoids rebuild on script edits)
colcon build --symlink-install --packages-select \
  diy_sim diy_robot_description diy_zone_nav challenge_bringup

# Source
source install/setup.bash
```

> **Tip:** If you only changed Python scripts (launch files, circuit_runner_node.py),
> you do NOT need to rebuild — `--symlink-install` picks up edits immediately.
> Just re-source if you opened a new terminal.

---

## Running the sim (quick start)

### Terminal 1 — launch everything

```bash
source ~/DIY-Challenge-Repo/install/setup.bash
ros2 launch diy_sim sim_simple_loop.launch.py
```

Ignition Gazebo opens with the **simple_loop** track.
Wait ~18 seconds for Nav2 to finish loading (watch the terminal — you'll see
`"bt_navigator lifecycle transition: configure → activate"` when it's ready).

### Terminal 2 — fire the start signal

```bash
source ~/DIY-Challenge-Repo/install/setup.bash
ros2 topic pub --once /green_light std_msgs/msg/Bool "data: true"
```

The robot starts moving. `circuit_runner` navigates turn-by-turn.

### Terminal 3 — live telemetry (optional but very useful)

```bash
source ~/DIY-Challenge-Repo/install/setup.bash
python3 src/diy_sim/scripts/sim_telemetry.py
```

Shows: XY trajectory on the arena, velocity, nav mode timeline, odom pose.

### Terminal 4 — record trajectory to CSV + plot (for offline debugging)

```bash
python3 scripts/record_trajectory.py --out /tmp/run1
# Ctrl-C when run ends → saves run1_trajectory.csv and run1_plot.png
```

---

## Launch options

| Argument | Default | Description |
|---|---|---|
| `world` | `simple_loop.sdf` | World file in `diy_sim/worlds/` |
| `use_circuit_runner` | `true` | Autonomous lap orchestrator |
| `use_rviz` | `false` | Open RViz2 for costmap/path visualisation |
| `use_zone_nav` | `true` | Include zone state machine |
| `gui` | `true` | `false` = headless Gazebo |

Examples:

```bash
# With RViz2 to see costmaps
ros2 launch diy_sim sim_simple_loop.launch.py use_rviz:=true

# Headless (SSH / CI)
ros2 launch diy_sim sim_simple_loop.launch.py gui:=false

# Full competition arena
ros2 launch diy_sim sim_simple_loop.launch.py world:=competition_arena.sdf

# Manual nav (disable auto-runner, use 2D Nav Goal in RViz)
ros2 launch diy_sim sim_simple_loop.launch.py use_circuit_runner:=false use_rviz:=true
```

---

## Track geometry (simple_loop.sdf)

```
NW gate (-0.5, 0) ──────── Leg 1 (+X) ──────────── NE turn (13, 0)
     │                    y_center = 0                     │
  Leg 4                  width = 1.05 m                  Leg 2
  (+Y)                                                    (-Y)
  x=-0.5                                                  x=13
     │                                                     │
SW turn (0, -13) ───────── Leg 3 (-X) ──────────── SE turn (13, -13)
                           y_center = -13
```

Corridor width: **1.05 m** (wall to wall).
Robot width:    **0.40 m**.
Clearance per side: **0.325 m** — very tight for turning.

---

## Known issue — Turn 1 (NE corner)

> **This is the bug we need your help fixing.**

The robot reliably completes Leg 1 (east) but fails at Turn 1 (NE corner, x≈13, y=0):

- **Symptom**: turns too early / too late → drives into the east wall on Leg 2, gets stuck.
- **Architecture**: `circuit_runner_node.py` sends a `NavigateToPose` goal to x=11, y=0 (2 m before the corner to avoid wall inflation), then does an **in-place rotation** to yaw=-1.57 (south) via direct `/cmd_vel`, then sends the next `NavigateToPose` to the Leg 2 turn.
- **Suspected causes**:
  1. MPPI controller (current) can't reliably track the path in 1.05 m corridors — it samples trajectories that mostly hit walls, so ObstaclesCritic dominates and freezes motion.
  2. `xy_goal_tolerance=0.10` with `vx_max=0.30` → robot overshoots turn1 XY goal and Nav2 "succeeds" too early.
  3. After in-place rotation, MPPI sends the robot into the wall before the next Nav2 goal is accepted.

### Recommended fix to try

**Switch `FollowPath` controller from MPPI → Regulated Pure Pursuit (RPP)**:

```yaml
# src/challenge_bringup/config/nav2_params.yaml
controller_server:
  ros__parameters:
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.4
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      use_cost_regulated_linear_velocity_scaling: true
      inflation_cost_scaling_factor: 3.0
      allow_reversing: false
```

Also add to `exec_depend` in `challenge_bringup/package.xml`:

```xml
<exec_depend>nav2_regulated_pure_pursuit_controller</exec_depend>
```

Then rebuild + test:

```bash
colcon build --symlink-install --packages-select challenge_bringup
ros2 launch diy_sim sim_simple_loop.launch.py
```

---

## Key config files

| File | What to tune |
|---|---|
| `src/challenge_bringup/config/nav2_params.yaml` | Controller, planner, costmap, goal tolerances |
| `src/diy_zone_nav/config/sim_simple_loop_waypoints.yaml` | Waypoint XY positions and target yaw |
| `src/diy_zone_nav/scripts/circuit_runner_node.py` | Turn detection, rotation logic, retry policy |

---

## Debugging tips

```bash
# Watch Nav2 feedback (goal status per waypoint)
ros2 topic echo /navigate_to_pose/_action/feedback

# Watch what the controller is commanding
ros2 topic echo /cmd_vel

# Watch robot position
ros2 topic echo /odom --field pose.pose.position

# Check Nav2 lifecycle state (should all be 'active')
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server

# Visualise costmaps in RViz
ros2 launch diy_sim sim_simple_loop.launch.py use_rviz:=true
# Add displays: /global_costmap/costmap, /local_costmap/costmap, /plan
```

---

## Two-terminal alternative (same as the single launch)

If the all-in-one launch has issues, run them separately:

```bash
# Terminal 1: sim world + robot
ros2 launch diy_sim sim_competition.launch.py world:=simple_loop.sdf

# Terminal 2: Nav2 + circuit_runner (after Gazebo is fully up)
ros2 launch challenge_bringup sim_nav.launch.py

# Terminal 3: start signal
ros2 topic pub --once /green_light std_msgs/msg/Bool "data: true"
```
