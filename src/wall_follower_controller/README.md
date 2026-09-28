# Wall Follower Controller

`wall_follower_controller` is a separate Nav2 `nav2_core::Controller` plugin
for the fixed Ackermann speed course. It does not replace or modify the
existing Regulated Pure Pursuit controller or Smac Hybrid-A* planner.

## Control flow

1. The existing `GridBased` planner supplies the course path.
2. The controller transforms that path into the robot frame and calculates
   Pure Pursuit path curvature.
3. `/cloud_registered_body` supplies wall points in `base_footprint`.
4. Robust line fits estimate the left wall, right wall, corridor center
   offset, and corridor heading.
5. Bounded wall corrections are added to the planned-path curvature.
6. Curvature ahead on the path limits speed and starts braking before turns.
7. The configured footprint is projected through the local costmap before a
   command is returned.
8. Nav2's velocity smoother publishes `/cmd_vel_smoothed` to `ackermann-drive`.

If either wall estimate is missing or stale, the default configuration throws
a controller failure so Nav2 stops and runs the wait-only recovery tree.

## Build

On the Ubuntu 22.04 / ROS 2 Humble target:

```bash
cd ~/DIY-Challenge-Repo
colcon build --symlink-install \
  --packages-select wall_follower_controller challenge_bringup
source install/setup.bash
```

## Launch

```bash
ros2 launch challenge_bringup \
  nav2_navigation_wall_follower_ackermann.launch.py
```

The launch loads these files in order:

1. `nav2_params_3d_speed_course_ackermann.yaml`
2. `nav2_params_3d_speed_course_wall_follower_ackermann.yaml`

The second file only changes the selected controller and behavior trees. The
base `GridBased` planner configuration remains unchanged.

## Debug output

The fitted local wall centerline is published as:

```text
/controller_server/WallFollow/wall_centerline
```

Add it to RViz as a `Path`. Do not enable high-speed control until the path is
stable, centered, and continuous through both hairpins.

## Field tuning order

Search the override YAML for `FIELD-TUNE-WALL` and tune in this order:

1. Point-cloud freshness and sampling
2. Wall point-selection bounds
3. Lateral and heading correction gains
4. Lookahead and preview speed control
5. Measured acceleration, braking, steering, and lateral limits
6. Footprint collision prediction

Start with low controller and smoother velocity limits. Increase one parameter
group at a time and require at least three repeatable runs.
