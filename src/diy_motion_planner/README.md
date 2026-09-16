# diy_motion_planner

C++ motion planners for the DIY Challenge robot: a standalone PD path
follower (`pd_motion_planner_node`, unchanged/untouched) and a pure-pursuit
path follower, converted to a Nav2 `Controller` plugin.

## pure_pursuit: standalone node -> Nav2 Controller plugin

The pure-pursuit controller originally ran standalone: it subscribed to
`/a_star/path` and a 100ms timer that looked up the robot's pose via TF and
published directly to `/cmd_vel`. It has been rewritten as a Nav2
`nav2_core::Controller` plugin, loaded by `controller_server` via
`pluginlib`.

**What stayed the same (the actual pure-pursuit control law):**

- `selectLookaheadPose()` — closest-point search along the path, then a
  forward scan for the first pose at or beyond `lookahead_distance_`.
- `transformPose()` — yaw-only rotate/translate of a path pose into another
  frame.
- `normalizeAngle()`.
- Rotate-in-place check: if `|heading_error| > rotate_in_place_threshold_`,
  spin in place at `max_angular_velocity_`.
- Curvature calc: `curvature = 2 * sin(heading_error) / distance`.
- Turning-scale/speed shaping:
  `turning_scale = max(minimum_turning_velocity_ / linear_velocity_, 1 - |heading_error| / pi)`,
  then `linear_velocity_ * turning_scale`.
- Final angular velocity clamp to `+/- max_angular_velocity_`.
- Goal-tolerance stop check against the final path pose.
- `publishLookaheadMarker()` visualization.

**What changed (plumbing, not algorithm):**

| Aspect | Before (standalone node) | Now (Nav2 Controller plugin) |
|---|---|---|
| Path input | `/a_star/path` topic subscription (`pathCallback`) | `setPlan(path)` called directly by `controller_server` |
| Robot pose | Own TF lookup (`odom_to_base`) on a 100ms timer | Passed in directly as the `pose` argument to `computeVelocityCommands()` |
| Path-to-frame transform | Own TF lookup (`odom_to_path`) via an owned `tf2_ros::TransformListener` | Same lookup, against the `tf_` buffer handed in via `configure()` |
| Velocity output | Published to `/cmd_vel` on a timer | Returned as a `geometry_msgs::msg::TwistStamped` |
| Parameters | `declare_parameter()` on itself as an `rclcpp::Node` | Declared on the parent node, namespaced as `<plugin_name>.<param>` (e.g. `FollowPath.lookahead_distance`) |
| Lifecycle | Constructor + timer did everything | Split into `configure()` / `activate()` / `deactivate()` / `cleanup()` |
| Speed limiting | Not supported | `setSpeedLimit()` — required by the interface; scales `linear_velocity_` when Nav2's speed-limit behavior invokes it |
| Debug publishers | Plain `rclcpp::Publisher` (`/pd/next_pose`, `/pd/lookahead_marker`, `/pd/goal_reached`) | `rclcpp_lifecycle::LifecyclePublisher` (`<plugin_name>/next_pose`, `.../lookahead_marker`, `.../goal_reached`), activated/deactivated with the plugin |

### New files added

- `include/diy_motion_planner/pure_pursuit_motion_planner_node.hpp` — plugin
  class declaration.
- `plugin.xml` — pluginlib description, registers
  `diy_motion_planner::PurePursuitController` as a `nav2_core::Controller`.

### CMakeLists.txt / package.xml changes

- `pure_pursuit_motion_planner_node` executable target replaced with a
  shared library target `pure_pursuit_controller_plugin`, exported via
  `pluginlib_export_plugin_description_file()`.
- `pd_motion_planner_node` executable is untouched — still builds and
  installs exactly as before.
- Added `rclcpp_lifecycle`, `nav2_core`, `nav2_costmap_2d`, `nav2_util`,
  `pluginlib` dependencies; added the
  `<nav2_core plugin="${prefix}/plugin.xml" />` export tag.

### nav2_params.yaml

`controller_server.FollowPath.plugin` updated from
`"diy_motion_planner::pure_pursuit_motion_planner_node"` (not a real
plugin class — would have failed to load) to
`"diy_motion_planner::PurePursuitController"`. Removed parameters that no
longer apply to a Controller plugin (`map_yaml`, `use_sim_time`,
`base_frame`, `odom_frame`, `robot_clearance`, `cmd_vel_topic`,
`use_rviz`) and kept the ones the algorithm actually uses
(`lookahead_distance`, `linear_velocity`, `max_angular_velocity`,
`rotate_in_place_threshold`, `minimum_turning_velocity`, `goal_tolerance`).

### Deprecated launch file

`launch/pure_pursuit_navigation.launch.py`'s `Node(package='diy_motion_planner',
executable='pure_pursuit_motion_planner_node', ...)` block is commented out
— that executable no longer exists. Reviving this standalone launch file
would require a small bridge node between the `FollowPath` action interface
and a `/a_star/path`-style topic.

## Verified (2026-09-15)

Ran a bare `controller_server` configured with
`FollowPath.plugin: diy_motion_planner::PurePursuitController`:

- pluginlib created the class: `Created controller : FollowPath of type diy_motion_planner::PurePursuitController`
- `configure()` ran: `Configured pure pursuit controller "FollowPath"`
- `activate()` ran: `Activating pure pursuit controller "FollowPath"`
- `setPlan()` received a real `FollowPath` action goal's path:
  `Received path with 2 poses in frame "odom"`

Full closed-loop drive-to-goal completion wasn't exercised in this bare test
since there's no simulator/real robot moving the TF tree — that would need
Gazebo or hardware, not just `controller_server` standalone.
