# planning

Nav2 `GlobalPlanner` plugin providing a fast, custom A* path planner for the
DIY Challenge robot. Originally an `ament_python` node; converted to a C++
`ament_cmake` package and then integrated as a Nav2 plugin.

## History / what changed

### 1. Python → C++ (ament_python → ament_cmake)

- Removed the old Python scaffolding: `setup.py`, `setup.cfg`, `resource/`,
  and the `diy_planning/a_star_planner_node.py` module.
- Added `CMakeLists.txt` (C++17) and ported the node logic to
  `src/a_star_planner_node.cpp`.
- `package.xml`: `buildtool_depend`/`export` switched from `ament_python` to
  `ament_cmake`; `rclpy` replaced with `rclcpp`.
- The A* algorithm itself (8-connected search, `heapq`-style priority queue
  with a deterministic tie-breaker, octile heuristic, circular obstacle
  inflation, diagonal corner-cut prevention, path reconstruction) was
  translated as-is — no logic changes in this step.

### 2. Split into header + source

- Added `include/planning/a_star_planner_node.hpp` with the class/struct
  declarations.
- `src/a_star_planner_node.cpp` now only contains method definitions.
- `CMakeLists.txt` updated to add `include/` to the target's include path
  and install the header directory.

### 3. Standalone node → Nav2 `GlobalPlanner` plugin

The node originally ran standalone: it subscribed to `/map` and
`/goal_pose` directly, looked up the robot pose via TF itself, and published
the result on `/a_star/path`. It has been rewritten as a Nav2
`nav2_core::GlobalPlanner` plugin, loaded by `planner_server` via
`pluginlib`.

**What stayed the same (the actual A* logic):**

- `buildSafeGrid()` — circular inflation offsets, inflate-every-blocked-cell
  loop, border-within-clearance-radius-is-unsafe rule.
- The search itself — `g_score` / `came_from` / `closed_set`, the
  `(f_score, insertion_counter, x, y)` priority queue ordering, the octile
  heuristic, and diagonal corner-cut prevention are unchanged.
- Path reconstruction and grid-to-world conversion (cell-center sampling)
  are unchanged.

**What changed (plumbing, not algorithm):**

| Aspect | Before (standalone node) | Now (Nav2 plugin) |
|---|---|---|
| Obstacle data | `/map` (`nav_msgs/OccupancyGrid`) subscription | `costmap_ros_->getCostmap()` (`nav2_costmap_2d::Costmap2D`), handed in via `configure()` |
| Trigger | `goal_callback()` on `/goal_pose` | `createPlan(start, goal)` called directly by `planner_server` |
| Output | Published to `/a_star/path` | Returned as a `nav_msgs::msg::Path` |
| Start pose | Looked up itself via `tf2_ros::Buffer::lookupTransform` | Passed in directly as `start` by `planner_server` |
| Parameters | `declare_parameter()` on itself as an `rclcpp::Node` | Declared on the parent node, namespaced as `<plugin_name>.<param>` |
| Lifecycle | Constructor did all setup | Split into `configure()` / `activate()` / `deactivate()` / `cleanup()` |
| Occupied-cell threshold | `occupied_threshold` default `50` (OccupancyGrid 0–100 scale) | `occupied_threshold` default `253` (Costmap2D 0–255 scale, `INSCRIBED_INFLATED_OBSTACLE`) |
| Debug visualization | `/a_star/visited_map` (`OccupancyGrid` publisher) | `<plugin_name>/visited_map` (`LifecyclePublisher`, transient-local) |

### New files added

- `include/planning/a_star_planner_node.hpp` — plugin class declaration.
- `plugin.xml` — pluginlib description, registers
  `nav2_astar_planner::AStarPlanner` as a `nav2_core::GlobalPlanner`.
- `CMakeLists.txt` — builds `planning_astar_plugin` as a shared library
  (not an executable) and exports the pluginlib description via
  `pluginlib_export_plugin_description_file()`.

### package.xml dependency changes

Added `rclcpp_lifecycle`, `nav2_core`, `nav2_costmap_2d`, `nav2_util`,
`pluginlib`; added the `<nav2_core plugin="${prefix}/plugin.xml" />` export
tag so `planner_server` can discover the plugin.

## Remaining work

- Wire `nav2_astar_planner::AStarPlanner` into `nav2_params.yaml`'s
  `planner_server.plugins` list (in `challenge_bringup`).
- Update/remove the launch files that still reference the old
  `package='planning', executable='a_star_planner_node'` (that executable
  no longer exists — this is now a plugin library, not a node).
- Verify `pluginlib` can discover/load the class at runtime.
- Test `createPlan()` end-to-end through Nav2's `planner_server`.
