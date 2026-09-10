#!/usr/bin/env python3
"""
waypoint_sequencer_node.py — Automated /goal_pose sequencer
═══════════════════════════════════════════════════════════════════════════
Publishes /goal_pose (geometry_msgs/PoseStamped) one waypoint at a time from
a simple YAML list, advancing each time /pd/goal_reached fires True. This
exists so a competition run doesn't need a human clicking "2D Goal Pose" in
RViz for every leg of the course.

DESIGN GOAL — fully optional, zero hard dependents:
─────────────────────────────────────────────────────────────────────────────
This node is the ONLY thing in the custom A*/PD stack that assumes anything
about "zones" or automated sequencing. Deliberately kept as its own,
separate, minimal package (diy_waypoint_sequencer) — NOT part of
diy_zone_nav, which has a much larger surface (BLIND_DRIVE tunnel handling,
Nav2 costmap/speed-limit modulation) that this simpler A*+PD stack has never
consumed (confirmed: no reference to /nav_mode or /speed_limit anywhere in
diy_planning or diy_motion_planner).

If this node is not run at all:
  - a_star_planner_node simply never receives a /goal_pose message and idles
    (confirmed in its own goal_callback — no timeout, no crash, no warning
    spam, just waits).
  - /goal_pose can still be published manually, e.g. RViz's "2D Goal Pose"
    tool, or:
      ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}}}"
  - Nothing else in the pipeline changes behavior — this node has no other
    side effects (no service calls, no parameter changes elsewhere).

INPUTS:
  /green_light        (std_msgs/Bool)  — competition start trigger (same
                       topic diy_zone_nav's manager uses — see
                       docs/reuse_plan_step1.md for why this is std_msgs/Bool,
                       edge-triggered on data:true). Optional — see
                       wait_for_green_light param.
  /pd/goal_reached     (std_msgs/Bool)  — published by pd_motion_planner_node
                       / pure_pursuit_motion_planner_node the instant the
                       robot is within goal_tolerance of the current goal.
                       This topic did not exist before this change; both
                       controller nodes were updated to publish it.

OUTPUT:
  /goal_pose  (geometry_msgs/PoseStamped) — same topic a_star_planner_node
              already subscribes to, and the same topic RViz's "2D Goal
              Pose" tool publishes — this node is just an alternate,
              automatic publisher of that same topic, fully interchangeable
              with manual goal-setting.
"""

import sys

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from tf_transformations import quaternion_from_euler


class WaypointSequencerNode(Node):

    def __init__(self):
        super().__init__('waypoint_sequencer_node')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('wait_for_green_light', True)
        self.declare_parameter('start_delay_s', 1.0)
        self.declare_parameter('loop', False)

        waypoints_file = self.get_parameter('waypoints_file').value
        self.goal_frame_id = self.get_parameter('goal_frame_id').value
        self.wait_for_green_light = bool(
            self.get_parameter('wait_for_green_light').value
        )
        self.start_delay_s = float(self.get_parameter('start_delay_s').value)
        # Falls back to the YAML's own top-level `loop:` key if the launch
        # argument is left at its ROS-parameter default (False) AND the
        # YAML explicitly sets loop: true — see _load_waypoints below.
        self._loop_override = bool(self.get_parameter('loop').value)

        if not waypoints_file:
            self.get_logger().fatal(
                'waypoints_file parameter is empty — nothing to sequence. '
                'Set it via the waypoints_file launch argument.'
            )
            raise SystemExit(1)

        self.waypoints, self.loop = self._load_waypoints(waypoints_file)
        if not self.waypoints:
            self.get_logger().fatal(
                f'No waypoints loaded from {waypoints_file} — check the file.'
            )
            raise SystemExit(1)

        self._index = 0
        self._started = False

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.goal_reached_sub = self.create_subscription(
            Bool, '/pd/goal_reached', self._goal_reached_cb, 10
        )

        if self.wait_for_green_light:
            self.green_light_sub = self.create_subscription(
                Bool, '/green_light', self._green_light_cb, 10
            )
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoint(s). '
                'Waiting for /green_light to start.'
            )
        else:
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoint(s). '
                f'wait_for_green_light=false — starting in {self.start_delay_s:.1f}s.'
            )
            self.create_timer(self.start_delay_s, self._start_once)

    # ── Waypoint file loading ────────────────────────────────────────────

    def _load_waypoints(self, path):
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().fatal(f'Failed to read {path}: {exc}')
            return [], False

        raw_waypoints = data.get('waypoints', [])
        waypoints = []
        for i, wp in enumerate(raw_waypoints):
            try:
                waypoints.append({
                    'label': wp.get('label', f'wp{i}'),
                    'x': float(wp['x']),
                    'y': float(wp['y']),
                    'yaw': float(wp.get('yaw', 0.0)),
                })
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().error(
                    f'Skipping malformed waypoint at index {i}: {exc}'
                )
        # File-level frame_id overrides the launch-arg default only if set.
        if 'frame_id' in data:
            self.goal_frame_id = data['frame_id']
        loop = self._loop_override or bool(data.get('loop', False))
        return waypoints, loop

    # ── Start triggers ───────────────────────────────────────────────────

    def _green_light_cb(self, msg: Bool):
        if msg.data and not self._started:
            self.get_logger().info('GREEN LIGHT — starting waypoint sequence.')
            self._start_once()

    def _start_once(self):
        if self._started:
            return
        self._started = True
        self._publish_current_waypoint()

    # ── Sequencing ───────────────────────────────────────────────────────

    def _goal_reached_cb(self, msg: Bool):
        if not msg.data or not self._started:
            return
        self._index += 1
        if self._index >= len(self.waypoints):
            if self.loop:
                self.get_logger().info('Course complete — looping back to waypoint 0.')
                self._index = 0
            else:
                self.get_logger().info(
                    'All waypoints reached. Sequence complete — idling '
                    '(no more /goal_pose messages will be published).'
                )
                return
        self._publish_current_waypoint()

    def _publish_current_waypoint(self):
        wp = self.waypoints[self._index]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.goal_frame_id
        msg.pose.position.x = wp['x']
        msg.pose.position.y = wp['y']
        q = quaternion_from_euler(0.0, 0.0, wp['yaw'])
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.goal_pub.publish(msg)
        self.get_logger().info(
            f"Publishing waypoint {self._index + 1}/{len(self.waypoints)} "
            f"'{wp['label']}': ({wp['x']:.2f}, {wp['y']:.2f}, yaw={wp['yaw']:.2f})"
        )


def main():
    rclpy.init()
    node = WaypointSequencerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
