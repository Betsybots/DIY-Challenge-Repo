#!/usr/bin/env python3
"""
waypoint_sequencer_node.py — Automated goal sequencer
═══════════════════════════════════════════════════════════════════════════
Sends one waypoint at a time from a simple YAML list, advancing each time
the previous goal is reached. This exists so a competition run doesn't need
a human clicking "2D Goal Pose" in RViz for every leg of the course.

TWO INTERCHANGEABLE MODES (use_nav2_action param, default false):
─────────────────────────────────────────────────────────────────────────────
  use_nav2_action:=false (default) — TOPIC MODE, for the custom A*/PD stack:
    Publishes /goal_pose (geometry_msgs/PoseStamped) and advances on
    /pd/goal_reached (std_msgs/Bool). See INPUTS/OUTPUT below. This is the
    ONLY thing in the custom A*/PD stack that assumes anything about "zones"
    or automated sequencing — deliberately kept as its own, separate,
    minimal package (diy_waypoint_sequencer), NOT part of zone_nav, which has
    a much larger surface (BLIND_DRIVE tunnel handling, Nav2 costmap/speed-
    limit modulation) that this simpler A*+PD stack has never consumed
    (confirmed: no reference to /nav_mode or /speed_limit anywhere in
    planning or motion_planner).

    If this node is not run at all in this mode:
      - a_star_planner_node simply never receives a /goal_pose message and
        idles (confirmed in its own goal_callback — no timeout, no crash, no
        warning spam, just waits).
      - /goal_pose can still be published manually, e.g. RViz's "2D Goal
        Pose" tool, or:
          ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
            "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}}}"
      - Nothing else in the pipeline changes behavior — this node has no
        other side effects (no service calls, no parameter changes
        elsewhere).

  use_nav2_action:=true — NAV2 ACTION MODE, for bt_navigator (Nav2):
    Drives nav2_msgs/action/NavigateToPose (default action name
    'navigate_to_pose') directly with an rclpy action client — the same
    action RViz's "Nav2 Goal" tool and bt_navigator's own topic-based
    /goal_pose bridge ultimately call into. Nav2 does NOT publish anything
    resembling /pd/goal_reached; goal completion is the action's own
    terminal GoalStatus (SUCCEEDED / ABORTED / CANCELED), delivered via the
    result callback, so that's what triggers sending the next waypoint. If a
    goal is rejected or finishes as anything other than SUCCEEDED, this node
    logs the failure and halts the sequence (no silent skips) — restart the
    node (or click a manual "Nav2 Goal" in RViz) once the underlying problem
    is fixed. /goal_pose and /pd/goal_reached are unused in this mode.

INPUTS:
  /green_light        (std_msgs/Bool)  — competition start trigger (same
                       topic zone_nav's manager uses — see
                       docs/reuse_plan_step1.md for why this is std_msgs/Bool,
                       edge-triggered on data:true). Optional — see
                       wait_for_green_light param. Used in BOTH modes.
  /pd/goal_reached     (std_msgs/Bool)  — TOPIC MODE ONLY. Published by
                       pd_motion_planner_node / pure_pursuit_motion_planner_node
                       the instant the robot is within goal_tolerance of the
                       current goal.

OUTPUT:
  /goal_pose  (geometry_msgs/PoseStamped) — TOPIC MODE ONLY. Same topic
              a_star_planner_node already subscribes to, and the same topic
              RViz's "2D Goal Pose" tool publishes — this node is just an
              alternate, automatic publisher of that same topic, fully
              interchangeable with manual goal-setting.
"""

import sys
from math import cos, sin

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool

try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    # Fallback for environments where tf_transformations is unavailable.
    def quaternion_from_euler(roll, pitch, yaw):
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy
        return (x, y, z, w)


class WaypointSequencerNode(Node):

    def __init__(self):
        super().__init__('waypoint_sequencer_node')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('goal_frame_id', 'map')
        self.declare_parameter('wait_for_green_light', True)
        self.declare_parameter('start_delay_s', 1.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('loop_count', -1)
        # false (default): TOPIC MODE, unchanged /goal_pose + /pd/goal_reached
        # behavior for the custom A*/PD stack. true: NAV2 ACTION MODE, drives
        # nav2_msgs/action/NavigateToPose directly — see module docstring.
        self.declare_parameter('use_nav2_action', False)
        self.declare_parameter('nav2_action_name', 'navigate_to_pose')
        self.declare_parameter('nav2_action_server_timeout', 10.0)

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
        # -1 (default) = loop forever if loop:true, same as always. A
        # positive value caps the total number of full passes through the
        # waypoint list before stopping (e.g. 3 = run the whole circuit 3
        # times then idle, instead of forever). Only meaningful when
        # loop:true — ignored (single pass) when loop:false regardless.
        # -1 on the launch arg means "not overridden here" -- fall back to
        # the YAML's own loop_count: key, same pattern as loop above.
        self._loop_count_override = int(self.get_parameter('loop_count').value)
        self.use_nav2_action = bool(self.get_parameter('use_nav2_action').value)
        self.nav2_action_name = self.get_parameter('nav2_action_name').value
        self.nav2_action_server_timeout = float(
            self.get_parameter('nav2_action_server_timeout').value
        )

        if not waypoints_file:
            self.get_logger().fatal(
                'waypoints_file parameter is empty — nothing to sequence. '
                'Set it via the waypoints_file launch argument.'
            )
            raise SystemExit(1)

        self.waypoints, self.loop, self.loop_count = self._load_waypoints(waypoints_file)
        if not self.waypoints:
            self.get_logger().fatal(
                f'No waypoints loaded from {waypoints_file} — check the file.'
            )
            raise SystemExit(1)

        if self.loop_count > 0:
            self.get_logger().info(
                f'Looping enabled, capped at {self.loop_count} lap(s).'
                if self.loop else
                'loop_count is set but loop is false — ignored (single pass).'
            )
        elif self.loop:
            self.get_logger().info('Looping enabled, unlimited laps.')

        self._index = 0
        self._started = False
        self._laps_completed = 0
        self._finished = False
        self._nav2_goal_handle = None

        if self.use_nav2_action:
            self._nav2_client = ActionClient(
                self, NavigateToPose, self.nav2_action_name
            )
            self.get_logger().info(
                f"use_nav2_action=true — driving Nav2's "
                f"'{self.nav2_action_name}' action directly; /goal_pose and "
                "/pd/goal_reached are unused."
            )
        else:
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
            return [], False, -1

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
        # -1 on the launch arg means "use the YAML's own loop_count: key
        # instead" (same override precedence pattern as loop above);
        # anything else on the launch arg wins outright.
        if self._loop_count_override != -1:
            loop_count = self._loop_count_override
        else:
            loop_count = int(data.get('loop_count', -1))
        return waypoints, loop, loop_count

    # ── Start triggers ───────────────────────────────────────────────────

    def _green_light_cb(self, msg: Bool):
        if msg.data and not self._started:
            self.get_logger().info('GREEN LIGHT — starting waypoint sequence.')
            self._start_once()

    def _start_once(self):
        if self._started:
            return
        self._started = True
        self._dispatch_current_waypoint()

    # ── Sequencing ───────────────────────────────────────────────────────
    # Common to both modes: TOPIC MODE's _goal_reached_cb and NAV2 ACTION
    # MODE's _nav2_result_cb both funnel into _advance_to_next_waypoint on
    # success, which figures out the next index (or loop/finish) and then
    # hands off to _dispatch_current_waypoint for the active mode.

    def _goal_reached_cb(self, msg: Bool):
        if not msg.data:
            return
        self._advance_to_next_waypoint()

    def _advance_to_next_waypoint(self):
        if not self._started or self._finished:
            return
        self._index += 1
        if self._index >= len(self.waypoints):
            if self.loop:
                self._laps_completed += 1
                if self.loop_count > 0 and self._laps_completed >= self.loop_count:
                    self._finished = True
                    self.get_logger().info(
                        f'Completed {self._laps_completed}/{self.loop_count} '
                        'lap(s) — sequence complete, idling (no more '
                        'goals will be sent).'
                    )
                    return
                lap_desc = (f'{self._laps_completed}/{self.loop_count}'
                            if self.loop_count > 0
                            else f'{self._laps_completed} (unlimited)')
                self.get_logger().info(
                    f'Lap {lap_desc} complete — looping back to waypoint 0.'
                )
                self._index = 0
            else:
                self._finished = True
                self.get_logger().info(
                    'All waypoints reached. Sequence complete — idling '
                    '(no more goals will be sent).'
                )
                return
        self._dispatch_current_waypoint()

    def _dispatch_current_waypoint(self):
        if self.use_nav2_action:
            self._send_current_waypoint_nav2()
        else:
            self._publish_current_waypoint()

    def _build_goal_pose(self, wp):
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
        return msg

    # ── TOPIC MODE dispatch ──────────────────────────────────────────────

    def _publish_current_waypoint(self):
        wp = self.waypoints[self._index]
        self.goal_pub.publish(self._build_goal_pose(wp))
        self.get_logger().info(
            f"Publishing waypoint {self._index + 1}/{len(self.waypoints)} "
            f"'{wp['label']}': ({wp['x']:.2f}, {wp['y']:.2f}, yaw={wp['yaw']:.2f})"
        )

    # ── NAV2 ACTION MODE dispatch ────────────────────────────────────────
    # Sends nav2_msgs/action/NavigateToPose directly and waits for its
    # terminal GoalStatus (Nav2 has no /pd/goal_reached-equivalent topic).
    # On anything other than SUCCEEDED, the sequence halts rather than
    # silently skipping or retrying — investigate bt_navigator/controller
    # logs, then restart this node (or send a manual "Nav2 Goal" in RViz)
    # once the underlying problem is fixed.

    def _send_current_waypoint_nav2(self):
        wp = self.waypoints[self._index]

        if not self._nav2_client.wait_for_server(
            timeout_sec=self.nav2_action_server_timeout
        ):
            self.get_logger().error(
                f"'{self.nav2_action_name}' action server not available "
                f"after {self.nav2_action_server_timeout:.1f}s — is "
                "bt_navigator running and lifecycle-activated? Halting "
                f"sequence before waypoint {self._index + 1}/"
                f"{len(self.waypoints)}."
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_goal_pose(wp)
        self.get_logger().info(
            f"Sending waypoint {self._index + 1}/{len(self.waypoints)} "
            f"'{wp['label']}' to '{self.nav2_action_name}': "
            f"({wp['x']:.2f}, {wp['y']:.2f}, yaw={wp['yaw']:.2f})"
        )
        send_goal_future = self._nav2_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._nav2_goal_response_cb)

    def _nav2_goal_response_cb(self, future):
        goal_handle = future.result()
        wp = self.waypoints[self._index]
        if not goal_handle.accepted:
            self.get_logger().error(
                f"Nav2 rejected waypoint {self._index + 1}/"
                f"{len(self.waypoints)} '{wp['label']}' — halting sequence."
            )
            return
        self._nav2_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav2_result_cb)

    def _nav2_result_cb(self, future):
        status = future.result().status
        wp = self.waypoints[self._index]
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Nav2 reached waypoint {self._index + 1}/"
                f"{len(self.waypoints)} '{wp['label']}'."
            )
            self._advance_to_next_waypoint()
        else:
            self.get_logger().error(
                f"Nav2 did not reach waypoint {self._index + 1}/"
                f"{len(self.waypoints)} '{wp['label']}' "
                f"(GoalStatus={status}) — halting sequence, no further "
                "waypoints will be sent."
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
