#!/usr/bin/env python3
"""
waypoint_recorder_node.py — record waypoints live during an offline mapping
drive, using the RPi's joystick trigger axes as record/undo controls.

WHY THIS EXISTS:
Post-hoc waypoint picking (pick_waypoints.py) requires a finished, cleaned-up
map — but during an offline mapping drive itself (FAST-LIO2 + RTAB-Map, see
challenge_master.launch.py's mapping mode), the operator already knows
exactly where the meaningful course waypoints are while driving past them.
This node lets them mark those spots live, using the same physical
controller already driving the robot (driveStack's manual.launch.py) — no
second pass, no separate mapping-then-clicking workflow.

POSE SOURCE: a live TF lookup of map_frame -> base_frame, NOT a raw odometry
topic. During mapping, RTAB-Map broadcasts a loop-closure-corrected
map->odom TF (see rtabmap_mapping_launch.py, replaces the old loop_pgo
package), composed with FAST-LIO2's own odom->base_link TF
(laserMapping.cpp) -- together giving the best available pose estimate at
each instant, already in the same "map" frame convention every other
waypoints YAML in this repo uses. Subscribing to /Odometry directly would
give the same numbers only before any loop closure correction has happened.

CONTROLS (Logitech F710, D-mode -- see driveStack's joystick.yaml for the
full axis/button layout reference):
  LT (axis 2 by default) -- record a waypoint at the robot's current pose.
  RT (axis 5 by default) -- remove the most recently recorded waypoint.
      Pressing it again with nothing left to remove is a safe no-op (never
      goes negative, never crashes) -- and holding it down or mashing it
      only ever removes ONE waypoint per physical press-release cycle
      (edge-triggered with hysteresis, see _TriggerEdge below), never more.

/joy (sensor_msgs/msg/Joy) is expected to already be visible on the Jetson
from the RPi's joy_node -- confirm with `ros2 topic echo /joy` before
relying on this (plain ROS 2 DDS discovery across the LAN, no bridge
needed as of the Zenoh-bridge removal).

OUTPUT: written after every record/remove (crash-safe -- never only at
shutdown) AND one final time on shutdown, to --output_file, in the exact
same schema as maps/test_waypoints.yaml (frame_id/loop/label+x+y+yaw) so it
works directly with diy_waypoint_sequencer with no conversion step.

Usage:
    ros2 run diy_waypoint_sequencer waypoint_recorder_node \
        --ros-args -p output_file:=/absolute/path/to/recorded_waypoints.yaml
"""

import math
import sys

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Joy
from tf2_ros import Buffer, TransformListener


class _TriggerEdge:
    """Hysteresis-debounced edge detector for one analog trigger axis.

    Released ~= 1.0, pressed ~= -1.0 (Logitech F710 D-mode convention).
    Fires exactly once per physical press, no matter how long it's held or
    how noisy the analog signal is near either threshold -- requires a full
    return above release_threshold before it will arm for the next press.
    """

    def __init__(self, press_threshold=-0.5, release_threshold=0.5):
        self._press_threshold = press_threshold
        self._release_threshold = release_threshold
        self._armed = True

    def update(self, value: float) -> bool:
        """Returns True exactly once per press, on the press edge."""
        if self._armed and value <= self._press_threshold:
            self._armed = False
            return True
        if not self._armed and value >= self._release_threshold:
            self._armed = True
        return False


class WaypointRecorderNode(Node):

    def __init__(self):
        super().__init__('waypoint_recorder_node')

        self.declare_parameter('output_file', '')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('record_axis_index', 2)   # LT
        self.declare_parameter('remove_axis_index', 5)   # RT
        self.declare_parameter('press_threshold', -0.5)
        self.declare_parameter('release_threshold', 0.5)
        self.declare_parameter('loop', False)

        self.output_file = self.get_parameter('output_file').value
        if not self.output_file:
            self.get_logger().fatal(
                'output_file parameter is empty — nothing to save to. '
                'Set it via the output_file launch argument/parameter.'
            )
            raise SystemExit(1)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.record_axis_index = int(self.get_parameter('record_axis_index').value)
        self.remove_axis_index = int(self.get_parameter('remove_axis_index').value)
        self.loop = bool(self.get_parameter('loop').value)

        press_threshold = float(self.get_parameter('press_threshold').value)
        release_threshold = float(self.get_parameter('release_threshold').value)
        self._record_edge = _TriggerEdge(press_threshold, release_threshold)
        self._remove_edge = _TriggerEdge(press_threshold, release_threshold)

        self._waypoints = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        joy_topic = self.get_parameter('joy_topic').value
        self.joy_sub = self.create_subscription(Joy, joy_topic, self._joy_cb, 10)

        self.get_logger().info(
            f'waypoint_recorder_node ready. LT (axis {self.record_axis_index}) '
            f'records, RT (axis {self.remove_axis_index}) removes the last '
            f'one. Listening on {joy_topic}, reading pose from TF '
            f'{self.map_frame} -> {self.base_frame}. Output: {self.output_file}'
        )

    # ── Joystick handling ────────────────────────────────────────────────

    def _joy_cb(self, msg: Joy):
        axes = msg.axes

        if self.remove_axis_index < len(axes):
            if self._remove_edge.update(axes[self.remove_axis_index]):
                self._remove_last_waypoint()

        if self.record_axis_index < len(axes):
            if self._record_edge.update(axes[self.record_axis_index]):
                self._record_waypoint()

    # ── Recording ────────────────────────────────────────────────────────

    def _record_waypoint(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except Exception as exc:
            self.get_logger().error(
                f'LT pressed but could not look up {self.map_frame} -> '
                f'{self.base_frame}: {exc} — waypoint NOT recorded.'
            )
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self._waypoints.append({'x': x, 'y': y, 'yaw': yaw})
        self._save()
        self.get_logger().info(
            f'Recorded waypoint {len(self._waypoints)}: '
            f'({x:.3f}, {y:.3f}), yaw={yaw:.3f}'
        )

    def _remove_last_waypoint(self):
        if not self._waypoints:
            self.get_logger().warn(
                'RT pressed but there are no waypoints to remove — no-op.'
            )
            return

        removed = self._waypoints.pop()
        self._save()
        self.get_logger().info(
            f'Removed waypoint {len(self._waypoints) + 1}: '
            f"({removed['x']:.3f}, {removed['y']:.3f}). "
            f'{len(self._waypoints)} remaining.'
        )

    # ── Saving ───────────────────────────────────────────────────────────

    def _save(self):
        data = {
            'frame_id': self.map_frame,
            'loop': self.loop,
            'waypoints': [
                {
                    'label': f'wp{i + 1}',
                    'x': round(wp['x'], 4),
                    'y': round(wp['y'], 4),
                    'yaw': round(wp['yaw'], 4),
                }
                for i, wp in enumerate(self._waypoints)
            ],
        }
        try:
            with open(self.output_file, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)
        except OSError as exc:
            self.get_logger().error(f'Failed to write {self.output_file}: {exc}')


def main():
    rclpy.init()
    try:
        node = WaypointRecorderNode()
    except SystemExit:
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save()
        node.get_logger().info(
            f'Final save: {len(node._waypoints)} waypoint(s) written to '
            f'{node.output_file}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
