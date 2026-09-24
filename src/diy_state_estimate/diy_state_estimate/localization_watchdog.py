#!/usr/bin/env python3
"""
localization_watchdog — hard-stops the robot if the fused pose (/odom)
implies a physically implausible speed.
═══════════════════════════════════════════════════════════════════════════
WHY THIS EXISTS
───────────────
Seen live (2026-09-19): after a NavigateToPose goal failed and every Nav2
recovery behavior had already stopped commanding any motion, the fused
/odom pose kept drifting for 20+ seconds at an accelerating ~0.4-0.5 m/s,
eventually reporting the robot 10+ metres outside the map with nothing in
the stack noticing or reacting -- Nav2 just kept logging "Start/Goal is
inside inflated obstacle region" / "Robot is out of bounds of the costmap"
forever. The root cause is upstream of this package (map_localizer's, now
RTAB-Map's, map->odom correction and/or FAST-LIO2's /Odometry -- both
external, closed to this repo), but NOTHING downstream was watching for
"this estimate cannot possibly be real" and cutting power. This node is that
missing check.

WHAT IT DOES
────────────
Subscribes to the fused /odom. Every message, it computes the *implied*
planar speed from the position delta since the previous message (not just
trusting twist.twist.linear.x, which would miss a pure position jump with
no matching twist report). If that implied linear speed, or the message's
own reported angular speed, stays above the configured ceiling for
`sustained_violation_time` seconds in a row, the estimate is declared
diverged:

  * latches permanently (does NOT auto-clear -- a stack whose localization
    has already gone divergent is not safe to auto-resume; restart the
    launch after the underlying estimate has been fixed/re-initialized),
  * publishes a zero Twist at `stop_publish_rate` Hz on `cmd_vel_topic`
    for as long as the process keeps running, and
  * publishes latched status on /localization_watchdog/diverged (std_msgs/
    Bool) so rqt / other nodes can key off it.

`cmd_vel_topic` defaults to /cmd_vel_raw so the zero command still passes
through nav2_collision_monitor (harmless -- zero velocity never trips a
stop/slowdown polygon) rather than racing controller_server for ownership
of the final /cmd_vel topic.

THRESHOLDS
──────────
Defaults are set relative to this robot's actual commanded limits (see
challenge_bringup/config/nav2_params.yaml FollowPath params):
  max_linear_speed   0.6 m/s   (2x linear_velocity: 0.3)
  max_angular_speed  2.0 rad/s (2x max_angular_velocity: 1.0)
Both are comfortably above real operation/noise, but well below the
~0.4-0.5 m/s runaway observed above, so the same event would trip this
watchdog.

THIS DOES NOT FIX THE ROOT CAUSE. It only prevents the robot from being
commanded to "chase" a runaway estimate once one starts. See
/memories/repo/nav2_stack.md (or ask for the live-diagnosis plan --
nav2_monitor.py + rqt_plot on /nav2_monitor/map_odom_* vs base_link_* vs
/wheel_odom) to pin down whether map_localizer or the EKF/FAST-LIO2 side
is the actual source, next time this trips.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class LocalizationWatchdog(Node):

    def __init__(self):
        super().__init__('localization_watchdog')

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_raw')
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('sustained_violation_time', 1.0)
        self.declare_parameter('startup_grace_period', 5.0)
        self.declare_parameter('stop_publish_rate', 10.0)

        p = self.get_parameter
        self._odom_topic = p('odom_topic').value
        self._cmd_vel_topic = p('cmd_vel_topic').value
        self._max_linear_speed = float(p('max_linear_speed').value)
        self._max_angular_speed = float(p('max_angular_speed').value)
        self._sustained_violation_time = float(p('sustained_violation_time').value)
        self._startup_grace_period = float(p('startup_grace_period').value)
        stop_publish_rate = float(p('stop_publish_rate').value)

        self._start_time = self.get_clock().now()
        self._last_stamp = None
        self._last_x = None
        self._last_y = None
        self._violation_start_stamp = None
        self._diverged = False

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._status_pub = self.create_publisher(Bool, '/localization_watchdog/diverged', 10)

        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 20)
        self.create_timer(1.0 / stop_publish_rate, self._on_timer)

        self.get_logger().info(
            f'localization_watchdog: watching {self._odom_topic}, tripping if '
            f'implied speed exceeds {self._max_linear_speed:g} m/s / '
            f'{self._max_angular_speed:g} rad/s for '
            f'{self._sustained_violation_time:g}s straight (after a '
            f'{self._startup_grace_period:g}s startup grace period). Holds zero '
            f'velocity on {self._cmd_vel_topic} once tripped -- latched until restart.')

    def _on_odom(self, msg: Odometry):
        if self._diverged:
            return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._last_stamp is not None:
            dt = stamp - self._last_stamp
            if dt > 1.0e-3:
                implied_linear_speed = math.hypot(x - self._last_x, y - self._last_y) / dt
                implied_angular_speed = abs(msg.twist.twist.angular.z)
                violating = (
                    implied_linear_speed > self._max_linear_speed or
                    implied_angular_speed > self._max_angular_speed)

                elapsed_since_start = (
                    (self.get_clock().now() - self._start_time).nanoseconds * 1e-9)

                if violating and elapsed_since_start >= self._startup_grace_period:
                    if self._violation_start_stamp is None:
                        self._violation_start_stamp = stamp
                    elif stamp - self._violation_start_stamp >= self._sustained_violation_time:
                        self._trip(implied_linear_speed, implied_angular_speed)
                else:
                    self._violation_start_stamp = None

        self._last_stamp = stamp
        self._last_x = x
        self._last_y = y

    def _trip(self, implied_linear_speed: float, implied_angular_speed: float):
        self._diverged = True
        self.get_logger().error(
            'LOCALIZATION DIVERGENCE DETECTED on %s: implied speed %.2f m/s / '
            '%.2f rad/s sustained for >=%.1fs. Holding zero velocity on %s and '
            'latching -- restart the localization/nav stack once the estimate '
            'has been fixed.' % (
                self._odom_topic, implied_linear_speed, implied_angular_speed,
                self._sustained_violation_time, self._cmd_vel_topic))

    def _on_timer(self):
        status = Bool()
        status.data = self._diverged
        self._status_pub.publish(status)
        if self._diverged:
            self._cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
