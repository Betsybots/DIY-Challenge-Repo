#!/usr/bin/env python3
"""
zone_nav_logger.py — Standalone CSV logger for zone nav debugging.

Subscribes to all key zone nav topics and writes a timestamped CSV log
for post-run analysis in PlotJuggler, Excel, or pandas.

Usage:
    python3 scripts/zone_nav_logger.py

Output: ~/bags/zone_nav_log_<TIMESTAMP>.csv

CSV columns:
    wall_time_s, ros_time_s, nav_mode, mux_mode, estop_active,
    ndt_fitness, odom_x, odom_y, odom_yaw_deg, speed_limit_ms,
    cmd_vel_linear_x, cmd_vel_angular_z, event
"""

import csv
import math
import os
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry


# ── Helpers ──────────────────────────────────────────────────────────────────

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


CSV_COLUMNS = [
    "wall_time_s",
    "ros_time_s",
    "nav_mode",
    "mux_mode",
    "estop_active",
    "ndt_fitness",
    "odom_x",
    "odom_y",
    "odom_yaw_deg",
    "speed_limit_ms",
    "cmd_vel_linear_x",
    "cmd_vel_angular_z",
    "event",
]

# Throttle period for high-frequency topics (seconds)
THROTTLE_INTERVAL = 1.0 / 5.0  # 5 Hz


# ── Main Node ─────────────────────────────────────────────────────────────────

class ZoneNavLogger(Node):
    def __init__(self, csv_path: Path):
        super().__init__("zone_nav_logger")

        # ── State ────────────────────────────────────────────────────────────
        self._nav_mode: str = ""
        self._mux_mode: str = ""
        self._estop_active: bool = False
        self._ndt_fitness: float = float("nan")
        self._odom_x: float = float("nan")
        self._odom_y: float = float("nan")
        self._odom_yaw_deg: float = float("nan")
        self._speed_limit_ms: float = float("nan")
        self._cmd_vel_linear_x: float = float("nan")
        self._cmd_vel_angular_z: float = float("nan")

        # Track last write time for throttled topics
        self._last_throttled_write: float = 0.0

        # Track whether /nav_mode has been seen (startup gating)
        self._nav_mode_seen: bool = False

        # ── CSV output ───────────────────────────────────────────────────────
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = open(csv_path, "w", newline="")
        self._writer = csv.DictWriter(self._csv_file, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._csv_file.flush()
        self.get_logger().info(f"Logging to {csv_path}")

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(String,    "/nav_mode",        self._cb_nav_mode,        10)
        self.create_subscription(String,    "/mux_mode",        self._cb_mux_mode,        10)
        self.create_subscription(Bool,      "/estop_active",    self._cb_estop,           10)
        self.create_subscription(Bool,      "/green_light",     self._cb_green_light,     10)
        self.create_subscription(Float64,   "/ndt_fitness_score", self._cb_ndt,           10)
        self.create_subscription(Odometry,  "/odometry/filtered", self._cb_odom,          10)
        self.create_subscription(SpeedLimit, "/speed_limit",    self._cb_speed_limit,     10)
        self.create_subscription(Twist,     "/cmd_vel_zone_nav", self._cb_cmd_vel,        10)

        print("WAITING... watching for /nav_mode (and other zone nav topics).")
        print(f"Output: {csv_path}\n")

    # ── Callbacks: event topics (always write immediately) ────────────────────

    def _cb_nav_mode(self, msg: String):
        new_val = msg.data
        changed = new_val != self._nav_mode
        self._nav_mode = new_val
        if not self._nav_mode_seen:
            self._nav_mode_seen = True
            print("[zone_nav_logger] /nav_mode received — logging active.")
        event = f"nav_mode→{new_val}" if changed else ""
        self._write_row(event=event, force=changed)

    def _cb_mux_mode(self, msg: String):
        new_val = msg.data
        changed = new_val != self._mux_mode
        self._mux_mode = new_val
        event = f"mux_mode→{new_val}" if changed else ""
        self._write_row(event=event, force=changed)

    def _cb_estop(self, msg: Bool):
        new_val = msg.data
        changed = new_val != self._estop_active
        self._estop_active = new_val
        event = f"estop_active={new_val}" if changed else ""
        self._write_row(event=event, force=changed)

    def _cb_green_light(self, msg: Bool):
        if msg.data:
            self._write_row(event="green_light", force=True)

    # ── Callbacks: throttled topics ───────────────────────────────────────────

    def _cb_ndt(self, msg: Float64):
        self._ndt_fitness = msg.data
        self._write_row(throttled=True)

    def _cb_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        self._odom_x = pos.x
        self._odom_y = pos.y
        self._odom_yaw_deg = quat_to_yaw(msg.pose.pose.orientation)
        self._write_row(throttled=True)

    def _cb_speed_limit(self, msg: SpeedLimit):
        self._speed_limit_ms = msg.speed_limit
        self._write_row(throttled=True)

    def _cb_cmd_vel(self, msg: Twist):
        self._cmd_vel_linear_x = msg.linear.x
        self._cmd_vel_angular_z = msg.angular.z
        self._write_row(throttled=True)

    # ── Row writer ────────────────────────────────────────────────────────────

    def _write_row(self, event: str = "", force: bool = False, throttled: bool = False):
        """Write one CSV row.

        - Event rows (force=True) are always written and printed.
        - Throttled rows are written at most at THROTTLE_INTERVAL.
        - Startup gating: skip throttled rows until /nav_mode is seen.
        """
        if throttled and not self._nav_mode_seen:
            return

        now = time.monotonic()
        if throttled and not force:
            if now - self._last_throttled_write < THROTTLE_INTERVAL:
                return
            self._last_throttled_write = now

        ros_time_s = self.get_clock().now().nanoseconds * 1e-9

        row = {
            "wall_time_s":        f"{time.time():.6f}",
            "ros_time_s":         f"{ros_time_s:.6f}",
            "nav_mode":           self._nav_mode,
            "mux_mode":           self._mux_mode,
            "estop_active":       self._estop_active,
            "ndt_fitness":        _fmt(self._ndt_fitness),
            "odom_x":             _fmt(self._odom_x),
            "odom_y":             _fmt(self._odom_y),
            "odom_yaw_deg":       _fmt(self._odom_yaw_deg),
            "speed_limit_ms":     _fmt(self._speed_limit_ms),
            "cmd_vel_linear_x":   _fmt(self._cmd_vel_linear_x),
            "cmd_vel_angular_z":  _fmt(self._cmd_vel_angular_z),
            "event":              event,
        }
        self._writer.writerow(row)

        if event:
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] EVENT: {event}")

    def flush_and_close(self):
        self._csv_file.flush()
        self._csv_file.close()
        print("\n[zone_nav_logger] Log closed.")


def _fmt(val: float) -> str:
    """Format float; return empty string for NaN (field not yet received)."""
    return "" if math.isnan(val) else f"{val:.6f}"


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = Path.home() / "bags" / f"zone_nav_log_{timestamp}.csv"

    rclpy.init()
    node = ZoneNavLogger(csv_path)

    def _shutdown(sig, frame):
        print("\n[zone_nav_logger] Caught signal — shutting down...")
        node.flush_and_close()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.flush_and_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
