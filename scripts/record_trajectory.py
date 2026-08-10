#!/usr/bin/env python3
"""
record_trajectory.py
Records robot trajectory from /odom and saves a plot + CSV.

Usage:
  # Terminal 1: run normally, Ctrl+C to stop and save
  python3 scripts/record_trajectory.py

  # With custom output path:
  python3 scripts/record_trajectory.py --out /tmp/run1

  # Auto-stop after N seconds:
  python3 scripts/record_trajectory.py --duration 60

Output:
  <out>.csv   — raw data: time, x, y, yaw_deg, vx, wz
  <out>.png   — trajectory plot with:
                 • path coloured by speed (blue=fast, red=slow/stuck)
                 • stuck zones highlighted (speed < 0.02 m/s for > 1s)
                 • waypoints overlaid from YAML if found
                 • track walls drawn
"""

import argparse
import csv
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# ── Track walls (from simple_loop.sdf) ──────────────────────────────────────
WALLS = [
    # (x0, y0, x1, y1, label)
    (-1.0,  0.525,  13.525,  0.525, "l1_n"),
    (-1.0, -0.525,  13.525, -0.525, "l1_s"),
    (13.525,  0.525, 13.525, -13.525, "l2_e"),
    (12.475,  0.525, 12.475, -13.525, "l2_w"),
    (-1.0, -13.525, 13.525, -13.525, "l3_s"),
    (-1.0, -12.475, 13.525, -12.475, "l3_n"),
    (-1.0,   0.525,  -1.0,  -13.525, "l4_w"),
    ( 0.0,   0.525,   0.0,  -13.525, "l4_e"),
]

STUCK_SPEED   = 0.02   # m/s — below this = stuck
STUCK_MIN_SEC = 1.0    # seconds below STUCK_SPEED before marking as stuck


class TrajectoryRecorder(Node):
    def __init__(self, duration=None):
        super().__init__("trajectory_recorder")
        self.records = []          # list of (t, x, y, yaw_deg, vx, wz)
        self.start_t = None
        self.duration = duration
        self.sub = self.create_subscription(Odometry, "/odom", self._cb, 20)
        self.get_logger().info("Recording trajectory — Ctrl+C to stop and save")

    def _cb(self, msg: Odometry):
        now = time.monotonic()
        if self.start_t is None:
            self.start_t = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        vx  = msg.twist.twist.linear.x
        wz  = msg.twist.twist.angular.z

        self.records.append((now - self.start_t, p.x, p.y,
                              math.degrees(yaw), vx, wz))

        if self.duration and (now - self.start_t) >= self.duration:
            self.get_logger().info(f"Duration {self.duration}s reached — stopping")
            raise KeyboardInterrupt


def save_csv(records, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "x", "y", "yaw_deg", "vx", "wz"])
        w.writerows(records)
    print(f"CSV saved: {path}")


def save_plot(records, path, wp_file=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.collections as mc
        import numpy as np
    except ImportError:
        print("matplotlib not installed — skipping plot. Run: pip install matplotlib")
        return

    if len(records) < 2:
        print("Not enough data to plot")
        return

    arr = np.array(records)   # columns: t, x, y, yaw, vx, wz
    xs, ys, vxs = arr[:, 1], arr[:, 2], arr[:, 4]

    fig, ax = plt.subplots(figsize=(14, 12))
    ax.set_aspect("equal")
    ax.set_title("Robot Trajectory\nColour = speed (blue=fast, red=slow/stuck)", fontsize=13)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    # ── Draw track walls ────────────────────────────────────────────────────
    for x0, y0, x1, y1, lbl in WALLS:
        ax.plot([x0, x1], [y0, y1], "k-", linewidth=3, alpha=0.5)

    # ── Colour path by speed ─────────────────────────────────────────────────
    speeds = np.abs(vxs)
    norm_speeds = np.clip(speeds / 0.5, 0, 1)

    points  = np.array([xs, ys]).T.reshape(-1, 1, 2)
    segs    = np.concatenate([points[:-1], points[1:]], axis=1)
    colors  = plt.cm.RdYlBu(norm_speeds[:-1])
    lc = mc.LineCollection(segs, colors=colors, linewidths=2)
    ax.add_collection(lc)

    # ── Mark stuck zones ─────────────────────────────────────────────────────
    stuck_xs, stuck_ys = [], []
    i, n = 0, len(records)
    while i < n:
        if speeds[i] < STUCK_SPEED:
            j = i
            while j < n and speeds[j] < STUCK_SPEED:
                j += 1
            duration = arr[j-1, 0] - arr[i, 0]
            if duration >= STUCK_MIN_SEC:
                cx = np.mean(xs[i:j])
                cy = np.mean(ys[i:j])
                stuck_xs.append(cx)
                stuck_ys.append(cy)
                ax.annotate(f"STUCK\n{duration:.1f}s",
                            (cx, cy), fontsize=8, color="red",
                            ha="center",
                            bbox=dict(boxstyle="round,pad=0.2", fc="yellow", alpha=0.7))
            i = j
        else:
            i += 1

    if stuck_xs:
        ax.scatter(stuck_xs, stuck_ys, c="red", s=120, zorder=5, label="Stuck zone")

    # ── Start / end markers ──────────────────────────────────────────────────
    ax.plot(xs[0],  ys[0],  "g^", markersize=12, zorder=6, label="Start")
    ax.plot(xs[-1], ys[-1], "rs", markersize=12, zorder=6, label="End")

    # ── Heading arrows every N points ────────────────────────────────────────
    step = max(1, len(records) // 40)
    yaws_rad = np.radians(arr[::step, 3])
    ax.quiver(xs[::step], ys[::step],
              np.cos(yaws_rad), np.sin(yaws_rad),
              scale=25, width=0.003, alpha=0.4, color="gray")

    # ── Overlay waypoints if YAML provided ───────────────────────────────────
    if wp_file and os.path.isfile(wp_file):
        try:
            import yaml
            with open(wp_file) as f:
                data = yaml.safe_load(f)
            for wp in data.get("waypoints", []):
                ax.plot(wp["x"], wp["y"], "b*", markersize=14, zorder=7)
                ax.annotate(wp.get("label", ""), (wp["x"], wp["y"]),
                            textcoords="offset points", xytext=(5, 5),
                            fontsize=8, color="blue")
        except Exception as e:
            print(f"Could not load waypoints: {e}")

    # ── Colourbar ────────────────────────────────────────────────────────────
    sm = plt.cm.ScalarMappable(cmap="RdYlBu",
                               norm=plt.Normalize(vmin=0, vmax=0.5))
    sm.set_array([])
    plt.colorbar(sm, ax=ax, label="Speed (m/s)", shrink=0.6)

    ax.legend(loc="upper left")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"Plot saved: {path}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out",      default="/tmp/trajectory",
                        help="Output path prefix (no extension)")
    parser.add_argument("--duration", type=float, default=None,
                        help="Auto-stop after N seconds")
    parser.add_argument("--waypoints", default=None,
                        help="Path to waypoints YAML to overlay on plot")
    args = parser.parse_args()

    # Auto-detect waypoints file
    wp_file = args.waypoints
    if wp_file is None:
        candidate = os.path.join(
            os.path.dirname(__file__), "..",
            "src/diy_zone_nav/config/sim_simple_loop_waypoints.yaml")
        if os.path.isfile(candidate):
            wp_file = os.path.realpath(candidate)

    rclpy.init()
    node = TrajectoryRecorder(duration=args.duration)

    def _shutdown(sig, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

    records = node.records
    print(f"\nRecorded {len(records)} samples over "
          f"{records[-1][0]:.1f}s" if records else "\nNo data recorded")

    if records:
        save_csv(records, args.out + ".csv")
        save_plot(records, args.out + ".png", wp_file=wp_file)
        print(f"\nOpen plot: xdg-open {args.out}.png")


if __name__ == "__main__":
    main()
