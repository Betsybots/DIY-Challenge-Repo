#!/usr/bin/env python3
"""
sim_telemetry.py — Live telemetry dashboard for sim navigation pipeline.

Shows 4 live plots:
  1. Robot XY trajectory on the arena map (with zone circles + waypoints)
  2. Linear and angular velocity vs time
  3. Nav mode state timeline
  4. Odom pose (x, y, yaw) vs time

Usage (with sim + sim_nav running):
    source install/setup.bash
    python3 src/diy_sim/scripts/sim_telemetry.py

Dependencies: rclpy, matplotlib (both available in the ROS 2 Humble env)
"""

import math
import threading
import time

import matplotlib
matplotlib.use('TkAgg')          # use TkAgg so it works without a display server
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# ── Arena geometry (for background map) ────────────────────────────────────
ARENA_WALLS = [
    # (x1, y1, x2, y2, label)
    # Leg 1 outer walls
    (0, 0.525,  13, 0.525,  'L1 N wall'),
    (0, -0.525, 13, -0.525, 'L1 S wall'),
    # Leg 2 outer walls
    (12.475, 0,   12.475, -13, 'L2 W wall'),
    (13.525, 0,   13.525, -13, 'L2 E wall'),
    # Leg 3 outer walls
    (0,  -12.475, 13, -12.475, 'L3 N wall'),
    (0,  -13.525, 13, -13.525, 'L3 S wall'),
    # Leg 4 outer walls
    (-1.0, 0,   -1.0, -13, 'L4 W wall'),
    (0.0,  0,    0.0, -13, 'L4 E wall'),
]

# Zone definitions matching zone_waypoints.yaml
ZONES = [
    {'label': 'ramp',            'x':  6.6,  'y':  0.0,  'r': 1.8,  'color': 'gold'},
    {'label': 'narrow_section',  'x': 13.0,  'y': -4.0,  'r': 1.8,  'color': 'orange'},
    {'label': 'bucket_obstacles','x':  8.0,  'y':-13.0,  'r': 2.5,  'color': 'salmon'},
    {'label': 'tunnel',          'x': -0.5,  'y': -6.5,  'r': 1.0,  'color': 'mediumpurple'},
]

FINISH_LINE = {'x': -0.5, 'y': 0.0, 'r': 1.0}

# Nav mode → colour for state timeline
MODE_COLORS = {
    'NORMAL_NAV':       'steelblue',
    'SLOW_NAV':         'gold',
    'NAVIGATE_AROUND':  'orange',
    'BLIND_DRIVE':      'mediumpurple',
    'DONE':             'limegreen',
    'INIT':             'lightgray',
}

MAX_HISTORY = 600   # seconds of data to keep


class TelemetryNode(Node):
    def __init__(self):
        super().__init__('sim_telemetry')
        self.lock = threading.Lock()

        self.xs, self.ys, self.yaws = [], [], []
        self.vx_hist, self.wz_hist, self.t_vel = [], [], []
        self.mode_hist  = []   # list of (t_start, t_end, mode_str)
        self._cur_mode  = 'INIT'
        self._mode_start = time.time()
        self.t0 = time.time()

        self.create_subscription(Odometry, '/odom',     self._odom_cb,    10)
        self.create_subscription(Twist,    '/cmd_vel',  self._vel_cb,     10)
        self.create_subscription(String,   '/nav_mode', self._mode_cb,    10)

        self.get_logger().info('Telemetry node started — subscribing to /odom /cmd_vel /nav_mode')

    def _now(self):
        return time.time() - self.t0

    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        with self.lock:
            self.xs.append(x);  self.ys.append(y);  self.yaws.append(yaw)

    def _vel_cb(self, msg):
        t = self._now()
        with self.lock:
            self.vx_hist.append(msg.linear.x)
            self.wz_hist.append(msg.angular.z)
            self.t_vel.append(t)
            # trim old history
            while self.t_vel and (t - self.t_vel[0]) > MAX_HISTORY:
                self.t_vel.pop(0); self.vx_hist.pop(0); self.wz_hist.pop(0)

    def _mode_cb(self, msg):
        mode = msg.data
        t = self._now()
        with self.lock:
            if mode != self._cur_mode:
                self.mode_hist.append((self._mode_start, t, self._cur_mode))
                self._cur_mode  = mode
                self._mode_start = t


def draw_arena(ax):
    """Draw arena walls and zones on the trajectory axes."""
    for x1, y1, x2, y2, _ in ARENA_WALLS:
        ax.plot([x1, x2], [y1, y2], 'k-', linewidth=1.5, zorder=1)
    for z in ZONES:
        c = plt.Circle((z['x'], z['y']), z['r'], color=z['color'],
                        alpha=0.25, zorder=2)
        ax.add_patch(c)
        ax.text(z['x'], z['y'], z['label'], ha='center', va='center',
                fontsize=7, color='dimgray', zorder=3)
    # finish line
    fl = plt.Circle((FINISH_LINE['x'], FINISH_LINE['y']), FINISH_LINE['r'],
                     color='limegreen', alpha=0.35, zorder=2)
    ax.add_patch(fl)
    ax.text(FINISH_LINE['x'], FINISH_LINE['y']+1.3, 'START/FINISH',
            ha='center', fontsize=7, color='green', zorder=3)
    ax.set_xlim(-3, 16);  ax.set_ylim(-15, 3)
    ax.set_aspect('equal');  ax.set_xlabel('X (m)');  ax.set_ylabel('Y (m)')
    ax.set_title('Robot Trajectory')
    ax.grid(True, alpha=0.3)


def main():
    rclpy.init()
    node = TelemetryNode()

    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle('DIY Robot Sim Telemetry', fontsize=13, fontweight='bold')
    ax_traj, ax_vel, ax_mode, ax_pose = axes.flatten()

    draw_arena(ax_traj)
    traj_line, = ax_traj.plot([], [], 'b-', linewidth=1, alpha=0.6, zorder=4)
    robot_dot, = ax_traj.plot([], [], 'ro', markersize=7, zorder=5)
    heading_arrow = [None]

    ax_vel.set_title('Velocity');  ax_vel.set_xlabel('Time (s)')
    ax_vel.set_ylabel('m/s  |  rad/s')
    vx_line,  = ax_vel.plot([], [], 'b-',  label='linear.x (m/s)')
    wz_line,  = ax_vel.plot([], [], 'r--', label='angular.z (rad/s)')
    ax_vel.legend(fontsize=8);  ax_vel.grid(True, alpha=0.3)

    ax_mode.set_title('Nav Mode');  ax_mode.set_xlabel('Time (s)')
    ax_mode.set_yticks([])
    ax_mode.grid(True, alpha=0.2)

    ax_pose.set_title('Odom Pose');  ax_pose.set_xlabel('Time (s)')
    ax_pose.set_ylabel('m  |  rad')
    px_line, = ax_pose.plot([], [], 'b-',  label='x (m)')
    py_line, = ax_pose.plot([], [], 'g-',  label='y (m)')
    pyaw_line,= ax_pose.plot([], [], 'r--', label='yaw (rad)')
    ax_pose.legend(fontsize=8);  ax_pose.grid(True, alpha=0.3)

    def update(_):
        with node.lock:
            xs = list(node.xs);  ys = list(node.ys);  yaws = list(node.yaws)
            tv = list(node.t_vel)
            vx = list(node.vx_hist);  wz = list(node.wz_hist)
            mode_hist = list(node.mode_hist)
            cur_mode  = node._cur_mode
            mode_start = node._mode_start

        t_now = time.time() - node.t0

        # ── Trajectory ────────────────────────────────────────────────────
        if xs:
            traj_line.set_data(xs, ys)
            robot_dot.set_data([xs[-1]], [ys[-1]])
            # heading arrow
            if heading_arrow[0]:
                heading_arrow[0].remove()
            dx = 0.8 * math.cos(yaws[-1])
            dy = 0.8 * math.sin(yaws[-1])
            heading_arrow[0] = ax_traj.annotate(
                '', xy=(xs[-1]+dx, ys[-1]+dy), xytext=(xs[-1], ys[-1]),
                arrowprops=dict(arrowstyle='->', color='red', lw=2), zorder=6)

        # ── Velocity ──────────────────────────────────────────────────────
        if tv:
            vx_line.set_data(tv, vx);  wz_line.set_data(tv, wz)
            ax_vel.set_xlim(max(0, tv[-1]-60), tv[-1]+1)
            ymax = max(0.5, max(abs(v) for v in vx+wz) * 1.2)
            ax_vel.set_ylim(-ymax, ymax)

        # ── Nav mode timeline ─────────────────────────────────────────────
        ax_mode.cla()
        ax_mode.set_title('Nav Mode');  ax_mode.set_xlabel('Time (s)')
        ax_mode.set_yticks([]);  ax_mode.grid(True, alpha=0.2)
        all_modes = mode_hist + [(mode_start, t_now, cur_mode)]
        t_window_start = max(0, t_now - 120)
        for (ts, te, m) in all_modes:
            if te < t_window_start:
                continue
            ts = max(ts, t_window_start)
            color = MODE_COLORS.get(m, 'lightblue')
            ax_mode.barh(0, te - ts, left=ts, height=0.5,
                         color=color, edgecolor='white')
            ax_mode.text((ts+te)/2, 0, m, ha='center', va='center',
                         fontsize=8, fontweight='bold', color='black')
        ax_mode.set_xlim(t_window_start, t_now + 5)
        ax_mode.set_ylim(-0.5, 0.5)

        # ── Odom pose ─────────────────────────────────────────────────────
        if xs:
            n = len(xs)
            t_pose = np.linspace(max(0, t_now - MAX_HISTORY), t_now, n)
            px_line.set_data(t_pose, xs)
            py_line.set_data(t_pose, ys)
            pyaw_line.set_data(t_pose, yaws)
            ax_pose.set_xlim(max(0, t_now-60), t_now+1)
            all_vals = xs + ys + yaws
            ymin = min(all_vals[-200:]) - 0.5
            ymax = max(all_vals[-200:]) + 0.5
            ax_pose.set_ylim(ymin, ymax)

        fig.canvas.draw_idle()

    from matplotlib.animation import FuncAnimation
    ani = FuncAnimation(fig, update, interval=200, cache_frame_data=False)

    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
