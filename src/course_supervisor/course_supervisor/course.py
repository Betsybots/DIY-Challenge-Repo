"""Course geometry: the driving line, progress along it, sections, pure pursuit."""
import math

import numpy as np
import yaml

MODES = ('nav2', 'wall_follower', 'path_tracker', 'stop')


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Section:
    def __init__(self, cfg, s_start):
        self.name = str(cfg['name'])
        self.mode = str(cfg.get('mode', 'nav2'))
        if self.mode not in MODES:
            raise ValueError(f'section {self.name}: mode must be one of {MODES}')
        self.lidar = bool(cfg.get('lidar', True))
        self.speed = float(cfg['speed']) if cfg.get('speed') is not None else None
        self.trigger_tag = cfg.get('trigger_tag')
        self.trigger_label = cfg.get('trigger_label')
        self.trigger_range = float(cfg.get('trigger_range', 2.0))
        self.s_start = float(s_start)


class Course:
    """Driving line (map frame) plus sections along it.

    File format (written by make_course_file):
      frame_id: map
      path: [[x, y], ...]          # densely resampled driving line
      sections:                     # in driving order
        - {name, s_start, mode, lidar, speed?, trigger_tag? | trigger_label?, trigger_range?}
    """

    def __init__(self, path_xy, sections_cfg, frame_id='map'):
        self.frame_id = frame_id
        self.xy = np.asarray(path_xy, dtype=float)
        if self.xy.ndim != 2 or self.xy.shape[0] < 2 or self.xy.shape[1] != 2:
            raise ValueError('path must be a list of at least two [x, y] points')
        seg = np.diff(self.xy, axis=0)
        self.s = np.r_[0.0, np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))]
        d = np.vstack([seg, seg[-1:]])
        self.yaw = np.arctan2(d[:, 1], d[:, 0])
        self.length = float(self.s[-1])
        self.sections = [Section(c, c['s_start']) for c in sections_cfg]
        if not self.sections:
            raise ValueError('course needs at least one section')
        starts = [sec.s_start for sec in self.sections]
        if any(b < a for a, b in zip(starts, starts[1:])):
            raise ValueError('sections must be in driving order (increasing s_start)')

    @classmethod
    def load(cls, path):
        with open(path) as f:
            cfg = yaml.safe_load(f)
        return cls(cfg['path'], cfg['sections'], cfg.get('frame_id', 'map'))

    def section_index(self, s):
        idx = 0
        for i, sec in enumerate(self.sections):
            if s >= sec.s_start:
                idx = i
        return idx

    def point_at(self, s):
        s = min(max(s, 0.0), self.length)
        i = int(np.clip(np.searchsorted(self.s, s, side='right') - 1, 0, len(self.s) - 2))
        t = (s - self.s[i]) / max(self.s[i + 1] - self.s[i], 1e-9)
        p = self.xy[i] + t * (self.xy[i + 1] - self.xy[i])
        return float(p[0]), float(p[1]), float(self.yaw[i])

    def project(self, x, y, s_lo, s_hi):
        """Closest point on the path with s in [s_lo, s_hi]. Returns (s, distance)."""
        i0 = max(int(np.searchsorted(self.s, s_lo, side='left')) - 1, 0)
        i1 = min(int(np.searchsorted(self.s, s_hi, side='right')), len(self.s) - 1)
        if i1 <= i0:
            i1 = min(i0 + 1, len(self.s) - 1)
        a = self.xy[i0:i1]
        b = self.xy[i0 + 1:i1 + 1]
        ab = b - a
        L2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
        t = np.clip(((np.array([x, y]) - a) * ab).sum(axis=1) / L2, 0.0, 1.0)
        q = a + t[:, None] * ab
        d = np.hypot(q[:, 0] - x, q[:, 1] - y)
        k = int(np.argmin(d))
        s = self.s[i0 + k] + t[k] * math.sqrt(L2[k])
        return float(min(max(s, s_lo), s_hi)), float(d[k])


class Progress:
    """Monotonic progress along the course.

    The robot's pose is projected only onto the stretch [s - back, s + ahead],
    so parallel lanes elsewhere on the course can never capture it, and s
    never moves backwards by more than `back`.
    """

    def __init__(self, course, back=0.5, ahead=2.5, max_offset=1.5):
        self.course = course
        self.back, self.ahead, self.max_offset = back, ahead, max_offset
        self.s = 0.0
        self.offset = 0.0
        self.lost = False

    def update(self, x, y):
        s, d = self.course.project(x, y, max(self.s - self.back, 0.0), self.s + self.ahead)
        self.offset = d
        self.lost = d > self.max_offset
        if not self.lost:
            self.s = max(self.s, s)
        return self.s

    def jump_to(self, s):
        if s > self.s:
            self.s = s

    def reset(self):
        """Start a new lap: the course line ends next to where it starts."""
        self.s = 0.0
        self.offset = 0.0
        self.lost = False


def pure_pursuit(course, x, y, yaw, s, speed, lookahead, min_turn_radius):
    """Twist (v, w) that steers toward the path point `lookahead` metres ahead."""
    tx, ty, _ = course.point_at(s + lookahead)
    dx, dy = tx - x, ty - y
    c, sn = math.cos(yaw), math.sin(yaw)
    bx, by = c * dx + sn * dy, -sn * dx + c * dy
    L2 = max(bx * bx + by * by, 1e-6)
    curvature = 2.0 * by / L2
    heading_err = math.atan2(by, bx)
    v = speed * max(0.3, math.cos(heading_err))
    w = v * curvature
    if min_turn_radius > 0.0:
        w_max = v / min_turn_radius
        w = max(-w_max, min(w_max, w))
    return v, w
