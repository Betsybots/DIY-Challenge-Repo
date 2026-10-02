#!/usr/bin/env python3
"""
plan_tags -- compute AprilTag placements on the 2D map for course_supervisor.

Each tag is mounted on top of a lane wall (right or left), close to its inner
edge, with the printed face turned back toward the approaching robot. For each
requested tag the tool finds the wall's inner face on the 2D map at that point
of the driving line and prints:
  * the tag-map entry {x, y, z, facing_deg} for config/tag_map_obstacle_course.yaml,
  * the trigger_range to use if the tag triggers a section (distance between
    the section start and the tag along the driving line),
  * where along the course the robot can see it (within --max-range, inside the
    camera field of view, with line of sight on the map),
and optionally saves a placement figure for the team.

Plan file (YAML):
  map_yaml: <path to obstacle_course_clean.yaml>   # optional, else --map
  tags:
    - {id: 2, label: TUNNEL_EXIT, s: 19.3, side: right, height: 0.55, section: narrow_path, trigger_range: 0.6}
  # s: metres along the driving line where the tag stands
  # height: tag centre height above the floor (m)
  # section: optional, section this tag triggers (for trigger_range)
  # trigger_range: optional; default fires with the robot at the section start

Usage:
  ros2 run course_supervisor plan_tags --plan config/tag_plan_obstacle_course.yaml \\
      --map ../diy_localization/map/Obstacle_course/obstacle_course_clean.yaml \\
      --course config/obstacle_course.yaml --figure /tmp/tag_plan.png
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

from course_supervisor.course import Course

FLOOR_Z = -0.12      # map z of the floor (map origin = FAST-LIO body at the start pose)


class GridMap:
    def __init__(self, map_yaml):
        from PIL import Image
        with open(map_yaml) as f:
            meta = yaml.safe_load(f)
        self.img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), meta['image'])))
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.free_value = 254

    def cell(self, x, y):
        col = int((x - self.ox) / self.res)
        row = self.img.shape[0] - 1 - int((y - self.oy) / self.res)
        return row, col

    def is_free(self, x, y):
        row, col = self.cell(x, y)
        h, w = self.img.shape
        return 0 <= row < h and 0 <= col < w and self.img[row, col] == self.free_value

    def ray(self, x, y, dx, dy, max_d=2.0, step=0.01):
        d = 0.0
        while d < max_d:
            d += step
            if not self.is_free(x + dx * d, y + dy * d):
                return d
        return None

    def line_of_sight(self, x0, y0, x1, y1, stop_short=0.12):
        d = math.hypot(x1 - x0, y1 - y0)
        if d <= stop_short:
            return True
        n = max(int((d - stop_short) / 0.02), 1)
        for k in range(1, n + 1):
            t = k * 0.02 / d
            if not self.is_free(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t):
                return False
        return True


def plan_tag(course, grid, spec, max_range, fov_deg, inset, min_range=0.3):
    s = float(spec['s'])
    x, y, yaw = course.point_at(s)
    side = 1.0 if spec.get('side', 'right') == 'left' else -1.0
    nx, ny = -math.sin(yaw) * side, math.cos(yaw) * side           # toward the chosen wall
    wall = grid.ray(x, y, nx, ny)
    if wall is None:
        raise ValueError(f"tag {spec['id']}: no wall within 2 m on the {spec.get('side', 'right')} side at s={s}")
    inset = float(spec.get('inset', inset))
    tx, ty = x + nx * (wall + inset), y + ny * (wall + inset)         # on the wall top, inset from the face
    facing = math.degrees(yaw) + 180.0
    facing = (facing + 180.0) % 360.0 - 180.0
    z = float(spec.get('height', 0.5)) + FLOOR_Z
    # Visibility along the approach: within range, inside the camera FOV, clear line of sight to
    # the wall face just in front of the tag.
    fx, fy = x + nx * (wall - 0.02), y + ny * (wall - 0.02)
    seen = []
    for sv in np.arange(max(s - max_range - 0.5, 0.0), s, 0.05):
        px, py, pyaw = course.point_at(sv)
        d = math.hypot(tx - px, ty - py)
        if d > max_range or d < min_range:
            continue
        bearing = (math.atan2(ty - py, tx - px) - pyaw + math.pi) % (2 * math.pi) - math.pi
        if abs(math.degrees(bearing)) > fov_deg / 2:
            continue
        if not grid.line_of_sight(px, py, fx, fy):
            continue
        seen.append(sv)
    return dict(id=int(spec['id']), label=str(spec.get('label', '')), s=s, x=tx, y=ty, z=z, facing=facing,
                lane_x=x, lane_y=y, wall_dist=wall, side=spec.get('side', 'right'),
                section=spec.get('section'), seen=seen)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plan', required=True, help='Tag plan YAML')
    ap.add_argument('--course', required=True, help='Course file (obstacle_course.yaml)')
    ap.add_argument('--map', default=None, help='2D map YAML (overrides map_yaml in the plan)')
    ap.add_argument('--max-range', type=float, default=None,
                    help='Detection range (m); default: max_range in the plan file, else 1.0')
    ap.add_argument('--min-range', type=float, default=0.3, help='Closest usable detection (m)')
    ap.add_argument('--fov-deg', type=float, default=80.0, help='Usable horizontal field of view (deg)')
    ap.add_argument('--inset', type=float, default=None,
                    help='Tag centre behind the wall face (m); default: inset in the plan file, else 0.03')
    ap.add_argument('--figure', default=None, help='Save a placement figure (PNG)')
    a = ap.parse_args()

    with open(a.plan) as f:
        plan = yaml.safe_load(f)
    map_yaml = a.map or plan.get('map_yaml')
    if not map_yaml:
        sys.exit('no map: pass --map or set map_yaml in the plan')
    if not os.path.isabs(map_yaml) and a.map is None:
        map_yaml = os.path.join(os.path.dirname(os.path.abspath(a.plan)), map_yaml)
    course = Course.load(a.course)
    grid = GridMap(map_yaml)
    sections = {sec.name: sec for sec in course.sections}

    max_range = a.max_range if a.max_range is not None else float(plan.get('max_range', 1.0))
    inset = a.inset if a.inset is not None else float(plan.get('inset', 0.03))
    results = [plan_tag(course, grid, spec, max_range, a.fov_deg, inset, a.min_range) for spec in plan['tags']]
    spec_of = {int(spec['id']): spec for spec in plan['tags']}
    print('# tag_map_obstacle_course.yaml entries (planned from the map; refine with a survey)')
    print('tags:')
    for r in results:
        print(f"  {r['id']}: {{x: {r['x']:.2f}, y: {r['y']:.2f}, z: {r['z']:.2f}, facing_deg: {r['facing']:.0f}}}"
              f"   # {r['label']}")
    print()
    for r in results:
        seen = r['seen']
        vis = (f"seen from s={seen[0]:.1f} to s={seen[-1]:.1f} ({seen[-1] - seen[0]:.1f} m of approach)"
               if seen else 'NOT VISIBLE on the approach (check placement)')
        line = (f"tag {r['id']} {r['label']}: at s={r['s']:.1f} on top of the {r['side']} wall "
                f"({r['wall_dist']:.2f} m from the lane centre line); {vis}")
        if r['section']:
            sec = sections.get(r['section'])
            if sec is None:
                line += f"; unknown section {r['section']}"
            else:
                # Default: fire with the robot at the section start. An explicit trigger_range
                # in the plan fires earlier/later (robot at s_tag - trigger_range).
                rng = float(spec_of[r['id']].get('trigger_range', r['s'] - sec.s_start))
                fire_s = r['s'] - rng
                line += (f"; trigger_range for section {sec.name} (starts s={sec.s_start:.2f}): {rng:.1f}"
                         f" (fires with the robot at s={fire_s:.2f})")
                if not seen or not (seen[0] - 0.05 <= fire_s <= seen[-1] + 0.05):
                    line += ('  WARNING: tag not visible at the section start, this trigger would '
                             'never fire -- use it as a position fix only or move the tag')
        print(line)

    if a.figure:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        n = len(results)
        fig, axs = plt.subplots(1, n, figsize=(6.5 * n, 6.5), squeeze=False)
        h, w = grid.img.shape
        for ax, r in zip(axs[0], results):
            ax.imshow(grid.img, cmap='gray', vmin=0, vmax=255, interpolation='nearest',
                      extent=[grid.ox, grid.ox + w * grid.res, grid.oy, grid.oy + h * grid.res])
            ss = np.arange(max(r['s'] - 5.0, 0.0), min(r['s'] + 2.0, course.length), 0.05)
            P = np.array([course.point_at(v)[:2] for v in ss])
            ax.plot(P[:, 0], P[:, 1], 'c-', lw=2, label='driving line')
            if r['seen']:
                V = np.array([course.point_at(v)[:2] for v in r['seen']])
                ax.plot(V[:, 0], V[:, 1], '-', color='lime', lw=5, alpha=0.8, label='tag visible')
            f = math.radians(r['facing'])
            ax.plot(r['x'], r['y'], 's', color='red', ms=12, label='tag')
            ax.arrow(r['x'], r['y'], 0.45 * math.cos(f), 0.45 * math.sin(f), width=0.03, color='red')
            sx, sy, syaw = course.point_at(max(r['s'] - 4.5, 0.0))
            ax.arrow(sx, sy, 0.4 * math.cos(syaw), 0.4 * math.sin(syaw), width=0.04, color='orange')
            ax.set_xlim(r['x'] - 2.5, r['x'] + 2.5)
            ax.set_ylim(r['y'] - 2.5, r['y'] + 2.5)
            ax.set_title(f"ID{r['id']} {r['label']}\n({r['x']:.2f}, {r['y']:.2f}) facing {r['facing']:.0f} deg, "
                         f"height {r['z'] - FLOOR_Z:.2f} m", fontsize=10)
            ax.grid(alpha=0.4)
            ax.legend(loc='lower left', fontsize=8)
        plt.tight_layout()
        plt.savefig(a.figure, dpi=80)
        print(f'figure: {a.figure}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
