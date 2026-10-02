#!/usr/bin/env python3
"""Draw the obstacle-course waypoints and AprilTags on the course drawing.

Fits the 2D map (walls) onto the drawing (yellow barriers) with a rotation +
translation at true scale, then plots waypoint_sequencer/config/waypoints.yaml,
course_supervisor/config/tag_map_obstacle_course.yaml (with labels from
aprilTag/config/tag_labels.yaml) and the course driving line.

  python3 draw_overlay.py            # writes course_waypoints_tags.png next to this file
"""
import math
import os
import sys

import cv2
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, os.path.join(SRC, 'course_supervisor'))
from course_supervisor.course import Course  # noqa: E402

DRAW = os.path.join(HERE, 'course_layout_2026.png')
MPP = 65 * 0.3048 / 885.0                    # drawing: 65 ft dimension line spans ~885 px
MAP = os.path.join(SRC, 'diy_localization', 'map', 'Obstacle_course', 'obstacle_course_clean.yaml')

dimg = np.array(Image.open(DRAW).convert('RGB')).astype(int)
r, g, b = dimg[..., 0], dimg[..., 1], dimg[..., 2]
yel = cv2.morphologyEx(((r - b > 60) & (g - b > 30)).astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)) > 0
dt = distance_transform_edt(~yel)
meta = yaml.safe_load(open(MAP))
pgm = np.array(Image.open(os.path.join(os.path.dirname(MAP), meta['image'])))
H, W = pgm.shape
res, ox, oy = meta['resolution'], meta['origin'][0], meta['origin'][1]
course = Course.load(os.path.join(SRC, 'course_supervisor', 'config', 'obstacle_course.yaml'))


def to_d(p, xy):
    th, k, du, dv = p
    c, s = math.cos(th), math.sin(th)
    xy = np.atleast_2d(xy)
    return np.stack([du + k * (c * xy[:, 0] - s * xy[:, 1]), dv - k * (s * xy[:, 0] + c * xy[:, 1])], 1)


rr, cc = np.nonzero(pgm == 0)
F = np.c_[ox + (cc + .5) * res, oy + (H - 1 - rr + .5) * res][::2]


def cost(q, mx=12):
    uv = to_d([q[0], 1 / MPP, q[1], q[2]], F)
    return np.mean(np.minimum(map_coordinates(dt, [uv[:, 1], uv[:, 0]], order=1, mode='constant', cval=mx), mx))


grid = sorted(((cost([math.pi + a, du, dv]), [math.pi + a, du, dv]) for a in np.radians(np.arange(-8, 8.1, 1))
               for du in range(430, 571, 6) for dv in range(520, 621, 6)), key=lambda t: t[0])
best = min((minimize(cost, g0, method='Nelder-Mead', options={'xatol': 1e-4, 'fatol': 1e-4, 'maxiter': 3000})
            for _, g0 in grid[:10]), key=lambda res_: res_.fun)
p = [best.x[0], 1 / MPP, best.x[1], best.x[2]]
th = p[0]
print(f'map -> drawing fit: rotation {math.degrees(th) - 180:+.2f} deg, mean wall distance {best.fun * MPP * 100:.1f} cm')

wps = yaml.safe_load(open(os.path.join(SRC, 'waypoint_sequencer', 'config', 'waypoints.yaml')))['waypoints']
tags = yaml.safe_load(open(os.path.join(SRC, 'course_supervisor', 'config', 'tag_map_obstacle_course.yaml')))['tags'] or {}
labels = yaml.safe_load(open(os.path.join(SRC, 'aprilTag', 'config', 'tag_labels.yaml')))['tag_labels']


def d_dir(yaw):
    a = yaw + th
    return math.cos(a), -math.sin(a)


import matplotlib  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

fig, ax = plt.subplots(figsize=(20, 15))
ax.imshow(dimg.astype(np.uint8))
S = np.arange(0, course.length, 0.05)
uv = to_d(p, np.array([course.point_at(s)[:2] for s in S]))
ax.plot(uv[:, 0], uv[:, 1], '-', color='#1f77b4', lw=1.2, alpha=0.6, label='run3 driving line')
for sec in course.sections:
    q = to_d(p, course.point_at(sec.s_start)[:2])[0]
    ax.plot(*q, 'D', ms=6, mfc='orange', mec='k', zorder=4)
for i, w in enumerate(wps, 1):
    q = to_d(p, [w['x'], w['y']])[0]
    dx, dy = d_dir(w['yaw'])
    hoop = 'hoop' in w['label']
    ax.plot(*q, 'o', ms=9, mfc='#9467bd' if hoop else '#2ca02c', mec='k', zorder=5)
    ax.arrow(q[0], q[1], 14 * dx, 14 * dy, width=1.2, color='k', zorder=5)
    ax.annotate(str(i), q, xytext=(5, -7), textcoords='offset points', fontsize=10, weight='bold',
                color='#4b2675' if hoop else '#145214', zorder=6,
                bbox=dict(boxstyle='round,pad=0.1', fc='white', ec='none', alpha=0.8))
for tid, t in tags.items():
    q = to_d(p, [t['x'], t['y']])[0]
    dx, dy = d_dir(math.radians(t['facing_deg']))
    ax.plot(*q, 's', ms=13, mfc='red', mec='k', zorder=7)
    ax.arrow(q[0], q[1], 22 * dx, 22 * dy, width=2.0, color='red', zorder=7)
    ax.annotate(f"ID{tid} {labels.get(f'ID{tid}', '')}", q, xytext=(8, 8), textcoords='offset points', fontsize=10,
                weight='bold', color='darkred', zorder=8,
                bbox=dict(boxstyle='round,pad=0.15', fc='#fff3f3', ec='darkred', alpha=0.9))
ax.plot([], [], 'o', mfc='#2ca02c', mec='k', label='waypoint (number = order, arrow = heading)')
ax.plot([], [], 'o', mfc='#9467bd', mec='k', label='hoop waypoint (before / through / after)')
ax.plot([], [], 's', mfc='red', mec='k', label='AprilTag on top of the wall (arrow = face direction)')
ax.plot([], [], 'D', mfc='orange', mec='k', label='course_supervisor section start')
ax.legend(loc='lower left', fontsize=11)
ax.set_xlim(30, 1010)
ax.set_ylim(760, 20)
ax.axis('off')
ax.set_title('Obstacle course: waypoints (waypoint_sequencer) and AprilTags (course_supervisor tag map)', fontsize=13)
plt.tight_layout()
out = os.path.join(HERE, 'course_waypoints_tags.png')
plt.savefig(out, dpi=75)
print('wrote', out)
