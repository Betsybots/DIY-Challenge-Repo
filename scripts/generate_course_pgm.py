#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════
                        generate_course_pgm.py
          Generate a Nav2-ready PGM map from the DIY Challenge course diagram
═════════════════════════════════════════════════════════════════════════════

HOW TO USE — QUICK START
────────────────────────
  python3 scripts/generate_course_pgm.py

  The default input is the official 2026 course diagram. All outputs land in
  maps/ alongside the existing PGM files.

CUSTOM DIAGRAM
──────────────
  python3 scripts/generate_course_pgm.py --diagram path/to/diagram.png

OUTPUTS (all in the same directory as --output prefix)
────────────────────────────────────────────────────────
  course_map.pgm        ← Nav2-ready wall map  (0.025 m/px, 65' × 48')
  course_map.yaml       ← ROS map_server YAML  (resolution, origin, thresholds)
  course_map_zones.png  ← Colour-coded zone overlay (debug / planning)
  course_map_zones.yaml ← Zone entry poses + polygons in map coordinates

OPTIONS
───────
  --diagram  PATH   Input course diagram PNG  (see default below)
  --output   PREFIX Output path prefix        (default: maps/course_map)
  --sigma    FLOAT  Gaussian smoothing sigma  (default: 5.0)
  --thick    INT    Wall draw thickness px    (default: 6)
  --no-display      Skip showing any windows
  --debug           Save intermediate step images to /tmp/

ALGORITHM
─────────
  1.  HSV colour-segment golden/yellow hatched borders → wall mask
  2.  Morphological dilation to fill hatching-pattern gaps
  3.  ADD manual walls for two borderless zones:
        • Circular helix ramp  (no golden border — only thin black lines)
        • 48" × 128" inclined plane  (open-sided; gap-closed manually)
  4.  Crop to course bounding box; scale to 0.025 m/px
  5.  Gaussian-smooth contour paths (same style as preprocess_map.py)
  6.  Flood-fill exterior → PGM pixel convention:
        0   = walls (black)
        205 = exterior / unknown (grey)
        254 = free / driveable (white)
  7.  Write PGM + YAML
  8.  Overlay zone polygons + entry-point markers → zones PNG + zones YAML

PGM PIXEL CONVENTION
─────────────────────
    0   = occupied  (walls)
    205 = unknown   (exterior / outside course)
    254 = free      (driveable corridor)

ZONE TRAVERSAL ORDER  (Start → Finish)
───────────────────────────────────────
  Start → Car Wash → Hoop → Bank/Inclined Plane → Gravel → Pothole →
  Obstacle/Wide → Narrow → Tunnel → Ramp/Helix → Finish
"""

import argparse
import os
import sys

import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


# ─── PGM pixel values ─────────────────────────────────────────────────────────
FREE_VALUE    = 254
UNKNOWN_VALUE = 205
WALL_VALUE    = 0

# ─── Course physical dimensions ───────────────────────────────────────────────
COURSE_W_FT   = 65.0
COURSE_H_FT   = 48.0
FEET_TO_M     = 0.3048
RESOLUTION    = 0.025   # metres per output pixel

# ─── Input image crop: tight box around the course in the diagram PNG ─────────
# (cols and rows in the 768 × 1003 diagram image)
CROP_ROW0 = 40
CROP_ROW1 = 705
CROP_COL0 = 110
CROP_COL1 = 960

# ─── Golden border HSV range ──────────────────────────────────────────────────
HSV_GOLD_LO = np.array([5,  40, 100])
HSV_GOLD_HI = np.array([35, 255, 255])

# ─── Manual ramp / helix circle parameters (in ORIGINAL image coordinates) ───
# The circular ramp at bottom-left uses thin black lines — not detected by the
# golden HSV mask.  We draw two concentric circles to close the walls.
#
# Measured from the diagram:
#   4' centeline radius → 52px (at ≈13 px/ft image scale)
#   41.5" = 3.46' wide track
#   inner radius ≈ 4' − 1.73' = 2.27' ≈ 30px
#   outer radius ≈ 4' + 1.73' = 5.73' ≈ 75px
#
RAMP_CENTER_IMG  = (130, 647)   # (col, row) in original image
RAMP_OUTER_R_IMG = 78
RAMP_INNER_R_IMG = 31
RAMP_WALL_T_IMG  = 8            # wall draw thickness in original image px

# ─── Interior seed points (normalized x, y) ──────────────────────────────────
# Pixels guaranteed to be inside the driveable corridor in the THICK classifier
# layer (k13 dilation only, no erosion).  The course is one connected region so
# even a single valid seed will reach the entire interior via flood fill.
# Expressed as fractions of the CROPPED + SCALED image (out_w × out_h).
INTERIOR_SEEDS_NORM = [
    (0.361, 0.853),  # Start / Finish straight      — verified free in thick layer
    (0.380, 0.087),  # Gravel top corridor           — verified free in thick layer
    (0.056, 0.873),  # Ramp / Helix track            — verified free in thick layer
    (0.571, 0.797),  # Car Wash                      — backup seed
    (0.802, 0.151),  # Bank / upper corridor         — backup seed
    (0.380, 0.315),  # Pothole / Obstacle area       — backup seed
    (0.056, 0.533),  # Narrow corridor centroid      — backup seed
    (0.140, 0.767),  # Tunnel entry area             — backup seed
]
# Polygons are expressed as normalized fractions of the CROPPED image area
# (x = col fraction 0-1 left→right, y = row fraction 0-1 top→bottom).
# At draw time they are scaled to output pixel coordinates.
#
# Traversal order: Start → Car Wash → Hoop → Bank → Gravel → Pothole →
#                  Obstacle → Wide → Narrow → Tunnel → Ramp → Finish
ZONES = [
    {
        'id':    'start',
        'label': 'Start / Finish',
        'color': (50,  205,  50),   # lime green
        'poly_norm': [(0.28, 0.74), (0.44, 0.74), (0.44, 0.89), (0.28, 0.89)],
        'entry_norm': (0.30, 0.81),  # (x, y) normalized → map pose
        'entry_yaw_deg': 0,
    },
    {
        'id':    'car_wash',
        'label': 'Car Wash',
        'color': (0,   191, 255),   # deep sky blue
        'poly_norm': [(0.44, 0.74), (0.73, 0.74), (0.73, 0.89), (0.44, 0.89)],
        'entry_norm': (0.46, 0.81),
        'entry_yaw_deg': 0,
    },
    {
        'id':    'hoop',
        'label': 'Hoop',
        'color': (255, 140,   0),   # dark orange
        'poly_norm': [(0.61, 0.50), (1.00, 0.50), (1.00, 0.82), (0.61, 0.82)],
        'entry_norm': (0.64, 0.72),
        'entry_yaw_deg': 90,
    },
    {
        'id':    'bank',
        'label': 'Bank / Inclined Plane',
        'color': (220,  20,  60),   # crimson
        'poly_norm': [(0.57, 0.01), (1.00, 0.01), (1.00, 0.34), (0.57, 0.34)],
        'entry_norm': (0.63, 0.30),
        'entry_yaw_deg': 180,
    },
    {
        'id':    'gravel',
        'label': 'Gravel',
        'color': (139,  90,  43),   # saddle brown
        'poly_norm': [(0.23, 0.01), (0.48, 0.01), (0.48, 0.19), (0.23, 0.19)],
        'entry_norm': (0.38, 0.05),
        'entry_yaw_deg': 180,
    },
    {
        'id':    'pothole',
        'label': 'Pothole',
        'color': (148,   0, 211),   # dark violet
        'poly_norm': [(0.23, 0.19), (0.53, 0.19), (0.53, 0.39), (0.23, 0.39)],
        'entry_norm': (0.38, 0.22),
        'entry_yaw_deg': 270,
    },
    {
        'id':    'obstacle',
        'label': 'Obstacle / Wide',
        'color': (30,  144, 255),   # dodger blue
        'poly_norm': [(0.19, 0.39), (0.55, 0.39), (0.55, 0.72), (0.19, 0.72)],
        'entry_norm': (0.35, 0.42),
        'entry_yaw_deg': 270,
    },
    {
        'id':    'narrow',
        'label': 'Narrow',
        'color': (255, 215,   0),   # gold
        'poly_norm': [(0.01, 0.08), (0.16, 0.08), (0.16, 0.74), (0.01, 0.74)],
        'entry_norm': (0.04, 0.12),
        'entry_yaw_deg': 270,
    },
    {
        'id':    'tunnel',
        'label': 'Tunnel',
        'color': (105, 105, 105),   # dim gray
        'poly_norm': [(0.08, 0.68), (0.20, 0.68), (0.20, 0.80), (0.08, 0.80)],
        'entry_norm': (0.12, 0.72),
        'entry_yaw_deg': 180,
    },
    {
        'id':    'ramp',
        'label': 'Ramp / Helix',
        'color': (255,  99,  71),   # tomato
        'poly_norm': [(0.00, 0.75), (0.15, 0.75), (0.15, 1.00), (0.00, 1.00)],
        'entry_norm': (0.07, 0.77),
        'entry_yaw_deg': 180,
    },
]


# ─── Helpers ──────────────────────────────────────────────────────────────────

def norm_to_px(nx, ny, out_w, out_h):
    """Convert normalized zone coords → output PGM pixel (col, row)."""
    return int(nx * out_w), int(ny * out_h)


# ─── Wall extraction ──────────────────────────────────────────────────────────

def extract_golden_walls(img, debug=False):
    """
    HSV-segment the golden/yellow hatched borders → binary wall mask.
    Returns a uint8 image (255 = wall, 0 = non-wall), same size as img.

    Two layers are built from the same HSV mask:
      • thin  (k13 dilate + k7 erode, net +6 px): used for the final PGM walls
      • thick (k13 dilate only, net +13 px):       used as classifier to separate
                interior from exterior via interior flood-fill
    """
    hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_mask = cv2.inRange(hsv, HSV_GOLD_LO, HSV_GOLD_HI)

    k13 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
    k7  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,  7))

    # Thin layer: dilate fills hatching gaps, erosion recovers wall centre
    thin  = cv2.dilate(hsv_mask, k13)
    thin  = cv2.erode (thin,     k7)

    # Thick layer: dilate only — slightly larger walls used as gap-proof classifier
    thick = cv2.dilate(hsv_mask, k13)

    if debug:
        cv2.imwrite('/tmp/dbg_thin_walls.png',  thin)
        cv2.imwrite('/tmp/dbg_thick_walls.png', thick)
        print('  [debug] saved /tmp/dbg_thin_walls.png  /tmp/dbg_thick_walls.png')

    return thin, thick


def add_ramp_circle_walls(thin, thick, img_shape):
    """
    Draw inner + outer walls of the circular helix ramp on both layers.
    The ramp has no golden border — only thin black outline in the diagram.
    Parameters are defined in original-image coordinates at the top of this file.
    """
    cx, cy = RAMP_CENTER_IMG
    t = RAMP_WALL_T_IMG
    cv2.circle(thin,  (cx, cy), RAMP_OUTER_R_IMG,     255, t)
    cv2.circle(thin,  (cx, cy), RAMP_INNER_R_IMG,     255, t)
    cv2.circle(thick, (cx, cy), RAMP_OUTER_R_IMG + 6, 255, t + 6)
    cv2.circle(thick, (cx, cy), max(1, RAMP_INNER_R_IMG - 6), 255, t + 6)
    return thin, thick


# ─── PGM assembly ─────────────────────────────────────────────────────────────

def build_course_pgm(thin_mask, thick_mask, out_w, out_h, debug=False):
    """
    Build a Nav2-ready PGM from the two-layer wall mask.

    Algorithm (two-layer interior-seed approach):
      1. Crop to course bounding box, scale to out_w × out_h
      2. Apply k9 morphological close on both layers to seal hairline gaps
      3. Add post-scale manual closures for the bank/inclined-plane section
         (which has no golden border: top wall + right wall drawn manually)
      4. Add a 3-pixel perimeter frame to the THICK layer only → forces corners
         to be walls so the course interior is fully enclosed for flood fill
      5. Seed the thick-layer interior from INTERIOR_SEEDS_NORM:
         flood fill from each seed marks reachable open pixels as interior (128)
      6. Assemble final PGM:
           thick-interior (128)  → 254  FREE    (white / driveable)
           thin  walls   (255)  →   0  WALL    (black / occupied)
           everything else       → 205  UNKNOWN (grey  / exterior)

    Pixel convention (Nav2 trinary mode):
      0   = occupied  (walls, lethal obstacle)
      205 = unknown   (exterior / outside course)
      254 = free      (driveable corridor)
    """
    k9 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

    # ── 1. Crop + scale ───────────────────────────────────────────────────────
    thin_s  = cv2.resize(thin_mask[CROP_ROW0:CROP_ROW1, CROP_COL0:CROP_COL1],
                         (out_w, out_h), interpolation=cv2.INTER_NEAREST)
    thick_s = cv2.resize(thick_mask[CROP_ROW0:CROP_ROW1, CROP_COL0:CROP_COL1],
                         (out_w, out_h), interpolation=cv2.INTER_NEAREST)

    # ── 2. Morphological close (seal hairline gaps at scaled resolution) ──────
    thin_s  = cv2.morphologyEx(thin_s,  cv2.MORPH_CLOSE, k9)
    thick_s = cv2.morphologyEx(thick_s, cv2.MORPH_CLOSE, k9)

    # ── 3. Post-scale manual closures: bank / inclined-plane section ──────────
    # The bank section has no golden border; close it with three line segments.
    # Coordinates are in the 792 × 585 scaled image.
    for m in (thin_s, thick_s):
        cv2.line(m, (500, 12), (791, 12), 255, 8)   # bank top wall
        cv2.line(m, (789, 10), (789, 343), 255, 8)  # bank right wall
        cv2.line(m, (500, 12), (500,  35), 255, 8)  # bank top-left junction

    # ── 4. Perimeter frame on THICK layer only ────────────────────────────────
    cv2.rectangle(thick_s, (0, 0), (out_w - 1, out_h - 1), 255, 3)

    if debug:
        cv2.imwrite('/tmp/dbg_thin_scaled.png',  thin_s)
        cv2.imwrite('/tmp/dbg_thick_scaled.png', thick_s)
        print('  [debug] saved /tmp/dbg_thin_scaled.png  /tmp/dbg_thick_scaled.png')

    # ── 5. Interior flood-fill on thick layer ─────────────────────────────────
    classifier = thick_s.copy()
    ff = np.zeros((out_h + 2, out_w + 2), np.uint8)
    seeded = 0
    for nx, ny in INTERIOR_SEEDS_NORM:
        cx = int(nx * out_w)
        cy = int(ny * out_h)
        # Walk ±20 px to find first non-wall pixel in the thick layer
        found = False
        for r in range(max(0, cy - 20), min(out_h, cy + 21)):
            for c in range(max(0, cx - 20), min(out_w, cx + 21)):
                if classifier[r, c] == 0:
                    cv2.floodFill(classifier, ff, (c, r), 128)
                    seeded += 1
                    found = True
                    break
            if found:
                break

    print(f'  Interior seeds placed : {seeded}/{len(INTERIOR_SEEDS_NORM)}')
    print(f'  Interior pixels (128) : {(classifier == 128).sum():,}')
    print(f'  Unclassified open (0) : {(classifier == 0).sum():,}  '
          '(isolated pockets → exterior)')

    # ── 6. Assemble final PGM ─────────────────────────────────────────────────
    final = np.full((out_h, out_w), UNKNOWN_VALUE, dtype=np.uint8)
    final[classifier == 128] = FREE_VALUE   # interior → white
    final[thin_s     == 255] = WALL_VALUE   # thin walls override interior

    return final


# ─── Zone visualization ───────────────────────────────────────────────────────

def draw_zone_overlay(pgm, zones, out_w, out_h, out_path):
    """
    Draw semi-transparent zone polygons + entry-point markers over the PGM.
    Saves a colour PNG.
    """
    fig, ax = plt.subplots(figsize=(16, 12))
    ax.imshow(pgm, cmap='gray', vmin=0, vmax=254, origin='upper')
    ax.set_title('Course Map — Zone Overlay\n'
                 'Start → Car Wash → Hoop → Bank → Gravel → Pothole → '
                 'Obstacle → Narrow → Tunnel → Ramp → Finish',
                 fontsize=10)

    legend_handles = []
    for z in zones:
        r, g, b  = z['color']
        rgba_fill = (r/255, g/255, b/255, 0.22)
        rgba_edge = (r/255, g/255, b/255, 0.90)

        # Polygon
        pts = [norm_to_px(nx, ny, out_w, out_h) for nx, ny in z['poly_norm']]
        poly = plt.Polygon(pts, closed=True,
                           facecolor=rgba_fill, edgecolor=rgba_edge,
                           linewidth=1.5)
        ax.add_patch(poly)

        # Entry-point marker
        ex, ey = norm_to_px(z['entry_norm'][0], z['entry_norm'][1], out_w, out_h)
        ax.plot(ex, ey, 'o', color=rgba_edge[:3], markersize=7,
                markeredgecolor='white', markeredgewidth=1.0)
        ax.text(ex + 4, ey - 4, z['label'], fontsize=5.5,
                color=rgba_edge[:3], fontweight='bold',
                bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.55, ec='none'))

        legend_handles.append(
            mpatches.Patch(facecolor=rgba_fill, edgecolor=rgba_edge,
                           label=z['label']))

    ax.legend(handles=legend_handles, loc='lower right',
              fontsize=6, framealpha=0.8)
    ax.axis('off')
    plt.tight_layout()
    plt.savefig(out_path, dpi=180, bbox_inches='tight')
    plt.close()
    print(f'Saved zone overlay  → {out_path}')


# ─── Zones YAML ───────────────────────────────────────────────────────────────

def write_zones_yaml(zones, out_w, out_h, yaml_path):
    """
    Write zones.yaml:  each zone has an entry pose (map coords, metres) and
    a bounding polygon (metres, for zone-estimator use).

    Map origin is at (0, 0) = bottom-left corner of the course.
    Row 0 in PGM = top of course = y = COURSE_H_FT * FEET_TO_M.
    """
    course_w_m = COURSE_W_FT * FEET_TO_M
    course_h_m = COURSE_H_FT * FEET_TO_M

    def px_to_m(nx, ny):
        mx = nx * course_w_m
        my = (1.0 - ny) * course_h_m   # ROS y grows upward
        return round(mx, 3), round(my, 3)

    lines = ['# Zone definitions for DIY Challenge 2026 course\n',
             '# entry_pose: [x_m, y_m, yaw_rad]  in the map frame\n',
             '# polygon:    list of [x_m, y_m] vertices (counter-clockwise)\n',
             '# traversal_order: 0 = first zone, 9 = last zone\n\n',
             'zones:\n']

    for order, z in enumerate(zones):
        ex, ey = px_to_m(z['entry_norm'][0], z['entry_norm'][1])
        yaw_r  = round(np.deg2rad(z['entry_yaw_deg']), 4)
        lines.append(f"  {z['id']}:\n")
        lines.append(f"    label:            \"{z['label']}\"\n")
        lines.append(f"    traversal_order:  {order}\n")
        lines.append(f"    entry_pose:       [{ex}, {ey}, {yaw_r}]\n")
        # Build polygon vertices
        verts = [px_to_m(nx, ny) for nx, ny in z['poly_norm']]
        vstr  = ', '.join(f'[{x}, {y}]' for x, y in verts)
        lines.append(f"    polygon:          [{vstr}]\n\n")

    with open(yaml_path, 'w') as f:
        f.writelines(lines)
    print(f'Saved zones YAML    → {yaml_path}')


# ─── Map YAML ─────────────────────────────────────────────────────────────────

def write_map_yaml(pgm_path, yaml_path):
    content = (
        f'image: {os.path.basename(pgm_path)}\n'
        f'resolution: {RESOLUTION}\n'
        'origin: [0.0, 0.0, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.196\n'
        'mode: trinary\n'
    )
    with open(yaml_path, 'w') as f:
        f.write(content)
    print(f'Saved map YAML      → {yaml_path}')


# ─── Main ─────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Generate Nav2 PGM from DIY Challenge course diagram')
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.dirname(here)
    default_diag = os.path.join(
        os.path.expanduser('~'),
        '.config/Code/User/globalStorage/github.copilot-chat/copilot-cli-images',
        '1787834791524-ro4cw3m9.png')
    default_out  = os.path.join(repo, 'maps', 'course_map')
    p.add_argument('--diagram',    default=default_diag,
                   help='Course diagram PNG  (default: cached Copilot attachment)')
    p.add_argument('--output',     default=default_out,
                   help='Output path prefix  (default: maps/course_map)')
    p.add_argument('--no-display', action='store_true')
    p.add_argument('--debug',      action='store_true',
                   help='Save intermediate images to /tmp/')
    return p.parse_args()


def main():
    args  = parse_args()
    diag  = os.path.abspath(args.diagram)
    out_prefix = os.path.abspath(args.output)
    out_dir    = os.path.dirname(out_prefix)
    os.makedirs(out_dir, exist_ok=True)

    out_pgm   = out_prefix + '.pgm'
    out_yaml  = out_prefix + '.yaml'
    out_zones_png  = out_prefix + '_zones.png'
    out_zones_yaml = out_prefix + '_zones.yaml'

    # ── Load diagram ──────────────────────────────────────────────────────────
    print(f'\nLoading diagram: {diag}')
    img = cv2.imread(diag)
    if img is None:
        print(f'ERROR: cannot read "{diag}"', file=sys.stderr)
        sys.exit(1)
    print(f'  Diagram size: {img.shape[1]}×{img.shape[0]} px')

    # ── Extract golden walls (thin + thick layers) ───────────────────────────
    print('\nDetecting golden wall borders …')
    thin_mask, thick_mask = extract_golden_walls(img, debug=args.debug)

    # ── Add manual ramp circle walls ──────────────────────────────────────────
    print('Adding manual ramp/helix circle walls …')
    thin_mask, thick_mask = add_ramp_circle_walls(thin_mask, thick_mask, img.shape)

    if args.debug:
        cv2.imwrite('/tmp/dbg_thin_combined.png',  thin_mask)
        cv2.imwrite('/tmp/dbg_thick_combined.png', thick_mask)

    # ── Target map dimensions ─────────────────────────────────────────────────
    out_w = int(round(COURSE_W_FT * FEET_TO_M / RESOLUTION))
    out_h = int(round(COURSE_H_FT * FEET_TO_M / RESOLUTION))
    print(f'\nTarget map size: {out_w}×{out_h} px  '
          f'({RESOLUTION} m/px, {COURSE_W_FT}×{COURSE_H_FT} ft)')

    # ── Build PGM (two-layer interior-seed approach) ──────────────────────────
    print('\nBuilding course PGM …')
    final = build_course_pgm(thin_mask, thick_mask, out_w, out_h,
                             debug=args.debug)

    print(f'\n  Output size  : {final.shape[1]}×{final.shape[0]} px')
    print(f'  Wall pixels  : {(final == WALL_VALUE).sum():,}')
    print(f'  Free pixels  : {(final == FREE_VALUE).sum():,}')
    print(f'  Exterior px  : {(final == UNKNOWN_VALUE).sum():,}')
    print(f'  Exterior px  : {(final == UNKNOWN_VALUE).sum():,}')

    # ── Write PGM + map YAML ──────────────────────────────────────────────────
    cv2.imwrite(out_pgm, final)
    print(f'\nSaved PGM           → {out_pgm}')
    write_map_yaml(out_pgm, out_yaml)

    # ── Zone overlay + zones YAML ─────────────────────────────────────────────
    print('\nGenerating zone overlay …')
    draw_zone_overlay(final, ZONES, out_w, out_h, out_zones_png)
    write_zones_yaml(ZONES, out_w, out_h, out_zones_yaml)

    # ── Summary ───────────────────────────────────────────────────────────────
    print('\n' + '═' * 60)
    print('  Done!  Outputs:')
    print(f'    {out_pgm}')
    print(f'    {out_yaml}')
    print(f'    {out_zones_png}')
    print(f'    {out_zones_yaml}')
    print('═' * 60)
    print('\nUse with Nav2:')
    print(f'  ros2 run nav2_map_server map_server --ros-args '
          f'-p yaml_filename:={out_yaml}')


if __name__ == '__main__':
    main()
