#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════
                         preprocess_map.py
          Clean & smooth any ROS PGM map for Nav2 map_server
═════════════════════════════════════════════════════════════════════════════

WHAT IT DOES
────────────
  Takes a raw PGM map (from a LiDAR scan or cartographer) and produces a
  clean, smooth, Nav2-ready version:
    • Removes all interior obstacle blobs (buckets, cones, scan artifacts)
    • Keeps only the real wall boundaries (the N largest black regions)
    • Smooths jagged pixel edges into clean curved wall contours
    • Flood-fills the entire driveable interior to pure white (free space)
    • Outputs at 2x resolution (0.025 m/px) for finer costmap detail

HOW TO USE — QUICK START
────────────────────────
  Step 1: Run with --preview-only to inspect blob sizes (no files written)
    python3 scripts/preprocess_map.py --input maps/your_map.pgm --preview-only

  Step 2: Look at the terminal output — real walls are 10–100x larger than
          noise blobs. Decide how many wall components to keep (usually 1 or 2).

  Step 3: Run for real
    python3 scripts/preprocess_map.py \
        --input maps/your_map.pgm     \
        --yaml  maps/your_map.yaml    \
        --keep-walls 2

  Output files written next to the input:
    your_map_smooth.pgm   ← use this with Nav2 map_server
    your_map_smooth.yaml  ← companion yaml (resolution updated to 0.025)
    your_map_diff.png     ← side-by-side before/after visual

COMMON RECIPES
──────────────
  # Simple rectangular arena with one outer wall:
    --keep-walls 1

  # Arena with outer wall + central island:
    --keep-walls 2   (default)

  # Map already clean, just want blob removal, no smoothing:
    --no-smooth

  # Walls still look jagged → increase smoothing:
    --smooth-sigma 10

  # Walls look over-smoothed / corners cut off:
    --smooth-sigma 3

  # Walls too thin (bad costmap inflation):
    --wall-thickness 12

  # Headless / SSH (no display):
    --no-display

ARGUMENTS AT A GLANCE
──────────────────────
  --input           Path to input PGM file                         (required)
  --yaml            Companion .yaml file (auto-detected if omitted)
  --keep-walls N    Keep N largest wall components             (default: 2)
  --min-area N      Alternative: keep all blobs >= N px        (use with --keep-walls 0)
  --occupied-thresh Pixel value cutoff for "occupied"          (default: 50)
  --no-smooth       Skip smoothing — blob filter + flood fill only
  --smooth-sigma    Gaussian sigma on contour path             (default: 6.0)
  --wall-thickness  Wall draw thickness at 4x scale            (default: 8)
  --preview-only    Save diff PNG, do NOT write PGM or yaml
  --no-display      Headless mode (no matplotlib window)

PGM PIXEL CONVENTION
──────────────────────
    0   = occupied (black)  — walls
    205 = unknown  (grey)   — exterior / unscanned area
    254 = free     (white)  — driveable corridor

WORKS ON ANY ROS PGM
──────────────────────
  This script makes no assumptions about arena shape or size.
  Walls just need to be closed loops — if your scan has a gap in a wall,
  use --wall-thickness 12 to help bridge it, or fix manually in GIMP.

ALGORITHM (smooth mode)
──────────────────────
    1. Upscale 4x (INTER_NEAREST) for smoothing headroom
    2. Gaussian-blur binary occupied mask to soften jagged edges
    3. Connected components → find all black pixel blobs
    4. Keep only the N largest (--keep-walls) — these are the real walls
    5. Extract outer contour of each wall blob; Gaussian-smooth the
       contour x/y path as a 1D periodic signal (removes pixel steps)
    6. Draw smooth closed contour at configurable wall thickness
    7. Flood-fill from all 4 image borders → marks exterior (grey)
    8. Everything not wall and not exterior → free space (white)
    9. Downsample 2x → 0.025 m/px output
"""

import argparse
import os
import sys

import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


FREE_VALUE    = 254
UNKNOWN_VALUE = 205
WALL_VALUE    = 0


def load_pgm(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"ERROR: cannot read '{path}'", file=sys.stderr)
        sys.exit(1)
    return img


# ─── Smooth pipeline ──────────────────────────────────────────────────────────

def smooth_pipeline(orig, occupied_thresh, keep_walls, min_area,
                    smooth_sigma, wall_thickness):
    """
    Upscale 4x -> Gaussian blur binary mask -> connected components ->
    smooth contour extraction -> draw closed walls -> flood-fill ->
    downsample 2x.

    Returns final map at 2x input resolution.
    """
    SCALE = 4

    big = cv2.resize(orig, (orig.shape[1]*SCALE, orig.shape[0]*SCALE),
                     interpolation=cv2.INTER_NEAREST)

    # Soften the jagged pixel edges
    occ_f = (big < occupied_thresh).astype(np.float32)
    occ_f = gaussian_filter1d(gaussian_filter1d(occ_f, sigma=2.5, axis=0),
                               sigma=2.5, axis=1)
    occ_bin = (occ_f > 0.3).astype(np.uint8)

    # Connected components on smooth mask
    n, labels, stats, _ = cv2.connectedComponentsWithStats(occ_bin, connectivity=8)
    areas = sorted([(stats[i, cv2.CC_STAT_AREA], i) for i in range(1, n)], reverse=True)

    if keep_walls > 0:
        kept_ids  = {i for _, i in areas[:keep_walls]}
        mode_desc = f"top {keep_walls} largest components"
    else:
        kept_ids  = {i for a, i in areas if a >= min_area}
        mode_desc = f"components >= {min_area} px"

    print(f"\n  Mode: {mode_desc}")
    for a, i in areas:
        print(f"  comp {i:3d}: {a:7d} px  {'<-- KEPT' if i in kept_ids else 'removed'}")

    canvas = np.zeros(big.shape, dtype=np.uint8)
    for _, comp_id in areas:
        if comp_id not in kept_ids:
            continue
        comp_mask = (labels == comp_id).astype(np.uint8) * 255
        # Morphological closing to seal hairline gaps before contour extraction
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        comp_mask = cv2.morphologyEx(comp_mask, cv2.MORPH_CLOSE, k)
        # Outer contour only — naturally drops dangling artifact extensions
        contours, _ = cv2.findContours(comp_mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_NONE)
        contours = sorted(contours, key=lambda c: len(c), reverse=True)
        for c in contours[:1]:
            pts = c[:, 0, :]
            xs = gaussian_filter1d(pts[:, 0].astype(float), sigma=smooth_sigma,
                                   mode='wrap')
            ys = gaussian_filter1d(pts[:, 1].astype(float), sigma=smooth_sigma,
                                   mode='wrap')
            smooth_pts = np.stack([xs, ys], axis=1).astype(np.int32).reshape(-1, 1, 2)
            cv2.polylines(canvas, [smooth_pts], isClosed=True,
                          color=255, thickness=wall_thickness)

    final = _flood_and_assemble(canvas)

    # Downsample to 2x original (0.025 m/px)
    OUT = SCALE // 2
    return cv2.resize(final, (orig.shape[1]*OUT, orig.shape[0]*OUT),
                      interpolation=cv2.INTER_NEAREST)


# ─── Simple pipeline (--no-smooth) ───────────────────────────────────────────

def simple_pipeline(orig, occupied_thresh, keep_walls, min_area):
    """
    Connected component filter only, no smoothing, original resolution.
    """
    occupied = (orig < occupied_thresh).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(occupied, connectivity=8)
    areas = sorted([(stats[i, cv2.CC_STAT_AREA], i) for i in range(1, n)], reverse=True)

    if keep_walls > 0:
        kept_ids  = {i for _, i in areas[:keep_walls]}
        mode_desc = f"top {keep_walls} largest components"
    else:
        kept_ids  = {i for a, i in areas if a >= min_area}
        mode_desc = f"components >= {min_area} px"

    print(f"\n  Mode: {mode_desc}")
    for a, i in areas:
        print(f"  comp {i:3d}: {a:6d} px  {'<-- KEPT' if i in kept_ids else 'removed'}")

    wall_mask = np.zeros(labels.shape, dtype=np.uint8)
    for _, i in areas:
        if i in kept_ids:
            wall_mask[labels == i] = 1

    canvas = np.where(wall_mask == 1, 255, 0).astype(np.uint8)
    return _flood_and_assemble(canvas)


# ─── Shared: flood-fill exterior + assemble PGM ───────────────────────────────

def _flood_and_assemble(canvas):
    h, w = canvas.shape
    ff = np.zeros((h+2, w+2), np.uint8)
    for col in range(w):
        if canvas[0,   col] == 0: cv2.floodFill(canvas, ff, (col, 0),    128)
        if canvas[h-1, col] == 0: cv2.floodFill(canvas, ff, (col, h-1),  128)
    for row in range(h):
        if canvas[row, 0]   == 0: cv2.floodFill(canvas, ff, (0,   row),  128)
        if canvas[row, w-1] == 0: cv2.floodFill(canvas, ff, (w-1, row),  128)

    final = np.full(canvas.shape, UNKNOWN_VALUE, dtype=np.uint8)
    final[canvas == 255] = WALL_VALUE
    final[canvas == 0  ] = FREE_VALUE
    final[canvas == 128] = UNKNOWN_VALUE
    return final


# ─── Yaml helpers ─────────────────────────────────────────────────────────────

def write_yaml(src_yaml, dst_yaml, new_pgm_name, new_resolution=None):
    lines_out = []
    if src_yaml and os.path.isfile(src_yaml):
        with open(src_yaml) as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith('image:'):
                    lines_out.append(f'image: {os.path.basename(new_pgm_name)}\n')
                elif new_resolution and stripped.startswith('resolution:'):
                    lines_out.append(f'resolution: {new_resolution}\n')
                else:
                    lines_out.append(line)
    else:
        res = new_resolution or 0.05
        lines_out = [
            f'image: {os.path.basename(new_pgm_name)}\n',
            f'resolution: {res}\n',
            'origin: [0.0, 0.0, 0.0]\n',
            'negate: 0\n',
            'occupied_thresh: 0.65\n',
            'free_thresh: 0.196\n',
        ]
    with open(dst_yaml, 'w') as f:
        f.writelines(lines_out)


# ─── Diff visualisation ───────────────────────────────────────────────────────

def save_diff(original, final, out_path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle('Map Preprocessing Result', fontsize=13, fontweight='bold')
    axes[0].imshow(original, cmap='gray', vmin=0, vmax=254, origin='upper')
    axes[0].set_title('Original PGM');  axes[0].axis('off')
    axes[1].imshow(final,    cmap='gray', vmin=0, vmax=254, origin='upper')
    axes[1].set_title('Preprocessed\nblack=walls  white=free  grey=outside')
    axes[1].axis('off')
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()


# ─── CLI ─────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='Clean and smooth a ROS PGM map for Nav2')
    p.add_argument('--input',            required=True)
    p.add_argument('--yaml',             default=None,
                   help='Companion yaml (auto-detected if omitted)')
    p.add_argument('--keep-walls',       type=int, default=2,
                   help='Keep N largest wall components (default 2). '
                        'Set 0 to use --min-area instead.')
    p.add_argument('--min-area',         type=int, default=500,
                   help='[--keep-walls=0] keep blobs >= this px (default 500)')
    p.add_argument('--occupied-thresh',  type=int, default=50,
                   help='Pixels darker than this = occupied (default 50)')
    p.add_argument('--no-smooth',        action='store_true',
                   help='Skip contour smoothing; blob filter only (original resolution)')
    p.add_argument('--smooth-sigma',     type=float, default=6.0,
                   help='Gaussian sigma for contour path smoothing (default 6)')
    p.add_argument('--wall-thickness',   type=int, default=8,
                   help='Wall thickness in pixels at 4x scale (default 8 = ~0.05m real)')
    p.add_argument('--no-display',       action='store_true')
    p.add_argument('--preview-only',     action='store_true',
                   help='Save diff PNG but do not write PGM/yaml')
    return p.parse_args()


def main():
    args = parse_args()

    in_pgm  = os.path.abspath(args.input)
    base    = os.path.splitext(in_pgm)[0]
    suffix  = '_clean' if args.no_smooth else '_smooth'
    out_pgm = base + suffix + '.pgm'
    out_yaml= base + suffix + '.yaml'
    out_diff= base + '_diff.png'

    src_yaml = args.yaml or (base + '.yaml')
    if not os.path.isfile(src_yaml):
        src_yaml = None

    print(f"\nLoading: {in_pgm}")
    original = load_pgm(in_pgm)
    print(f"  Size: {original.shape[1]}x{original.shape[0]} px")

    if args.no_smooth:
        print("\nMode: simple blob filter (no smoothing)")
        final = simple_pipeline(original, args.occupied_thresh,
                                args.keep_walls, args.min_area)
        new_res = None
    else:
        print(f"\nMode: smooth contour reconstruction  "
              f"(sigma={args.smooth_sigma}, wall_thickness={args.wall_thickness})")
        final = smooth_pipeline(original, args.occupied_thresh,
                                args.keep_walls, args.min_area,
                                args.smooth_sigma, args.wall_thickness)
        new_res = 0.025

    print(f"\n  Output size: {final.shape[1]}x{final.shape[0]} px")
    print(f"  Wall px    : {(final == WALL_VALUE).sum()}")
    print(f"  Interior px: {(final == FREE_VALUE).sum()}  (-> white / free)")
    print(f"  Exterior px: {(final == UNKNOWN_VALUE).sum()}  (-> grey / unknown)")

    print(f"\nSaving diff  -> {out_diff}")
    save_diff(original, final, out_diff)

    if args.preview_only:
        print("--preview-only: PGM and yaml not written.")
        return

    print(f"Saving PGM   -> {out_pgm}")
    cv2.imwrite(out_pgm, final)

    print(f"Saving yaml  -> {out_yaml}")
    write_yaml(src_yaml, out_yaml, out_pgm, new_res)

    print(f"\nDone.")
    print(f"\nUse with Nav2:")
    print(f"  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={out_yaml}")
    print(f"\nTuning tips:")
    print(f"  Wrong walls kept    -> change --keep-walls (currently {args.keep_walls})")
    print(f"  Walls too smooth    -> lower --smooth-sigma (currently {args.smooth_sigma})")
    print(f"  Walls too jagged    -> raise --smooth-sigma")
    print(f"  Walls too thin/fat  -> adjust --wall-thickness (currently {args.wall_thickness})")


if __name__ == '__main__':
    main()
