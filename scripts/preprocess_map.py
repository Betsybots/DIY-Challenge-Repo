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

  # Inner wall / loop comes out broken or missing (real scan gaps):
    --close-kernel 18   (default 13; watch the "KEPT" component log —
                          if two real walls suddenly merge into one, back off)

  # Inner wall/loop still broken, or shows as a "double line" (an open
  # horseshoe instead of a closed ring — the gap wasn't fully bridged):
    --inner-close-kernel 35   (default 20; only touches non-outer walls,
                                so it's safe to raise a lot)

  # A wall has a small fake spike or dent poking out of it (a stray scan
  # point got welded onto the wall by the gap-bridging step):
    --denoise-area 40   (default 20; raises the native-px size cutoff for
                          what counts as "just noise" before any blur runs)

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
  --close-kernel    Gap-bridging kernel (px) before wall selection (default: 13)
  --inner-close-kernel  2nd gap-bridging pass, non-outer walls only (default: 20)
  --denoise-area    Drop noise specks (native px area) at native res  (default: 20)
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
    3. Morphologically close (--close-kernel) to bridge real scan gaps
       (doorway shadows, missed sweeps) so a broken wall loop is one
       connected component, not several small fragments
    4. Connected components → find all black pixel blobs
    5. Keep only the N largest (--keep-walls) — these are the real walls
    6. Extract outer contour of each wall blob; Gaussian-smooth the
       contour x/y path as a 1D periodic signal (removes pixel steps)
    7. Draw smooth closed contour at configurable wall thickness
    8. Flood-fill from all 4 image borders → marks exterior (grey)
    9. Everything not wall and not exterior → free space (white)
   10. Downsample 2x → 0.025 m/px output
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
                    smooth_sigma, wall_thickness, close_kernel, inner_close_kernel,
                    denoise_area):
    """
    Drop noise specks (native res) -> upscale 4x -> Gaussian blur binary
    mask -> bridge scan gaps -> connected components -> smooth contour
    extraction -> draw closed walls -> flood-fill -> downsample 2x.

    Returns final map at 2x input resolution.
    """
    SCALE = 4

    # Drop stray noise specks (a handful of isolated scan points, not a
    # real wall) FIRST, at native resolution, before any blur/upscale. If
    # left in, they are almost touching a real wall by just a few native
    # pixels; the moment the mask is blurred, that tiny gap gets bridged
    # by the blur itself, permanently fusing the speck onto the wall. From
    # that point on no area filter can tell them apart anymore, and
    # --close-kernel/--inner-close-kernel will happily "bridge" through
    # them too, producing a lumpy fake bump/jump in an otherwise clean
    # wall. Real wall arcs are comfortably bigger than scan noise even at
    # native resolution, so filtering here (before blur can merge
    # anything) cleanly tells them apart.
    occ_native = (orig < occupied_thresh).astype(np.uint8)
    if denoise_area > 0:
        n0, labels0, stats0, _ = cv2.connectedComponentsWithStats(occ_native, connectivity=8)
        for i in range(1, n0):
            if stats0[i, cv2.CC_STAT_AREA] < denoise_area:
                occ_native[labels0 == i] = 0

    big = cv2.resize(occ_native * 255, (orig.shape[1]*SCALE, orig.shape[0]*SCALE),
                     interpolation=cv2.INTER_NEAREST)

    # Soften the jagged pixel edges
    occ_f = (big > 0).astype(np.float32)
    occ_f = gaussian_filter1d(gaussian_filter1d(occ_f, sigma=2.5, axis=0),
                               sigma=2.5, axis=1)
    occ_bin = (occ_f > 0.3).astype(np.uint8)

    # Bridge real gaps in a scanned wall (doorway shadows, single missed
    # sweeps, etc.) BEFORE labeling. Without this, a wall that is broken
    # into several separate arcs gets split into several small components,
    # and only the single largest arc survives --keep-walls filtering —
    # the rest of that same wall (e.g. most of an inner loop) is discarded
    # as "noise" even though it belongs to a real, larger wall.
    if close_kernel > 0:
        k_gap = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_kernel * SCALE, close_kernel * SCALE))
        occ_bin = cv2.morphologyEx(occ_bin, cv2.MORPH_CLOSE, k_gap)

    # Connected components on smooth mask
    n, labels, stats, _ = cv2.connectedComponentsWithStats(occ_bin, connectivity=8)
    areas = sorted([(stats[i, cv2.CC_STAT_AREA], i) for i in range(1, n)], reverse=True)

    # --- Stage 2: bridge the *remaining* gaps in the inner wall(s) -----------
    # The outer wall is almost always one clean loop already (it's the
    # single largest component). An inner wall/island is usually the one
    # still broken into several arcs, because the real gaps between those
    # arcs (e.g. an actual doorway, or a bigger scan shadow) can be WIDER
    # than the outer<->inner clearance — so a single global --close-kernel
    # can never bridge them without also welding the inner wall to the
    # outer one. Fix: peel the biggest ("outer") component off the mask
    # first, then close what's left with a bigger, independent kernel —
    # it has nothing to accidentally merge into anymore.
    if inner_close_kernel > 0 and len(areas) > 1:
        outer_id = areas[0][1]
        rest_mask = ((labels > 0) & (labels != outer_id)).astype(np.uint8)
        k_inner = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (inner_close_kernel * SCALE, inner_close_kernel * SCALE))
        rest_mask = cv2.morphologyEx(rest_mask, cv2.MORPH_CLOSE, k_inner)

        n2, labels2, stats2, _ = cv2.connectedComponentsWithStats(rest_mask, connectivity=8)
        # Re-assemble: outer wall keeps its own label; everything else is
        # relabeled (offset) from the re-merged "rest" pass.
        merged_labels = np.zeros_like(labels)
        merged_labels[labels == outer_id] = outer_id
        offset = int(labels.max()) + 1
        areas = [(stats[outer_id, cv2.CC_STAT_AREA], outer_id)]
        for i in range(1, n2):
            new_id = offset + i
            merged_labels[labels2 == i] = new_id
            areas.append((stats2[i, cv2.CC_STAT_AREA], new_id))
        areas.sort(reverse=True)
        labels = merged_labels

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
        # Morphological closing to seal hairline gaps before contour extraction.
        # Safe to use a generous kernel here: each component mask is fully
        # isolated at this point (stage 2 already merged/separated wall
        # groups), so this can only round off/seal *this* wall's own gaps —
        # it cannot bridge into a different wall.
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        comp_mask = cv2.morphologyEx(comp_mask, cv2.MORPH_CLOSE, k)
        # Outer contour only — naturally drops dangling artifact extensions.
        # If the component is now a true closed ring (has an enclosed hole),
        # RETR_EXTERNAL returns just its outer edge as ONE line. If it is
        # still an open horseshoe (a real, unbridged gap remains), the
        # "outer contour" instead has to trace both sides of the stroke and
        # back, which is what causes a "double line" look — a sign the gap
        # needs a bigger --inner-close-kernel, not a smoothing issue.
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

def simple_pipeline(orig, occupied_thresh, keep_walls, min_area, close_kernel):
    """
    Connected component filter only, no smoothing, original resolution.
    """
    occupied = (orig < occupied_thresh).astype(np.uint8)

    # Bridge real scan gaps before labeling — see comment in smooth_pipeline.
    if close_kernel > 0:
        k_gap = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_kernel, close_kernel))
        occupied = cv2.morphologyEx(occupied, cv2.MORPH_CLOSE, k_gap)

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
    p.add_argument('--close-kernel',     type=int, default=13,
                   help='Gap-bridging closing kernel, in original-image px '
                        '(default 13). Bridges small real scan gaps BEFORE '
                        'wall selection so a barely-broken wall loop counts '
                        'as one component. Keep this modest — too high '
                        'welds separate walls (e.g. inner + outer) together.')
    p.add_argument('--inner-close-kernel', type=int, default=20,
                   help='Second, independent gap-bridging pass (px, default '
                        '20) applied ONLY to whatever is left after the '
                        'single largest ("outer") wall is set aside. Fixes '
                        'inner walls/islands whose real gaps are wider than '
                        'the outer<->inner clearance, without risking a '
                        'merge with the outer wall. Raise this if an inner '
                        'loop is still broken or shows as a "double line" '
                        '(a sign the loop isn\'t fully closed yet). Set 0 '
                        'to disable.')
    p.add_argument('--denoise-area',     type=int, default=20,
                   help='Drop isolated scan-noise specks smaller than this '
                        '(native px area, default 20) at native resolution, '
                        'before any blur/upscale — so --close-kernel/'
                        '--inner-close-kernel can\'t weld a stray point onto '
                        'a real wall and create a fake spike/bump/jump. '
                        'Lower it if a real thin wall fragment is being '
                        'dropped; raise it if small spikes/bumps still show '
                        'up on the final wall. Set 0 to disable.')
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
                                args.keep_walls, args.min_area,
                                args.close_kernel)
        new_res = None
    else:
        print(f"\nMode: smooth contour reconstruction  "
              f"(sigma={args.smooth_sigma}, wall_thickness={args.wall_thickness})")
        final = smooth_pipeline(original, args.occupied_thresh,
                                args.keep_walls, args.min_area,
                                args.smooth_sigma, args.wall_thickness,
                                args.close_kernel, args.inner_close_kernel,
                                args.denoise_area)
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
    print(f"  Inner wall broken   -> raise --close-kernel (currently {args.close_kernel})")
    print(f"  Separate walls merged -> lower --close-kernel (currently {args.close_kernel})")
    print(f"  Inner loop still broken / double-lined -> raise --inner-close-kernel "
          f"(currently {args.inner_close_kernel})")
    print(f"  Fake spike/dent on a wall -> raise --denoise-area (currently {args.denoise_area})")


if __name__ == '__main__':
    main()
