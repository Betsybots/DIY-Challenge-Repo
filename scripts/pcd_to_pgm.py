#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════
                             pcd_to_pgm.py
     Project a FAST-LIO2 (or any XYZ) .pcd point cloud into a raw,
     Nav2-ready occupancy-grid PGM
═════════════════════════════════════════════════════════════════════════════

WHY THIS EXISTS
───────────────
FAST-LIO2's own /map_save service (see fast_lio_ros2/src/laserMapping.cpp)
already saves the accumulated map as a 3-D point cloud .pcd — that's the
format map_localizer's VGICP needs and it works today with no extra step.

But nav2_map_server / the A* planner need a 2-D occupancy-grid PGM, and
until now there was no tool in this repo that goes from a real lidar-built
.pcd to that PGM — scripts/generate_course_pgm.py builds a PGM from a
hand-drawn course DIAGRAM image, not from real sensor data, and
scripts/preprocess_map.py only cleans up a PGM that already exists. This
script is the missing first step: raw 3-D scan -> raw 2-D PGM. Feed its
output into preprocess_map.py afterwards for the wall-smoothing/denoise
pass — this script deliberately does NOT do any of that cleanup itself, to
avoid duplicating already-tested code.

HOW TO USE — QUICK START
────────────────────────
  Step 1: Drive the course with FAST-LIO2 running, then save the map:
    ros2 service call /map_save std_srvs/srv/Trigger {}
    (or just Ctrl-C the fastlio_mapping node — it saves on shutdown too)

  Step 2: Inspect the point cloud's height distribution to pick a wall
          slice (skip this if you already know your robot/course geometry):
    python3 scripts/pcd_to_pgm.py --input maps/fastlio_raw.pcd --preview-only

  Step 3: Convert, using a z-range that isolates walls from floor/ceiling:
    python3 scripts/pcd_to_pgm.py \
        --input maps/fastlio_raw.pcd \
        --output maps/fastlio_raw \
        --z-min 0.15 --z-max 1.2

  Step 4: Clean it up with the existing preprocessing pipeline:
    python3 scripts/preprocess_map.py \
        --input maps/fastlio_raw.pgm \
        --yaml  maps/fastlio_raw.yaml \
        --keep-walls 2

OUTPUTS (in the same directory as --output prefix)
───────────────────────────────────────────────────
  <prefix>.pgm   ← raw occupancy grid, Nav2 trinary convention
                   (0=occupied/black, 254=free/white; no "unknown" grey —
                   the whole scanned bounding box is treated as free unless
                   a cell actually contains points)
  <prefix>.yaml  ← companion map_server YAML (image/resolution/origin/
                   negate/occupied_thresh/free_thresh/mode) — origin is
                   computed from the point cloud's own bounding box, NOT
                   assumed to be [0,0,0] (unlike the hand-drawn-diagram
                   maps, a real lidar scan is centered whereever FAST-LIO2
                   happened to start, not the image corner)

ARGUMENTS AT A GLANCE
──────────────────────
  --input               Input .pcd file (ascii or binary, any PCL point
                         type — only x/y/z fields are read)
  --output              Output path prefix                (default: same
                         directory/basename as --input)
  --resolution          Metres per pixel                  (default: 0.05)
  --z-min / --z-max     Height slab (metres, in the .pcd's own frame) kept
                         for the 2-D projection — everything outside this
                         range is ignored. THIS IS THE MOST IMPORTANT
                         PARAMETER: too low and you pick up floor noise,
                         too high and you miss real walls or pick up
                         ceiling/overhang returns. Use --preview-only first.
  --min-points-per-cell Points required in a pixel before it counts as
                         occupied                          (default: 3)
  --preview-only        Print a height (z) histogram and point-cloud
                         bounding box, write NO files. Use this first to
                         choose --z-min/--z-max.
  --max-points          Safety cap on points read from a huge .pcd, for a
                         quick look                        (default: none)

ALGORITHM
─────────
  1. Read x/y/z out of the .pcd (minimal dependency-free binary/ascii
     parser — mirrors fast_lio_ros2/tools/check_map.py's own
     _load_pcd_minimal(), adapted here to avoid a fragile cross-package
     import).
  2. Keep only points with z in [--z-min, --z-max] — this is the entire
     "3-D to 2-D" step: everything else is a top-down projection of
     whatever's left, so choosing this slab well IS the map quality.
  3. Bin the remaining x/y points into a --resolution grid; a cell is
     "occupied" (black) if it has >= --min-points-per-cell points in it,
     else "free" (white). The grid is sized exactly to the point cloud's
     own x/y bounding box (with a small margin) — not a fixed canvas.
  4. Write the PGM + companion YAML (image/resolution/origin/negate/
     thresholds/mode) in the same format as this repo's other map YAMLs.
"""

import argparse
import os
import sys

import cv2
import numpy as np
from scipy.spatial import cKDTree


# ─── Minimal PCD reader (ascii or binary, any fields — only needs x/y/z) ──────
# Deliberately self-contained (no open3d/pcl/pypcd dependency) so this script
# runs anywhere numpy+opencv already do. Mirrors the approach in
# fast_lio_ros2/tools/check_map.py's _load_pcd_minimal(), reimplemented here
# rather than imported since tools/ isn't on the Python path as an installed
# package module.

def load_pcd_xyz(path, max_points=None):
    with open(path, "rb") as f:
        fields, sizes, types, counts = [], [], [], []
        npts = 0
        data_fmt = "ascii"
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if not line:
                continue
            key = line.split()[0].upper() if line.split() else ""
            if key == "FIELDS":
                fields = line.split()[1:]
            elif key == "SIZE":
                sizes = [int(x) for x in line.split()[1:]]
            elif key == "TYPE":
                types = line.split()[1:]
            elif key == "COUNT":
                counts = [int(x) for x in line.split()[1:]]
            elif key == "POINTS":
                npts = int(line.split()[1])
            elif key == "DATA":
                data_fmt = line.split()[1]
                break

        idx = {n: i for i, n in enumerate(fields)}
        for required in ("x", "y", "z"):
            if required not in idx:
                raise ValueError(f"PCD file is missing required field '{required}'")

        if data_fmt == "ascii":
            arr = np.loadtxt(f, dtype=np.float64)
            if arr.ndim == 1:
                arr = arr.reshape(1, -1)
            xyz = arr[:, [idx["x"], idx["y"], idx["z"]]]
        elif data_fmt == "binary":
            counts = counts or [1] * len(fields)
            np_types = {
                ("F", 4): "f4", ("F", 8): "f8",
                ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4",
                ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4",
            }
            dtype = np.dtype([
                (fields[i], np_types[(types[i], sizes[i])])
                for i in range(len(fields))
            ])
            raw = np.frombuffer(f.read(npts * dtype.itemsize), dtype=dtype)
            xyz = np.stack([raw["x"], raw["y"], raw["z"]], axis=1).astype(np.float64)
        else:
            raise ValueError(
                f"Unsupported PCD DATA format '{data_fmt}' "
                "(binary_compressed is not supported by this minimal reader "
                "— re-save as binary or ascii)"
            )

    # Drop any NaN/inf points (FAST-LIO2's ikd-tree can leave these behind
    # at map edges) before they poison the bounding box / histogram below.
    xyz = xyz[np.isfinite(xyz).all(axis=1)]

    if max_points and len(xyz) > max_points:
        sel = np.random.choice(len(xyz), max_points, replace=False)
        xyz = xyz[sel]

    return xyz


# ─── Preview mode ──────────────────────────────────────────────────────────────

def print_preview(xyz):
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    print(f"Points loaded: {len(xyz):,}")
    print(f"X range: [{x.min():.2f}, {x.max():.2f}]  (span {x.max() - x.min():.2f} m)")
    print(f"Y range: [{y.min():.2f}, {y.max():.2f}]  (span {y.max() - y.min():.2f} m)")
    print(f"Z range: [{z.min():.2f}, {z.max():.2f}]  (span {z.max() - z.min():.2f} m)")
    print()
    print("Z histogram (pick --z-min/--z-max to bracket the wall band, "
          "skipping the floor spike near the bottom and any ceiling/"
          "overhang returns near the top):")
    counts, edges = np.histogram(z, bins=20)
    peak = counts.max() or 1
    for c, lo, hi in zip(counts, edges[:-1], edges[1:]):
        bar = "#" * int(40 * c / peak)
        print(f"  [{lo:6.2f}, {hi:6.2f})  {c:8d}  {bar}")


# ─── Outlier removal ──────────────────────────────────────────────────────────

def remove_xy_outliers(slab, resolution, radius_factor, min_neighbors):
    """Statistical outlier removal (same idea as PCL's
    StatisticalOutlierRemoval), applied in 2D on the x/y-projected slab
    BEFORE rasterizing. Drops points whose local neighborhood is too
    sparse to be a real, densely-scanned wall — e.g. a handful of stray
    reflections/noise returns sitting apart from the main structure.
    Confirmed on real data (maps/refined_map.pcd) this cleanly separates
    the two populations: the vast majority of real wall points have
    dozens of neighbors within a few resolution-cells, while true noise
    points typically have single digits.

    Real wall points that are only sparse because of a genuine SCAN GAP
    (not noise) are NOT the target here -- those are handled downstream
    by preprocess_map.py's --close-kernel gap-bridging, which needs the
    real, correctly-shaped wall to still be present, just fragmented.
    This filter targets points that are isolated in every direction, not
    ones that are merely part of a thin/sparse-but-continuous wall.
    """
    if len(slab) == 0 or min_neighbors <= 0:
        return slab
    xy = slab[:, :2]
    tree = cKDTree(xy)
    radius = radius_factor * resolution
    neighbor_counts = tree.query_ball_point(xy, r=radius, return_length=True)
    # query_ball_point counts each point itself too, so >= min_neighbors+1
    # actually means "at least min_neighbors OTHER points nearby".
    keep = neighbor_counts >= (min_neighbors + 1)
    return slab[keep]


# ─── Conversion ─────────────────────────────────────────────────────────────

def convert(xyz, resolution, z_min, z_max, min_points_per_cell, point_radius_px=0,
            outlier_radius_factor=3.0, outlier_min_neighbors=0, exclude_regions=None):
    mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    slab = xyz[mask]
    if len(slab) == 0:
        print(
            f"ERROR: no points survive the z in [{z_min}, {z_max}] filter — "
            "widen the range (see --preview-only for the real z histogram).",
            file=sys.stderr,
        )
        sys.exit(1)

    if outlier_min_neighbors > 0:
        before = len(slab)
        slab = remove_xy_outliers(
            slab, resolution, outlier_radius_factor, outlier_min_neighbors
        )
        print(f"Outlier removal: kept {len(slab)}/{before} points "
              f"({before - len(slab)} dropped as isolated noise)")
        if len(slab) == 0:
            print(
                "ERROR: outlier removal dropped every point — "
                "--outlier-min-neighbors is too strict for this point cloud's "
                "real density. Lower it or set to 0 to disable.",
                file=sys.stderr,
            )
            sys.exit(1)

    if exclude_regions:
        for (x_lo, x_hi, y_lo, y_hi) in exclude_regions:
            before = len(slab)
            in_region = (
                (slab[:, 0] >= x_lo) & (slab[:, 0] <= x_hi)
                & (slab[:, 1] >= y_lo) & (slab[:, 1] <= y_hi)
            )
            slab = slab[~in_region]
            print(f"Excluded region x=[{x_lo},{x_hi}] y=[{y_lo},{y_hi}]: "
                  f"dropped {before - len(slab)} points")
        if len(slab) == 0:
            print("ERROR: exclude-region dropped every remaining point.",
                  file=sys.stderr)
            sys.exit(1)

    x, y = slab[:, 0], slab[:, 1]
    margin = 4 * resolution
    x_min, x_max = x.min() - margin, x.max() + margin
    y_min, y_max = y.min() - margin, y.max() + margin

    width = max(1, int(np.ceil((x_max - x_min) / resolution)))
    height = max(1, int(np.ceil((y_max - y_min) / resolution)))

    col = np.clip(((x - x_min) / resolution).astype(np.int64), 0, width - 1)
    # PGM row 0 is the TOP of the image; ROS map convention has increasing Y
    # go "up" (matching origin's bottom-left convention) — so image row must
    # count DOWN as y increases, i.e. flip vertically here.
    row = np.clip(((y_max - y) / resolution).astype(np.int64), 0, height - 1)

    occupancy = np.zeros((height, width), dtype=np.int32)
    np.add.at(occupancy, (row, col), 1)

    img = np.full((height, width), 254, dtype=np.uint8)  # default: free (white)
    img[occupancy >= min_points_per_cell] = 0             # occupied (black)

    if point_radius_px > 0:
        # Splat each occupied cell into a filled disk of the given pixel
        # radius, THEN re-derive occupancy from the splatted result. This is
        # the standard fix for sparse point-cloud-to-grid rasterization
        # (same idea used by real occupancy-grid converters like octomap or
        # PCL's grid projection) -- real wall points that are individually
        # too far apart to touch as single pixels still overlap once each
        # is inflated to a small disk, closing real gaps WITHOUT the
        # aggressive blob-merging side effects of running
        # preprocess_map.py's --close-kernel at a very large value on an
        # already-too-sparse image (that also blurs/merges unrelated
        # nearby structure, not just gaps in one wall). Do this splatting
        # here, at the true point-cloud resolution, before any downstream
        # blob/contour processing ever sees the image.
        occ_mask = (img == 0).astype(np.uint8)
        kernel_size = 2 * point_radius_px + 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
        )
        dilated = cv2.dilate(occ_mask, kernel)
        img[dilated > 0] = 0

    origin = (float(x_min), float(y_min), 0.0)
    return img, origin


# ─── YAML writer (matches this repo's existing map yaml convention) ──────────

def write_yaml(dst_yaml, pgm_name, resolution, origin):
    with open(dst_yaml, "w") as f:
        f.write(f"image: {pgm_name}\n")
        f.write(f"resolution: {resolution}\n")
        f.write(f"origin: [{origin[0]:.4f}, {origin[1]:.4f}, {origin[2]:.4f}]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
        f.write("mode: trinary\n")


# ─── CLI ─────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Project a FAST-LIO2 .pcd into a raw Nav2 occupancy-grid PGM.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--input", required=True, help="Input .pcd file")
    p.add_argument("--output", default=None,
                    help="Output path prefix (default: same as --input, "
                         "extension stripped)")
    p.add_argument("--resolution", type=float, default=0.05,
                    help="Metres per pixel (default: 0.05)")
    p.add_argument("--z-min", type=float, default=None,
                    help="Lower z bound (metres) of the wall slab to keep")
    p.add_argument("--z-max", type=float, default=None,
                    help="Upper z bound (metres) of the wall slab to keep")
    p.add_argument("--min-points-per-cell", type=int, default=3,
                    help="Points required in a pixel to count as occupied "
                         "(default: 3)")
    p.add_argument("--point-radius-px", type=int, default=0,
                    help="Splat each occupied pixel into a filled disk of "
                         "this pixel radius before writing the PGM (default: "
                         "0, no splatting). Fixes sparse point clouds where "
                         "real wall points are individually too far apart "
                         "to touch as single pixels, without the aggressive "
                         "unrelated-structure-merging side effects of a very "
                         "large preprocess_map.py --close-kernel on an "
                         "already-too-sparse image. Try 1-3 first.")
    p.add_argument("--outlier-min-neighbors", type=int, default=0,
                    help="Statistical outlier removal (same idea as PCL's "
                         "StatisticalOutlierRemoval): drop points with fewer "
                         "than this many OTHER points within "
                         "--outlier-radius-factor * --resolution of them, "
                         "BEFORE rasterizing. Default: 0 (disabled). Use "
                         "this to remove stray noise/reflection points that "
                         "would otherwise get welded onto a real wall by "
                         "--point-radius-px splatting or preprocess_map.py's "
                         "--close-kernel (both of which cannot tell isolated "
                         "noise apart from a real gap once splatting/closing "
                         "has run). Try 5-10 first; check with "
                         "--preview-only if too much/little gets removed.")
    p.add_argument("--outlier-radius-factor", type=float, default=3.0,
                    help="Neighbor search radius for --outlier-min-neighbors, "
                         "as a multiple of --resolution (default: 3.0).")
    p.add_argument("--exclude-region", action="append", default=[],
                    metavar="X_MIN,X_MAX,Y_MIN,Y_MAX",
                    help="Drop every point inside this world-space x/y "
                         "bounding box (metres, in the .pcd's own frame) "
                         "before rasterizing. Repeatable. Use this to "
                         "manually remove a specific known artifact (e.g. a "
                         "stray reflection cluster) that survives z-slicing/"
                         "--point-radius-px/--outlier-min-neighbors because "
                         "it is locally dense, not isolated noise. Use "
                         "--preview-only or inspect a first pass's overlay "
                         "to find the real x/y region to exclude.")
    p.add_argument("--preview-only", action="store_true",
                    help="Print bounding box + z histogram, write no files")
    p.add_argument("--max-points", type=int, default=None,
                    help="Randomly subsample to at most this many points "
                         "before processing (safety cap for huge PCDs)")
    return p.parse_args()


def main():
    args = parse_args()

    if not os.path.isfile(args.input):
        print(f"ERROR: input file not found: {args.input}", file=sys.stderr)
        sys.exit(1)

    print(f"Loading {args.input} ...")
    xyz = load_pcd_xyz(args.input, max_points=args.max_points)

    if args.preview_only:
        print_preview(xyz)
        return

    if args.z_min is None or args.z_max is None:
        print(
            "ERROR: --z-min and --z-max are required unless --preview-only "
            "is set. Run with --preview-only first to see the real z "
            "histogram for this point cloud.",
            file=sys.stderr,
        )
        sys.exit(1)

    exclude_regions = []
    for spec in args.exclude_region:
        try:
            parts = [float(v) for v in spec.split(",")]
            if len(parts) != 4:
                raise ValueError
        except ValueError:
            print(
                f"ERROR: --exclude-region '{spec}' must be exactly "
                "X_MIN,X_MAX,Y_MIN,Y_MAX (4 comma-separated numbers)",
                file=sys.stderr,
            )
            sys.exit(1)
        exclude_regions.append(tuple(parts))

    img, origin = convert(
        xyz, args.resolution, args.z_min, args.z_max, args.min_points_per_cell,
        point_radius_px=args.point_radius_px,
        outlier_radius_factor=args.outlier_radius_factor,
        outlier_min_neighbors=args.outlier_min_neighbors,
        exclude_regions=exclude_regions,
    )

    prefix = args.output or os.path.splitext(args.input)[0]
    out_pgm = prefix + ".pgm"
    out_yaml = prefix + ".yaml"

    cv2.imwrite(out_pgm, img)
    write_yaml(out_yaml, os.path.basename(out_pgm), args.resolution, origin)

    occupied_px = int(np.sum(img == 0))
    print(f"Wrote {out_pgm}  ({img.shape[1]}x{img.shape[0]} px, "
          f"{occupied_px:,} occupied px)")
    print(f"Wrote {out_yaml}")
    print()
    print("Next step — clean up with the existing preprocessing pipeline:")
    print(f"  python3 scripts/preprocess_map.py --input {out_pgm} "
          f"--yaml {out_yaml} --preview-only")


if __name__ == "__main__":
    main()
