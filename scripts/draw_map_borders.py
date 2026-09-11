#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════
                          draw_map_borders.py
     Hand-draw the outer wall + inner obstacle(s) on a raw PGM, producing
     a clean, closed Nav2 occupancy grid
═════════════════════════════════════════════════════════════════════════════

WHY THIS EXISTS
───────────────
preprocess_map.py's automatic wall detection (blob selection + gap-bridging
via --close-kernel/--inner-close-kernel) works well when the underlying scan
is dense and continuous, but it CANNOT invent wall data that was never
scanned — confirmed on a real course map (LIO_Localization's
refined_map_1.pcd) where the inner wall has a genuine gap (point density
drops from ~600 to 0 in one stretch), so no --close-kernel/
--inner-close-kernel value can fully close it; the tool can only bridge
gaps BETWEEN existing points, not gaps where zero points exist at all.

This script is the manual alternative for exactly that situation: you look
at the raw scan (shown as a background reference), and trace the real wall
boundary by eye through any gaps, the same way a person naturally
interpolates a partially-occluded shape. Use this INSTEAD of
preprocess_map.py when automatic wall detection keeps leaving a broken
ring/spike/gap no parameter combination fixes.

HOW TO USE
──────────
  Step 1: Get a raw PGM from a .pcd (this script does NOT read .pcd files
          directly — run pcd_to_pgm.py first):
    python3 scripts/pcd_to_pgm.py --input maps/refined_map.pcd \\
        --output maps/raw_course --z-min -0.3 --z-max 1.4

  Step 2: Trace the borders:
    python3 scripts/draw_map_borders.py \\
        --map maps/raw_course.yaml \\
        --output maps/course_hand_drawn

Controls:
  Left-click       Add a point to the CURRENTLY ACTIVE polygon
  'o'              Start drawing the OUTER wall (resets it if already started)
  'i'              Finish the current polygon and start a NEW inner
                   obstacle/wall (call this once per separate inner
                   obstacle — most courses only need it once, for the
                   single inner ring)
  'u'              Undo the last point in the currently active polygon
  Close the window Finish the current polygon and rasterize everything

The raw PGM's actual scanned points are shown dimmed in the background so
you can trace directly over real data, including through any gap — you are
the gap-bridging algorithm here, which is the whole point.

OUTPUT
──────
  <output>.pgm/.yaml   Trinary occupancy grid, same resolution/origin as the
                       INPUT map (frame is preserved exactly — this does
                       NOT re-derive origin, unlike pcd_to_pgm.py, since
                       you're just re-drawing walls on an already-correct
                       frame) — everything inside the outer polygon and
                       outside every inner polygon is FREE; everything
                       outside the outer polygon or inside any inner
                       polygon is UNKNOWN/unavailable; the traced
                       boundaries themselves are drawn as a closed,
                       smoothed WALL of configurable thickness.
  <output>_preview.png (if --overlay given) Visual sanity check overlaying
                       the drawn polygons on the raw input, before you
                       trust the output.

ARGUMENTS AT A GLANCE
──────────────────────
  --map              Input map YAML (the raw/messy PGM to trace over)
  --output           Output path prefix
  --points-file       Where to save/load the traced points (default:
                      <output>_borders.yaml). If this already exists it is
                      loaded automatically and the interactive tracer is
                      SKIPPED — trace once per input map, re-run freely
                      afterwards (e.g. to tweak --wall-thickness-in) with
                      NO re-tracing.
  --retrace           Force the interactive tracer even if --points-file
                      already exists (overwrites it).
  --wall-thickness    Wall draw thickness as a raw cv2.polylines pixel
                      count (default: 8) — NOT real-world-accurate, see
                      --wall-thickness-in.
  --wall-thickness-in Wall thickness in real-world INCHES — empirically
                      calibrated at runtime so the ACTUAL drawn width
                      matches (cv2.polylines does not draw exactly the
                      requested pixel count).
  --wall-thickness-m  Same as --wall-thickness-in, in metres.
  --smooth-sigma      Gaussian smoothing of the traced path  (default: 4.0,
                      0 to disable — draws exactly what you clicked, jagged
                      straight segments between clicks)
  --resample-points   Points to resample each polygon to before smoothing
                      (default: 200 — higher = smoother curve fidelity)
  --overlay           Optional preview PNG path
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

# Pick a working interactive matplotlib backend explicitly rather than
# letting matplotlib auto-detect one — a real failure mode already hit in
# this project: a stale/invalid QT_API env var makes matplotlib's Qt-backend
# auto-selection raise RuntimeError outright instead of falling back to
# another backend, even though other backends (TkAgg etc.) work fine on the
# same machine. See scripts/pick_waypoints.py for the same fix.
import matplotlib

_BACKEND_ERROR = None
for _backend in ("TkAgg", "Qt5Agg", "QtAgg", "GTK3Agg", "MacOSX"):
    try:
        matplotlib.use(_backend, force=True)
        _BACKEND_ERROR = None
        break
    except Exception as _exc:  # noqa: BLE001 - genuinely need to try the next one
        _BACKEND_ERROR = _exc
        continue

import matplotlib.pyplot as plt  # noqa: E402 - backend must be set first

FREE_VALUE = 254
UNKNOWN_VALUE = 205
WALL_VALUE = 0


# ─── Map loading (same convention as pick_waypoints.py / register_zones_to_map.py) ─

def load_map_yaml(path):
    if not os.path.isfile(path):
        sys.exit(f"[ERROR] Map YAML not found: {path}")
    with open(path) as f:
        meta = yaml.safe_load(f)
    map_dir = os.path.dirname(os.path.abspath(path))
    pgm_path = os.path.join(map_dir, meta["image"])
    return meta, pgm_path


def load_pgm(pgm_path):
    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        sys.exit(f"[ERROR] Could not read PGM: {pgm_path}")
    return img


# ─── Save / load traced points (so tracing only needs to happen once) ───────

def save_points_yaml(path, outer_points, inner_polygons, pgm_shape):
    data = {
        "source_image_shape": [int(pgm_shape[0]), int(pgm_shape[1])],
        "outer": [[float(x), float(y)] for x, y in outer_points],
        "inner": [
            [[float(x), float(y)] for x, y in poly] for poly in inner_polygons
        ],
    }
    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)
    print(f"Saved traced points → {path}  (re-run without --retrace to reuse "
          f"these instead of tracing again)")


def load_points_yaml(path, pgm_shape):
    with open(path) as f:
        data = yaml.safe_load(f)

    saved_shape = tuple(data.get("source_image_shape", []))
    if saved_shape and tuple(pgm_shape) != saved_shape:
        print(
            f"WARNING: {path}'s saved points were traced against an image "
            f"of shape {saved_shape}, but the current input image is "
            f"{tuple(pgm_shape)} — pixel coordinates will likely be wrong. "
            "Re-run with --retrace to trace fresh points for this image."
        )

    outer = [(p[0], p[1]) for p in data.get("outer", [])]
    inner = [[(p[0], p[1]) for p in poly] for poly in data.get("inner", [])]
    return outer, inner


# ─── Interactive polygon tracing ──────────────────────────────────────────────

def interactive_draw(pgm):
    """Returns (outer_points, inner_polygons) as lists of (px, py) pixel
    coordinates. outer_points is a single list; inner_polygons is a list of
    lists (0 or more separate inner obstacles)."""
    fig, ax = plt.subplots(figsize=(10, 10))
    # Dim the raw scan so hand-drawn lines are visually distinct on top of it.
    ax.imshow(pgm, cmap="gray", origin="upper", vmin=0, vmax=254, alpha=0.6)
    ax.set_title(
        "Trace OUTER wall first (click points), then press 'i' to start "
        "each inner obstacle.\n"
        "'o' = restart outer   'i' = finish current, start new inner   "
        "'u' = undo   close window = finish"
    )

    state = {"mode": "outer", "outer": [], "inner": [[]], "markers": []}

    def current_points():
        if state["mode"] == "outer":
            return state["outer"]
        return state["inner"][-1]

    def redraw():
        for m in state["markers"]:
            m.remove()
        state["markers"].clear()

        def draw_polygon(points, color):
            if not points:
                return
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            (line,) = ax.plot(xs, ys, "-o", color=color, markersize=4, linewidth=1.5)
            state["markers"].append(line)
            if len(points) > 2:
                # Preview the closing segment as a dashed line.
                (closer,) = ax.plot(
                    [xs[-1], xs[0]], [ys[-1], ys[0]], "--", color=color, linewidth=1,
                    alpha=0.5,
                )
                state["markers"].append(closer)

        draw_polygon(state["outer"], "red")
        for poly in state["inner"]:
            draw_polygon(poly, "blue")
        fig.canvas.draw()

    def onclick(event):
        if event.inaxes is not ax or event.xdata is None:
            return
        current_points().append((event.xdata, event.ydata))
        mode_label = "OUTER" if state["mode"] == "outer" else f"INNER #{len(state['inner'])}"
        print(f"  [{mode_label}] point {len(current_points())}: "
              f"({event.xdata:.1f}, {event.ydata:.1f})")
        redraw()

    def onkey(event):
        if event.key == "o":
            state["mode"] = "outer"
            state["outer"] = []
            print("Restarting OUTER wall trace.")
            redraw()
        elif event.key == "i":
            finished = current_points()
            if len(finished) < 3:
                print(f"WARNING: current polygon only has {len(finished)} "
                      "point(s) — need at least 3 to form a real shape. "
                      "Ignoring 'i' until you add more points.")
                return
            state["mode"] = "inner"
            state["inner"].append([])
            print(f"Starting INNER obstacle #{len(state['inner'])}.")
            redraw()
        elif event.key == "u":
            pts = current_points()
            if pts:
                removed = pts.pop()
                print(f"  undone: {removed}")
                redraw()

    fig.canvas.mpl_connect("button_press_event", onclick)
    fig.canvas.mpl_connect("key_press_event", onkey)
    print("\nTrace the OUTER wall first (click points in order around the "
          "boundary). Press 'i' when done to start each inner obstacle. "
          "Close the window when finished.\n")
    plt.tight_layout()
    plt.show()

    outer = state["outer"]
    inner_polygons = [p for p in state["inner"] if len(p) >= 3]

    if len(outer) < 3:
        sys.exit("[ERROR] Outer wall needs at least 3 points — got "
                  f"{len(outer)}. Re-run and trace a real closed shape.")

    return outer, inner_polygons


# ─── Real-world wall thickness resolution ────────────────────────────────────

def resolve_wall_thickness_px(target_thickness_m, resolution):
    """cv2.polylines(thickness=N) does NOT draw exactly N pixels wide --
    confirmed empirically (requesting 3 draws 5px, requesting 8 draws 9px;
    the exact relationship isn't a clean formula and may depend on the
    OpenCV build). Rather than hardcode a possibly-version-specific
    formula, calibrate at runtime: draw real test lines at increasing
    requested thicknesses on a throwaway canvas, measure the ACTUAL pixel
    width cv2 produces, and return the requested value whose actual
    drawn width is closest to the real-world target (converted to pixels
    via the map's own resolution)."""
    target_px = target_thickness_m / resolution
    test_canvas_size = 60
    test_pts = np.array(
        [[5, test_canvas_size // 2], [test_canvas_size - 5, test_canvas_size // 2]],
        dtype=np.int32,
    ).reshape(-1, 1, 2)

    best_req, best_diff, best_actual = 1, float("inf"), None
    for req in range(1, 51):
        canvas = np.full((test_canvas_size, test_canvas_size), 254, dtype=np.uint8)
        cv2.polylines(canvas, [test_pts], isClosed=False, color=0, thickness=req)
        col = canvas[:, test_canvas_size // 2]
        actual_px = int((col == 0).sum())
        diff = abs(actual_px - target_px)
        if diff < best_diff:
            best_req, best_diff, best_actual = req, diff, actual_px
        if actual_px >= target_px and diff <= best_diff:
            # Once we've reached/passed the target with the best-so-far
            # match, no larger req will get closer (actual grows
            # monotonically with req), so stop early.
            break

    print(f"Wall thickness: requested {target_thickness_m*39.3701:.2f} in "
          f"({target_px:.2f} px at {resolution} m/px) -> using cv2 thickness="
          f"{best_req} (empirically measured actual: {best_actual} px = "
          f"{best_actual*resolution*39.3701:.2f} in)")
    return best_req


# ─── Smoothing + rasterization ────────────────────────────────────────────────

def resample_and_smooth(points, n_resample, smooth_sigma):
    """Resample a closed polygon to n_resample evenly-spaced-by-index points
    (simple linear interpolation along the click sequence — NOT true
    arc-length resampling, but sufficient here since points are already
    roughly evenly spaced by hand) then Gaussian-smooth in periodic (wrap)
    mode, same technique preprocess_map.py uses on its own auto-extracted
    contours."""
    pts = np.array(points, dtype=float)
    n = len(pts)
    if n_resample and n_resample > n:
        # Parametrize by cumulative index, then interpolate to n_resample
        # evenly spaced parameter values around the closed loop.
        t_orig = np.linspace(0, 1, n, endpoint=False)
        t_new = np.linspace(0, 1, n_resample, endpoint=False)
        xs = np.interp(t_new, np.concatenate([t_orig, [1.0]]),
                       np.concatenate([pts[:, 0], [pts[0, 0]]]))
        ys = np.interp(t_new, np.concatenate([t_orig, [1.0]]),
                       np.concatenate([pts[:, 1], [pts[0, 1]]]))
        pts = np.stack([xs, ys], axis=1)

    if smooth_sigma and smooth_sigma > 0:
        from scipy.ndimage import gaussian_filter1d
        xs = gaussian_filter1d(pts[:, 0], sigma=smooth_sigma, mode="wrap")
        ys = gaussian_filter1d(pts[:, 1], sigma=smooth_sigma, mode="wrap")
        pts = np.stack([xs, ys], axis=1)

    return pts


def rasterize(pgm_shape, outer_points, inner_polygons, wall_thickness,
              smooth_sigma, resample_points):
    h, w = pgm_shape

    outer_smooth = resample_and_smooth(outer_points, resample_points, smooth_sigma)
    inner_smooths = [
        resample_and_smooth(p, resample_points, smooth_sigma) for p in inner_polygons
    ]

    outer_poly_i = outer_smooth.astype(np.int32).reshape(-1, 1, 2)
    inner_polys_i = [p.astype(np.int32).reshape(-1, 1, 2) for p in inner_smooths]

    inside_outer = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(inside_outer, [outer_poly_i], 1)

    inside_any_inner = np.zeros((h, w), dtype=np.uint8)
    for poly_i in inner_polys_i:
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [poly_i], 1)
        inside_any_inner |= mask

    valid_free = (inside_outer == 1) & (inside_any_inner == 0)

    final = np.full((h, w), UNKNOWN_VALUE, dtype=np.uint8)
    final[valid_free] = FREE_VALUE

    cv2.polylines(final, [outer_poly_i], isClosed=True, color=WALL_VALUE,
                  thickness=wall_thickness)
    for poly_i in inner_polys_i:
        cv2.polylines(final, [poly_i], isClosed=True, color=WALL_VALUE,
                      thickness=wall_thickness)

    return final


# ─── Overlay preview ──────────────────────────────────────────────────────────

def draw_overlay(pgm, final, outer_points, inner_polygons, out_path):
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    axes[0].imshow(pgm, cmap="gray", origin="upper", vmin=0, vmax=254)
    axes[0].set_title("Raw input (traced over)")
    if outer_points:
        xs = [p[0] for p in outer_points] + [outer_points[0][0]]
        ys = [p[1] for p in outer_points] + [outer_points[0][1]]
        axes[0].plot(xs, ys, "r-", linewidth=1.5)
    for poly in inner_polygons:
        xs = [p[0] for p in poly] + [poly[0][0]]
        ys = [p[1] for p in poly] + [poly[0][1]]
        axes[0].plot(xs, ys, "b-", linewidth=1.5)
    axes[0].axis("off")

    axes[1].imshow(final, cmap="gray", origin="upper", vmin=0, vmax=254)
    axes[1].set_title("Result: black=wall  white=free  grey=unavailable")
    axes[1].axis("off")

    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved preview overlay → {out_path}")


# ─── Output writer ────────────────────────────────────────────────────────────

def write_output(prefix, final, src_meta):
    out_pgm = prefix + ".pgm"
    out_yaml = prefix + ".yaml"
    cv2.imwrite(out_pgm, final)
    with open(out_yaml, "w") as f:
        f.write(f"image: {os.path.basename(out_pgm)}\n")
        f.write(f"resolution: {src_meta['resolution']}\n")
        origin = src_meta["origin"]
        f.write(f"origin: [{origin[0]:.4f}, {origin[1]:.4f}, {origin[2]:.4f}]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")
        f.write("mode: trinary\n")
    print(f"Wrote {out_pgm}")
    print(f"Wrote {out_yaml}  (resolution/origin copied unchanged from "
          f"{src_meta.get('image', 'input')} — frame is preserved exactly)")


# ─── CLI ─────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Hand-draw the outer wall + inner obstacle(s) on a raw "
                    "PGM to produce a clean Nav2 occupancy grid.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--map", required=True,
                    help="Input map YAML (the raw/messy PGM to trace over)")
    p.add_argument("--output", required=True, help="Output path prefix")
    p.add_argument("--wall-thickness", type=int, default=None,
                    help="Wall draw thickness as a raw cv2.polylines pixel "
                         "count (default: 8). NOTE: cv2.polylines does NOT "
                         "draw exactly this many pixels wide (confirmed "
                         "empirically: requesting 3 can draw 5px) -- use "
                         "--wall-thickness-in/--wall-thickness-m instead "
                         "for a real-world-accurate result; this raw option "
                         "is for matching a specific previously-used value.")
    p.add_argument("--wall-thickness-in", type=float, default=None,
                    help="Wall thickness in real-world INCHES. Empirically "
                         "calibrates the actual cv2.polylines parameter "
                         "needed to hit this size on THIS map's resolution "
                         "(measured at runtime, not a hardcoded formula, "
                         "since the requested-vs-actual relationship isn't "
                         "a clean 1:1 mapping). Mutually exclusive with "
                         "--wall-thickness/--wall-thickness-m.")
    p.add_argument("--wall-thickness-m", type=float, default=None,
                    help="Wall thickness in real-world METRES (alternative "
                         "to --wall-thickness-in). Mutually exclusive with "
                         "--wall-thickness/--wall-thickness-in.")
    p.add_argument("--smooth-sigma", type=float, default=4.0,
                    help="Gaussian smoothing of the traced path (default: "
                         "4.0, 0 to disable)")
    p.add_argument("--resample-points", type=int, default=200,
                    help="Points to resample each polygon to before "
                         "smoothing (default: 200)")
    p.add_argument("--overlay", default=None,
                    help="Optional output PNG path for a visual sanity check")
    p.add_argument("--points-file", default=None,
                    help="Path to save/load the traced border points as YAML "
                         "(default: <output>_borders.yaml). If this file "
                         "already exists, it is loaded automatically and "
                         "the interactive tracer is SKIPPED entirely — you "
                         "only need to trace once per input map. Use "
                         "--retrace to force opening the tracer again.")
    p.add_argument("--retrace", action="store_true",
                    help="Force the interactive tracer to open even if a "
                         "saved --points-file already exists (overwrites it "
                         "with the new trace).")
    args = p.parse_args()

    specified = [
        v is not None for v in
        (args.wall_thickness, args.wall_thickness_in, args.wall_thickness_m)
    ]
    if sum(specified) > 1:
        p.error("Specify only ONE of --wall-thickness / --wall-thickness-in "
                "/ --wall-thickness-m.")
    return args


def main():
    args = parse_args()

    meta, pgm_path = load_map_yaml(args.map)
    pgm = load_pgm(pgm_path)

    if args.wall_thickness_in is not None:
        wall_thickness_px = resolve_wall_thickness_px(
            args.wall_thickness_in * 0.0254, meta["resolution"]
        )
    elif args.wall_thickness_m is not None:
        wall_thickness_px = resolve_wall_thickness_px(
            args.wall_thickness_m, meta["resolution"]
        )
    else:
        wall_thickness_px = (
            args.wall_thickness if args.wall_thickness is not None else 8
        )

    points_file = args.points_file or (args.output + "_borders.yaml")

    if os.path.isfile(points_file) and not args.retrace:
        print(f"Loading previously-traced points from {points_file} "
              "(pass --retrace to trace fresh points instead)...")
        outer_points, inner_polygons = load_points_yaml(points_file, pgm.shape)
    else:
        outer_points, inner_polygons = interactive_draw(pgm)
        save_points_yaml(points_file, outer_points, inner_polygons, pgm.shape)

    final = rasterize(
        pgm.shape, outer_points, inner_polygons, wall_thickness_px,
        args.smooth_sigma, args.resample_points,
    )

    write_output(args.output, final, meta)

    if args.overlay:
        draw_overlay(pgm, final, outer_points, inner_polygons, args.overlay)


if __name__ == "__main__":
    main()
