#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════
                        register_zones_to_map.py
   Re-project diagram-derived zone waypoints onto a real, LIO-SAM-generated
   map, once a mapping pass on the actual (or full-size mock) course exists.
═════════════════════════════════════════════════════════════════════════════

WHY THIS EXISTS
───────────────
`scripts/generate_course_pgm.py` hand-places 10 zone entry-poses/polygons
(maps/course_map_zones.yaml) onto a map DERIVED FROM THE OFFICIAL COURSE
DIAGRAM (maps/course_map.pgm) — that's a synthetic map, not built from real
sensor data. Once the team gets a practice/walkthrough window on the actual
course (or a full-size mockup) and runs the real mapping pipeline
(FAST-LIO2 + LIO-SAM), a real map exists (e.g. maps/global_map.pgm /
maps/global_map_clean.pgm / maps/global_map_smooth.pgm) — but it has no
zones marked on it.

Rather than re-clicking all 10 zone waypoints from scratch on the new map,
this script re-projects the EXISTING zone yaml onto the new map's coordinate
frame using a similarity transform (rotation + uniform scale + translation)
fit from a handful of manually-identified corresponding landmarks that are
visible in both maps (e.g. a distinctive wall corner, the start-gate posts,
a tunnel mouth). This is NOT automatic image registration — the diagram map
and a real LiDAR map look too different for that to be reliable — the
correspondences must be picked by a human who can recognize the same
physical feature in both maps.

HOW TO USE
──────────
Step 1 — identify 3+ matching landmark points, visible in BOTH maps, and
record their world coordinates (metres) in each map's own frame. Two ways
to do this:

  (a) Interactive picker (recommended — no ROS needed):
        python3 scripts/register_zones_to_map.py --pick \\
            --source-map maps/course_map.yaml \\
            --target-map maps/global_map_clean.yaml \\
            --correspondences maps/zone_registration_points.yaml
      Click a landmark on the LEFT (source/diagram) image, then click the
      SAME physical landmark on the RIGHT (target/real) image. Repeat for
      at least 3 points (more = a more reliable fit), then close the window.

  (b) Manual: open both maps in RViz (map_server + a Publish Point tool, or
      just hover and read the frame coordinates in RViz's status bar) and
      write maps/zone_registration_points.yaml by hand — see the format
      documented in `load_correspondences()` below.

Step 2 — apply the fit and re-project the zone yaml:
    python3 scripts/register_zones_to_map.py \\
        --source-zones maps/course_map_zones.yaml \\
        --correspondences maps/zone_registration_points.yaml \\
        --target-map maps/global_map_clean.yaml \\
        --output maps/global_map_zones.yaml \\
        --overlay maps/global_map_zones.png

The script prints the fitted rotation/scale/translation and the per-point
RMS residual — a residual much larger than your position-picking precision
(a few cm) means either a mis-picked correspondence or that the two maps
don't actually share consistent scale (re-check before trusting the output).

OUTPUTS
───────
  <output>.yaml   Zone definitions (same schema as course_map_zones.yaml),
                  re-expressed in the target map's world frame.
  <overlay>.png   Optional visual sanity check: zone polygons drawn over the
                  target map's PGM image.
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

import matplotlib
matplotlib.use("Qt5Agg" if os.environ.get("DISPLAY") else "Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)


# ─── Map YAML / PGM loading ───────────────────────────────────────────────────

def load_map_yaml(path):
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


def world_to_px(x, y, meta, img_h):
    """ROS map_server convention: origin = bottom-left world coord of image;
    image row 0 is the TOP of the map (so pixel y is flipped vs. world y)."""
    ox, oy = meta["origin"][0], meta["origin"][1]
    res = meta["resolution"]
    px = (x - ox) / res
    py = img_h - 1 - (y - oy) / res
    return px, py


def px_to_world(px, py, meta, img_h):
    ox, oy = meta["origin"][0], meta["origin"][1]
    res = meta["resolution"]
    x = px * res + ox
    y = (img_h - 1 - py) * res + oy
    return x, y


# ─── Correspondences ──────────────────────────────────────────────────────────

def load_correspondences(path):
    """
    correspondences.yaml format:
      points:
        - label: "start gate, left post base"
          source: [1.83, 2.10]     # metres, in the SOURCE map's world frame
          target: [0.42, -1.15]    # metres, in the TARGET map's world frame
        - label: "tunnel mouth, near corner"
          source: [2.38, 4.68]
          target: [1.01, 0.53]
        ...  (3+ pairs recommended)
    """
    with open(path) as f:
        data = yaml.safe_load(f)
    pts = data.get("points", [])
    if len(pts) < 2:
        sys.exit(f"[ERROR] Need at least 2 correspondences, found {len(pts)} "
                  f"in {path}")
    if len(pts) < 3:
        print("[WARN] Only 2 correspondences given — a similarity transform "
              "has 4 DOF (rotation, scale, tx, ty), so 2 points is the "
              "mathematical minimum and gives ZERO residual check. Add a "
              "3rd point to catch mis-picked landmarks.")
    src = np.array([p["source"] for p in pts], dtype=float)
    tgt = np.array([p["target"] for p in pts], dtype=float)
    labels = [p.get("label", f"point{i}") for i, p in enumerate(pts)]
    return src, tgt, labels


def save_correspondences(path, src, tgt, labels):
    points = [
        {"label": labels[i], "source": [round(float(s[0]), 4), round(float(s[1]), 4)],
         "target": [round(float(t[0]), 4), round(float(t[1]), 4)]}
        for i, (s, t) in enumerate(zip(src, tgt))
    ]
    with open(path, "w") as f:
        yaml.safe_dump({"points": points}, f, default_flow_style=None, sort_keys=False)
    print(f"Saved correspondences → {path}")


def interactive_pick(source_meta, source_pgm, target_meta, target_pgm):
    """Side-by-side click-to-pick correspondence tool. Click a landmark on
    the left (source) image, then the matching landmark on the right
    (target) image; repeat. Close the window when done (Ctrl+W / close box)."""
    fig, (ax_l, ax_r) = plt.subplots(1, 2, figsize=(16, 8))
    ax_l.imshow(source_pgm, cmap="gray", origin="upper")
    ax_l.set_title("SOURCE map (diagram-derived)\nClick landmark, then its match on the right")
    ax_r.imshow(target_pgm, cmap="gray", origin="upper")
    ax_r.set_title("TARGET map (real mapping run)")

    src_px, tgt_px, labels = [], [], []
    picking_source = {"flag": True}

    def onclick(event):
        if event.inaxes is ax_l and picking_source["flag"]:
            src_px.append((event.xdata, event.ydata))
            ax_l.plot(event.xdata, event.ydata, "r+", markersize=12, markeredgewidth=2)
            fig.canvas.draw()
            picking_source["flag"] = False
            print(f"  [{len(src_px)}] source px = ({event.xdata:.1f}, {event.ydata:.1f}) "
                  f"— now click the matching point on the TARGET (right) image")
        elif event.inaxes is ax_r and not picking_source["flag"]:
            tgt_px.append((event.xdata, event.ydata))
            ax_r.plot(event.xdata, event.ydata, "b+", markersize=12, markeredgewidth=2)
            idx = len(tgt_px)
            ax_l.annotate(str(idx), src_px[-1], color="red", fontsize=9)
            ax_r.annotate(str(idx), tgt_px[-1], color="blue", fontsize=9)
            fig.canvas.draw()
            labels.append(f"point{idx}")
            picking_source["flag"] = True
            print(f"  [{idx}] target px = ({tgt_px[-1][0]:.1f}, {tgt_px[-1][1]:.1f}) — pair complete")

    fig.canvas.mpl_connect("button_press_event", onclick)
    print("\nClick alternating: SOURCE landmark, then its TARGET match. "
          "Close the window when you have 3+ pairs.\n")
    plt.tight_layout()
    plt.show()

    if len(src_px) != len(tgt_px):
        sys.exit("[ERROR] Unequal number of source/target clicks — an odd "
                  "click was left unpaired. Re-run --pick.")
    if len(src_px) < 2:
        sys.exit("[ERROR] Need at least 2 correspondence pairs.")

    src_h = source_pgm.shape[0]
    tgt_h = target_pgm.shape[0]
    src_world = np.array([px_to_world(px, py, source_meta, src_h) for px, py in src_px])
    tgt_world = np.array([px_to_world(px, py, target_meta, tgt_h) for px, py in tgt_px])
    return src_world, tgt_world, labels


# ─── Similarity transform (Umeyama) ───────────────────────────────────────────

def fit_similarity_transform(src, tgt):
    """
    Least-squares similarity transform (rotation + uniform scale +
    translation) mapping `src` points onto `tgt` points, via the Umeyama
    (1991) closed-form method. Returns (R (2x2), scale, t (2,), rms_residual).
    """
    n = src.shape[0]
    mu_src = src.mean(axis=0)
    mu_tgt = tgt.mean(axis=0)
    src_c = src - mu_src
    tgt_c = tgt - mu_tgt

    var_src = (src_c ** 2).sum() / n
    cov = (tgt_c.T @ src_c) / n

    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(2)
    if np.linalg.det(cov) < 0:
        S[-1, -1] = -1

    R = U @ S @ Vt
    scale = np.trace(np.diag(D) @ S) / var_src
    t = mu_tgt - scale * R @ mu_src

    # Per-point residual for sanity-checking the fit.
    pred = (scale * (R @ src.T).T) + t
    residuals = np.linalg.norm(pred - tgt, axis=1)
    rms = float(np.sqrt((residuals ** 2).mean()))
    return R, float(scale), t, rms, residuals


def apply_transform(xy, R, scale, t):
    xy = np.atleast_2d(xy)
    return (scale * (R @ xy.T).T) + t


# ─── Zone yaml I/O ────────────────────────────────────────────────────────────

def load_zones_yaml(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return data["zones"]


def write_zones_yaml(zones, R, scale, t, target_image_name, out_path, source_path):
    theta = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    lines = [
        "# Zone definitions re-projected onto a real map — generated by\n",
        "# scripts/register_zones_to_map.py. DO NOT hand-edit; re-run the\n",
        f"# script instead. Source: {os.path.relpath(source_path, REPO_ROOT)}\n",
        f"# Target map image: {target_image_name}\n",
        f"# Fitted transform: rotation={theta:.3f} deg, scale={scale:.5f}\n",
        "# entry_pose: [x_m, y_m, yaw_rad]  in the TARGET map frame\n",
        "# polygon:    list of [x_m, y_m] vertices (counter-clockwise)\n\n",
        "zones:\n",
    ]
    for zone_id, z in zones.items():
        ex, ey, eyaw = z["entry_pose"]
        new_xy = apply_transform([ex, ey], R, scale, t)[0]
        new_yaw = eyaw + np.arctan2(R[1, 0], R[0, 0])
        poly = np.array(z["polygon"], dtype=float)
        new_poly = apply_transform(poly, R, scale, t)

        lines.append(f"  {zone_id}:\n")
        lines.append(f"    label:            \"{z['label']}\"\n")
        lines.append(f"    traversal_order:  {z['traversal_order']}\n")
        lines.append(
            f"    entry_pose:       [{round(float(new_xy[0]), 3)}, "
            f"{round(float(new_xy[1]), 3)}, {round(float(new_yaw), 4)}]\n"
        )
        vstr = ", ".join(f"[{round(float(x), 3)}, {round(float(y), 3)}]" for x, y in new_poly)
        lines.append(f"    polygon:          [{vstr}]\n\n")

    with open(out_path, "w") as f:
        f.writelines(lines)
    print(f"Saved re-projected zones YAML → {out_path}")


def draw_overlay(target_pgm, target_meta, zones, R, scale, t, out_path):
    img_h = target_pgm.shape[0]
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.imshow(target_pgm, cmap="gray", origin="upper")
    ax.set_title("Target map — re-projected zone overlay")

    cmap = plt.get_cmap("tab10")
    legend_handles = []
    for i, (zone_id, z) in enumerate(zones.items()):
        color = cmap(i % 10)
        poly_world = np.array(z["polygon"], dtype=float)
        poly_world_t = apply_transform(poly_world, R, scale, t)
        poly_px = [world_to_px(x, y, target_meta, img_h) for x, y in poly_world_t]
        patch = plt.Polygon(poly_px, closed=True, facecolor=(*color[:3], 0.22),
                             edgecolor=(*color[:3], 0.9), linewidth=1.5)
        ax.add_patch(patch)

        ex, ey, _ = z["entry_pose"]
        ex_t, ey_t = apply_transform([ex, ey], R, scale, t)[0]
        epx, epy = world_to_px(ex_t, ey_t, target_meta, img_h)
        ax.plot(epx, epy, "o", color=color[:3], markersize=6,
                markeredgecolor="white", markeredgewidth=1.0)
        legend_handles.append(mpatches.Patch(color=color, label=z["label"]))

    ax.legend(handles=legend_handles, loc="lower right", fontsize=6, framealpha=0.8)
    ax.axis("off")
    plt.tight_layout()
    plt.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close()
    print(f"Saved overlay PNG              → {out_path}")


# ─── Main ─────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Re-project zone waypoints from a diagram-derived map "
                    "onto a real, LIO-SAM-generated map.")
    p.add_argument("--source-map", default=os.path.join(REPO_ROOT, "maps", "course_map.yaml"),
                   help="Map YAML the source zones were authored against")
    p.add_argument("--source-zones", default=os.path.join(REPO_ROOT, "maps", "course_map_zones.yaml"),
                   help="Zone yaml in the source map's frame")
    p.add_argument("--target-map", required=True,
                   help="Map YAML for the real mapping-run map (e.g. maps/global_map_clean.yaml)")
    p.add_argument("--correspondences",
                   default=os.path.join(REPO_ROOT, "maps", "zone_registration_points.yaml"),
                   help="Corresponding landmark points file (read, or written if --pick)")
    p.add_argument("--pick", action="store_true",
                   help="Launch the interactive click-to-pick correspondence tool "
                        "and save results to --correspondences before proceeding")
    p.add_argument("--output", default=None,
                   help="Output zones yaml path (default: <target-map-dir>/<target-map-name>_zones.yaml)")
    p.add_argument("--overlay", default=None,
                   help="Optional output PNG path for a visual zone-overlay sanity check")
    return p.parse_args()


def main():
    args = parse_args()

    source_meta, source_pgm_path = load_map_yaml(args.source_map)
    target_meta, target_pgm_path = load_map_yaml(args.target_map)

    if args.pick:
        source_pgm = load_pgm(source_pgm_path)
        target_pgm = load_pgm(target_pgm_path)
        src_world, tgt_world, labels = interactive_pick(
            source_meta, source_pgm, target_meta, target_pgm)
        save_correspondences(args.correspondences, src_world, tgt_world, labels)
    else:
        src_world, tgt_world, _ = load_correspondences(args.correspondences)

    R, scale, t, rms, residuals = fit_similarity_transform(src_world, tgt_world)
    theta = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    print(f"\nFitted similarity transform:")
    print(f"  rotation    = {theta:+.3f} deg")
    print(f"  scale       = {scale:.5f}  (expect ~1.0 for two accurately-metric maps)")
    print(f"  translation = ({t[0]:+.3f}, {t[1]:+.3f}) m")
    print(f"  RMS residual = {rms*100:.1f} cm across {len(residuals)} point(s)")
    for i, r in enumerate(residuals):
        flag = "  <-- large, check this point" if r > 3 * rms and r > 0.10 else ""
        print(f"    point {i+1}: {r*100:.1f} cm{flag}")
    if rms > 0.20:
        print("[WARN] RMS residual > 20 cm — re-check your correspondences "
              "before trusting the output zones.")

    zones = load_zones_yaml(args.source_zones)

    target_dir = os.path.dirname(os.path.abspath(args.target_map))
    target_base = os.path.splitext(os.path.basename(args.target_map))[0]
    out_path = args.output or os.path.join(target_dir, f"{target_base}_zones.yaml")
    write_zones_yaml(zones, R, scale, t, target_meta["image"], out_path, args.source_zones)

    if args.overlay:
        target_pgm = load_pgm(target_pgm_path)
        draw_overlay(target_pgm, target_meta, zones, R, scale, t, args.overlay)


if __name__ == "__main__":
    main()
