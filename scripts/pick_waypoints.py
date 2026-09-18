#!/usr/bin/env python3
"""
═════════════════════════════════════════════════════════════════════════════
                             pick_waypoints.py
     Click-to-pick tool: generate a diy_waypoint_sequencer waypoints.yaml
     by clicking points on a real map image
═════════════════════════════════════════════════════════════════════════════

WHY THIS EXISTS
───────────────
diy_waypoint_sequencer/config/waypoints.yaml (the example shipped in that
package) has hand-typed placeholder x/y/yaw values — fine as a schema
reference, useless for testing against a real map. Hand-computing real
map-frame coordinates from pixel positions (accounting for resolution,
origin, and the PGM-row/world-Y flip — see px_to_world() below) is tedious
and error-prone to do by hand.

This is the same click-to-pick pattern already used by
scripts/register_zones_to_map.py's --pick mode, purpose-built here for
diy_waypoint_sequencer's much simpler schema (just label/x/y/yaw — see
that package's config/waypoints.yaml for why it deliberately does NOT use
zone_nav's richer schema).

HOW TO USE
──────────
  python3 scripts/pick_waypoints.py \\
      --map maps/course_traced_smooth.yaml \\
      --output maps/test_waypoints.yaml \\
      --overlay maps/test_waypoints_preview.png

By default, once you close the picker window, the map frame's own origin
(0.0, 0.0) is auto-appended as the FINAL waypoint — PROVIDED that point
actually lands in free space on this map (checked automatically; skipped
if it doesn't, or if the origin is outside the image entirely). This
closes the loop back to wherever the robot physically starts (which
matches map_localizer's own default initial pose exactly, so as long as
the robot is physically placed at the map's origin before pressing green
light, no initial_x/y/yaw override is needed at all). It's added LAST,
not first, because commanding the robot to "navigate to" its own current
starting position as wp1 would be a trivial no-op — a meaningful finish
line, not a meaningful first goal. Pass --no-seed-origin to disable this
and get exactly the points you click.

Click points on the map image IN THE ORDER the robot should visit them.
Each click is validated against the map's own occupancy data immediately
(a click on an occupied/unknown pixel prints a warning right away — fix it
by clicking again, the last click always wins for the current waypoint
index). Press 'u' to undo the last point. Close the window when done.

Heading (yaw) is computed automatically for each waypoint as the bearing
toward the NEXT waypoint (a sane default for a path-following controller
test — nobody wants to hand-compute headings either). The LAST waypoint
is special: with --loop, it bears toward the FIRST waypoint (since that's
where the robot drives to next after wrapping around) — no need to click
the start point again as an extra "closing" waypoint, the sequencer's own
loop:true logic already wraps back to waypoint 0 on its own. Without
--loop, the last waypoint just keeps the previous segment's heading (or
0.0 if only one point was picked). Override entirely with --uniform-yaw
to set every waypoint's yaw to a fixed value instead.

OUTPUT
──────
  <output>            waypoints.yaml in diy_waypoint_sequencer's exact
                       schema (frame_id/loop/waypoints:[label,x,y,yaw])
  <overlay>  (if given) PNG with numbered markers + heading arrows overlaid
             on the map, for a visual sanity check before running on the
             robot — same idea as register_zones_to_map.py's --overlay

Also prints a ready-to-copy `initial_x`/`initial_y`/`initial_yaw` override
for `localization`'s localization.launch.py, matching wp1 exactly.
map_localizer's own relocalization initial pose defaults to (0,0,0) —
i.e. it assumes the robot starts at the MAP FRAME's origin, NOT at wp1 —
so if you actually place the robot at wp1's real-world position before
starting, pass this override or relocalization has to converge from
however far (0,0) is from wp1 (potentially several metres, risking a
failed/wrong VGICP convergence).

Then run it with:
  ros2 launch diy_waypoint_sequencer waypoint_sequencer.launch.py \\
      waypoints_file:=maps/test_waypoints.yaml
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

# Pick a working interactive matplotlib backend explicitly, rather than
# letting matplotlib auto-detect one. A real failure mode (hit on a real
# machine, not hypothetical): a stale/invalid QT_API env var (e.g.
# QT_API=pyqt, which isn't a real value -- valid ones are pyqt6/pyside6/
# pyqt5/pyside2) makes matplotlib's Qt-backend auto-selection raise
# RuntimeError outright instead of falling back to another backend, even
# though other backends (TkAgg etc.) may work fine on the same machine.
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

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

FREE, OCCUPIED, UNKNOWN = 254, 0, 205


# ─── Map loading (same convention as register_zones_to_map.py) ──────────────

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


def px_to_world(px, py, meta, img_h):
    """ROS map_server convention: origin = bottom-left world coord of the
    image; image row 0 is the TOP of the map (so pixel y is flipped vs.
    world y). Identical convention to register_zones_to_map.py's own
    px_to_world() — kept in sync deliberately, don't let these drift apart."""
    ox, oy = meta["origin"][0], meta["origin"][1]
    res = meta["resolution"]
    x = px * res + ox
    y = (img_h - 1 - py) * res + oy
    return x, y


def world_to_px(x, y, meta, img_h):
    """Inverse of px_to_world() — same origin/resolution/row-flip
    convention, used to seed the map-frame origin (0,0) as a pixel to
    occupancy-check before auto-adding it as wp1."""
    ox, oy = meta["origin"][0], meta["origin"][1]
    res = meta["resolution"]
    px = (x - ox) / res
    py = img_h - 1 - (y - oy) / res
    return px, py


def occupancy_at_px(pgm, px, py):
    """Nearest-pixel lookup of the raw PGM value at a clicked point."""
    row = int(round(py))
    col = int(round(px))
    row = min(max(row, 0), pgm.shape[0] - 1)
    col = min(max(col, 0), pgm.shape[1] - 1)
    return int(pgm[row, col])


def describe_occupancy(value):
    if value >= FREE - 1:
        return "free", False
    if value <= OCCUPIED + 1:
        return "OCCUPIED", True
    return "UNKNOWN", True


# ─── Heading computation ─────────────────────────────────────────────────────

def compute_headings(points_world, loop=False):
    """yaw[i] = bearing from point i to point i+1. The LAST point is special:
    if loop=True, its heading bears toward point[0] (the next place the
    robot will actually drive after wrapping around) instead of just
    repeating the previous segment's direction -- otherwise the robot would
    visibly face the wrong way right before turning back to the start."""
    n = len(points_world)
    yaws = [0.0] * n
    for i in range(n - 1):
        dx = points_world[i + 1][0] - points_world[i][0]
        dy = points_world[i + 1][1] - points_world[i][1]
        yaws[i] = float(np.arctan2(dy, dx))
    if n >= 2:
        if loop:
            dx = points_world[0][0] - points_world[-1][0]
            dy = points_world[0][1] - points_world[-1][1]
            yaws[n - 1] = float(np.arctan2(dy, dx))
        else:
            yaws[n - 1] = yaws[n - 2]
    return yaws


# ─── Interactive picker ──────────────────────────────────────────────────────

def interactive_pick(meta, pgm, seed_origin=True):
    """Left-click to add a waypoint (in visit order); 'u' to undo the last
    one. Close the window when done. Prints an immediate free-space
    warning for any click landing on an occupied/unknown pixel.

    seed_origin: if True (default), auto-appends the map frame's own
    origin (0.0, 0.0) as the FINAL waypoint (after everything you click)
    — but ONLY if that point actually falls inside free space on this
    map; otherwise it's skipped and you get exactly the points you
    clicked. This closes the loop back to wherever the robot physically
    starts (map_localizer's own default initial_x/y/yaw seed is also
    (0,0,0) — see print_initial_pose_hint), which is the natural finish
    line for a course, not a meaningful FIRST goal (commanding the robot
    to "navigate to" its own current position is a trivial no-op). Returns
    whether the origin was actually appended.
    """
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(pgm, cmap="gray", origin="upper", vmin=0, vmax=254)
    ax.set_title(
        "Click waypoints IN VISIT ORDER  |  'u' = undo last  |  "
        "close window when done"
    )

    px_list = []
    markers = []

    def redraw():
        for m in markers:
            m.remove()
        markers.clear()
        for i, (px, py) in enumerate(px_list):
            m, = ax.plot(px, py, "r+", markersize=14, markeredgewidth=2)
            markers.append(m)
            t = ax.annotate(str(i + 1), (px, py), color="red", fontsize=10,
                             xytext=(4, 4), textcoords="offset points")
            markers.append(t)
        fig.canvas.draw()

    def onclick(event):
        if event.inaxes is not ax or event.xdata is None:
            return
        px_list.append((event.xdata, event.ydata))
        value = occupancy_at_px(pgm, event.xdata, event.ydata)
        label, is_bad = describe_occupancy(value)
        print(f"  [{len(px_list)}] pixel = ({event.xdata:.1f}, {event.ydata:.1f})"
              f"  map value = {value}  ({label})")
        if is_bad:
            print(f"      WARNING: this point is {label} on the map — the "
                  "planner/controller will likely reject or fail near it. "
                  "Press 'u' to undo and click again.")
        redraw()

    def onkey(event):
        if event.key == "u" and px_list:
            removed = px_list.pop()
            print(f"  undone: pixel {removed}")
            redraw()

    fig.canvas.mpl_connect("button_press_event", onclick)
    fig.canvas.mpl_connect("key_press_event", onkey)
    print("\nClick waypoints in the order the robot should visit them. "
          "Press 'u' to undo. Close the window when finished.\n")
    plt.tight_layout()
    plt.show()

    if len(px_list) == 0:
        sys.exit("[ERROR] No waypoints picked.")

    img_h = pgm.shape[0]
    world = [px_to_world(px, py, meta, img_h) for px, py in px_list]

    appended_origin = False
    if seed_origin:
        ox_px, oy_px = world_to_px(0.0, 0.0, meta, img_h)
        if 0 <= ox_px <= pgm.shape[1] - 1 and 0 <= oy_px <= pgm.shape[0] - 1:
            value = occupancy_at_px(pgm, ox_px, oy_px)
            label, is_bad = describe_occupancy(value)
            if not is_bad:
                world.append((0.0, 0.0))
                appended_origin = True
                print(f"\n  [{len(world)}] map origin (0,0) auto-appended as "
                      "the FINAL waypoint (closes the loop back to start) "
                      f"-- map value = {value} ({label}).")
            else:
                print(f"\n  NOTE: map origin (0,0) is {label} on this map -- "
                      "NOT auto-appending it as the final waypoint.")
        else:
            print("\n  NOTE: map origin (0,0) falls outside this map's image "
                  "bounds -- NOT auto-appending it as the final waypoint.")

    return world, appended_origin


# ─── Output ──────────────────────────────────────────────────────────────────

def write_waypoints_yaml(path, points_world, yaws, loop, label_prefix):
    waypoints = [
        {
            "label": f"{label_prefix}{i + 1}",
            "x": round(float(x), 4),
            "y": round(float(y), 4),
            "yaw": round(float(yaw), 4),
        }
        for i, ((x, y), yaw) in enumerate(zip(points_world, yaws))
    ]
    data = {"frame_id": "map", "loop": bool(loop), "waypoints": waypoints}
    with open(path, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None, sort_keys=False)
    print(f"\nSaved {len(waypoints)} waypoint(s) → {path}")


def draw_overlay(pgm, meta, points_world, yaws, out_path):
    img_h = pgm.shape[0]
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(pgm, cmap="gray", origin="upper", vmin=0, vmax=254)
    res = meta["resolution"]
    ox, oy = meta["origin"][0], meta["origin"][1]

    pxs, pys = [], []
    for (x, y), yaw in zip(points_world, yaws):
        px = (x - ox) / res
        py = img_h - 1 - (y - oy) / res
        pxs.append(px)
        pys.append(py)
        arrow_len = 15
        ax.annotate(
            "", xy=(px + arrow_len * np.cos(yaw), py - arrow_len * np.sin(yaw)),
            xytext=(px, py),
            arrowprops=dict(arrowstyle="->", color="blue", linewidth=1.5),
        )

    ax.plot(pxs, pys, "r-", linewidth=1, alpha=0.6)
    for i, (px, py) in enumerate(zip(pxs, pys)):
        ax.plot(px, py, "r+", markersize=12, markeredgewidth=2)
        ax.annotate(str(i + 1), (px, py), color="red", fontsize=10,
                    xytext=(4, 4), textcoords="offset points")

    ax.set_title("Waypoint preview  (red = path order, blue = heading)")
    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved preview overlay → {out_path}")


def print_initial_pose_hint(points_world, yaws, appended_origin=False):
    """map_localizer's initial_x/initial_y/initial_yaw (see
    localization/scripts/trigger_map_relocalize.py) default to
    (0.0, 0.0, 0.0) -- i.e. they assume the robot physically starts at the
    MAP FRAME's own origin. This is entirely independent of the waypoint
    ORDER in the output file (the robot's physical starting pose at
    power-on/relocalize time is whatever it actually is, regardless of
    which waypoint comes first or last in waypoints.yaml).

    appended_origin: True if interactive_pick() confirmed (0,0) is free
    on this map and appended it as the final waypoint -- in that case the
    robot's real start position already matches map_localizer's default
    seed exactly, so no override is needed. If False (origin wasn't free,
    or --no-seed-origin was used), fall back to the older behavior: warn
    that wp1's position needs to be told to map_localizer explicitly if
    the robot will actually be physically placed there before start."""
    print()
    if appended_origin:
        print("Robot's physical start == the map's own origin (0,0,0), "
              "which is exactly map_localizer's own default initial_x/y/"
              "yaw, and has been appended as the FINAL waypoint (closing "
              "the loop back to start/finish) -- as long as the robot is "
              "physically placed at the map's origin before pressing "
              "green light, NO initial pose override is needed.")
        return
    x0, y0 = points_world[0]
    yaw0 = yaws[0]
    print("If the robot physically starts at wp1's position, tell "
          "map_localizer's relocalization the truth (its own initial_x/y/"
          "yaw default to 0.0/0.0/0.0, which assumes the robot starts at "
          "the MAP's origin, not at wp1):")
    print(
        "  ros2 launch localization localization.launch.py \\\n"
        "      mode:=runtime \\\n"
        f"      initial_x:={x0:.4f} initial_y:={y0:.4f} "
        f"initial_yaw:={yaw0:.4f}"
    )


# ─── CLI ─────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Click-to-pick waypoints on a real map for "
                    "diy_waypoint_sequencer.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--map", required=True,
                    help="Map YAML to pick waypoints on (e.g. "
                         "maps/course_traced_smooth.yaml)")
    p.add_argument("--output", required=True,
                    help="Output waypoints.yaml path")
    p.add_argument("--overlay", default=None,
                    help="Optional output PNG for a visual sanity check")
    p.add_argument("--loop", action="store_true",
                    help="Set loop: true in the output (wrap back to the "
                         "first waypoint after the last)")
    p.add_argument("--label-prefix", default="wp",
                    help="Waypoint label prefix (default: 'wp' -> wp1, wp2, ...)")
    p.add_argument("--uniform-yaw", type=float, default=None,
                    help="Skip auto-heading and set every waypoint's yaw to "
                         "this fixed value (radians) instead")
    p.add_argument("--no-seed-origin", action="store_false", dest="seed_origin",
                    default=True,
                    help="Don't auto-append the map origin (0,0,0) as the "
                         "final waypoint -- output exactly the points you click")
    return p.parse_args()


def main():
    args = parse_args()

    meta, pgm_path = load_map_yaml(args.map)
    pgm = load_pgm(pgm_path)

    points_world, appended_origin = interactive_pick(meta, pgm,
                                                      seed_origin=args.seed_origin)

    if args.uniform_yaw is not None:
        yaws = [args.uniform_yaw] * len(points_world)
    else:
        yaws = compute_headings(points_world, loop=args.loop)
        # No special-casing needed for the appended origin here -- it's
        # the LAST point now, so compute_headings' existing last-point
        # logic already does the right thing: with --loop it bears back
        # toward wp1 (the real next stop after closing the loop), and
        # without --loop it just keeps the arriving heading, same as any
        # other waypoint.

    write_waypoints_yaml(args.output, points_world, yaws, args.loop,
                          args.label_prefix)

    if args.overlay:
        draw_overlay(pgm, meta, points_world, yaws, args.overlay)

    print_initial_pose_hint(points_world, yaws, appended_origin=appended_origin)


if __name__ == "__main__":
    main()
