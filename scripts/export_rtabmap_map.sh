#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/export_rtabmap_map.sh — RTAB-Map .db -> map.pcd and/or Nav2 map.pgm+yaml
# ══════════════════════════════════════════════════════════════════════════════
# WHY THIS EXISTS
# ───────────────
# RTAB-Map (replaces loop_pgo, see repo history) keeps everything -- poses,
# per-keyframe scans, the pose graph -- in ONE SQLite .db, not a map.pcd +
# patches/ + poses.txt like loop_pgo used to produce. There is no built-in
# "dump per-keyframe patches" export, so map_hba's octomap_to_grid tool
# (which expects that exact loop_pgo-specific layout) cannot be fed directly
# from RTAB-Map -- see the NOTE comments in map_hba/config/octomap_to_grid.yaml
# and src/octomap_to_grid.cpp.
#
# Two independent outputs, pick one or both:
#   --cloud (default on)  Merged, loop-closed .pcd -- feed it into this
#                          repo's existing, hand-tuned "any XYZ .pcd -> Nav2
#                          map" pipeline (pcd_to_pgm.py -> draw_map_borders.py
#                          -> preprocess_map.py, chained by build_course_map.sh)
#                          instead of reproducing loop_pgo's patches/poses.txt
#                          format. Uses PCL's pcl_ply2pcd to convert
#                          rtabmap-export's .ply output to .pcd.
#   --map                  Nav2 map_server-ready .pgm + .yaml, built DIRECTLY
#                          by rtabmap-export from RTAB-Map's own per-node
#                          occupancy grids (Grid/* params, already configured
#                          in rtabmap_mapping.yaml) -- fully automatic, no
#                          hand-tracing step. Skips the .pcd pipeline
#                          entirely. Not chosen as the default here because
#                          build_course_map.sh's own header comment found
#                          automatic gap-bridging fails on this course's real
#                          scan gaps and hand-tracing was needed instead --
#                          --map's fully-automatic grid may hit the same
#                          issue. Compare both before trusting one.
# (--octomap is deliberately not wired in here: it's a 3D OctoMap .bt +
# colored cloud, for 3D obstacle visualization/awareness, not something
# Nav2's 2D map_server needs.)
#
# Uses only official RTAB-Map tooling (rtabmap-export, part of
# ros-humble-rtabmap) + PCL's own pcl_ply2pcd converter -- no custom
# database parsing.
#
# WHEN TO RUN THIS
# ────────────────
# Only AFTER cleanly stopping the mapping launch (Ctrl+C once, not kill -9,
# so RTAB-Map flushes/closes the database) -- not mid-session. To know your
# loop is actually closed first, while autonomous:=false is running, watch
# /rtabmap/info (rtabmap_msgs/msg/Info)'s loop_closure_id AND
# proximity_detection_id fields (nonzero = a constraint was just added tying
# the current position back to an earlier one -- this config has
# RGBD/ProximityBySpace enabled, so most same-course revisits show up as
# proximity_detection_id, not loop_closure_id):
#   ros2 topic echo /rtabmap/info --field loop_closure_id
#   ros2 topic echo /rtabmap/info --field proximity_detection_id
# or launch with use_rviz:=true and watch rtabmap_viz's graph view for a new
# edge connecting back to an earlier node.
#
# Usage:
#   scripts/export_rtabmap_map.sh [--db PATH] [--output-dir DIR] [--voxel M] \
#       [--cloud] [--map] [--no-cloud]
#
# Then, if using --cloud, continue with the existing pipeline, e.g.:
#   scripts/build_course_map.sh --pcd <output-dir>/rtabmap_map.pcd \
#       --output maps/course_final --z-min <m> --z-max <m>
set -euo pipefail

DB_PATH="src/challenge_bringup/maps/rtabmap.db"
OUTPUT_DIR="src/challenge_bringup/maps"
OUTPUT_NAME="rtabmap_map"
VOXEL="0.02"
DO_CLOUD=true
DO_MAP=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --db) DB_PATH="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --output-name) OUTPUT_NAME="$2"; shift 2 ;;
    --voxel) VOXEL="$2"; shift 2 ;;
    --cloud) DO_CLOUD=true; shift ;;
    --no-cloud) DO_CLOUD=false; shift ;;
    --map) DO_MAP=true; shift ;;
    -h|--help)
      echo "Usage: $0 [--db PATH] [--output-dir DIR] [--output-name NAME] [--voxel M] [--cloud] [--no-cloud] [--map]"
      exit 0
      ;;
    *) echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

if ! command -v rtabmap-export >/dev/null 2>&1; then
  echo "[ERROR] rtabmap-export not found. Install it: sudo apt install ros-humble-rtabmap-ros" >&2
  exit 1
fi
if $DO_CLOUD && ! command -v pcl_ply2pcd >/dev/null 2>&1; then
  echo "[ERROR] pcl_ply2pcd not found. Install it: sudo apt install pcl-tools" >&2
  exit 1
fi
if [[ ! -f "$DB_PATH" ]]; then
  echo "[ERROR] $DB_PATH does not exist -- run a mapping session first (rtabmap_mapping_launch.py)." >&2
  exit 1
fi
if ! $DO_CLOUD && ! $DO_MAP; then
  echo "[ERROR] Nothing to do: both --no-cloud and no --map given." >&2
  exit 1
fi

mkdir -p "$OUTPUT_DIR"

# --cloud and --map are independent rtabmap-export flags but share the same
# graph optimization pass, so request both together in one invocation.
EXPORT_ARGS=(--voxel "$VOXEL" --output "$OUTPUT_NAME" --output_dir "$OUTPUT_DIR")
$DO_CLOUD && EXPORT_ARGS+=(--cloud)
$DO_MAP && EXPORT_ARGS+=(--map)

echo "rtabmap-export: processing $DB_PATH ..."
rtabmap-export "${EXPORT_ARGS[@]}" "$DB_PATH"

if $DO_CLOUD; then
  PLY_PATH="$OUTPUT_DIR/${OUTPUT_NAME}_cloud.ply"
  PCD_PATH="$OUTPUT_DIR/${OUTPUT_NAME}.pcd"
  echo "pcl_ply2pcd: converting to $PCD_PATH ..."
  pcl_ply2pcd "$PLY_PATH" "$PCD_PATH"
  echo "Cloud ready: $PCD_PATH (feed into scripts/pcd_to_pgm.py / build_course_map.sh)."
fi

if $DO_MAP; then
  echo "Nav2 map ready: $OUTPUT_DIR/${OUTPUT_NAME}.pgm + $OUTPUT_DIR/${OUTPUT_NAME}.yaml"
fi
