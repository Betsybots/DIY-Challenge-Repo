#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/build_course_map.sh — pcd -> final smoothed course map, one command
# ══════════════════════════════════════════════════════════════════════════════
# Chains the full map-generation pipeline built up over this session's real
# debugging (see docs/reuse_plan_step1.md Steps 19-23 for the full history of
# why each stage exists):
#
#   1. pcd_to_pgm.py       — raw 2D projection of the .pcd, frame-correct by
#                            construction (origin derived from the point
#                            cloud's own bounding box, not assumed)
#   2. draw_map_borders.py — hand-trace the outer wall + inner obstacle(s)
#                            over the raw projection. INTERACTIVE the first
#                            time per map; every subsequent run on the SAME
#                            --output prefix reuses the saved trace
#                            automatically (see --retrace to force redrawing)
#   3. preprocess_map.py   — smooths the hand-traced walls into clean curves
#                            and marks the inner ring's interior unavailable
#                            (--isolate-largest-free), at a real-world wall
#                            thickness you specify in inches
#
# WHY THIS ORDER, NOT preprocess_map.py's OWN automatic wall detection alone:
# confirmed on real course data (LIO_Localization's refined_map_1.pcd) that
# automatic gap-bridging (--close-kernel/--inner-close-kernel) cannot close a
# REAL scan gap (zero points in some stretch) -- only a human tracing the
# boundary by eye can bridge that reliably. Since the hand-trace has no gaps
# by construction, preprocess_map.py's smoothing pass on top of it works
# perfectly every time (no fragmented walls, no gap-bridging tuning needed).
#
# Usage:
#   scripts/build_course_map.sh --pcd <path> --output <prefix> \
#       --z-min <m> --z-max <m> [options]
#
#   First, if you don't know the z-slab yet:
#   scripts/build_course_map.sh --pcd <path> --preview-only
#
# Options:
#   --pcd PATH             Input .pcd file (required)
#   --output PREFIX        Output path prefix, e.g. maps/course_final
#                           (required unless --preview-only)
#   --z-min / --z-max      Wall height slab in metres (required unless
#                           --preview-only — see the printed histogram to
#                           choose these)
#   --resolution M         Raw projection resolution, metres/px (default 0.05)
#   --point-radius-px N    Splat radius for sparse scans (default 3 — fixes
#                           the common case of individually-too-far-apart
#                           wall points; see pcd_to_pgm.py's own docs)
#   --keep-walls N         Number of wall components to keep (default 2:
#                           outer + one inner obstacle)
#   --wall-thickness-in N  FINAL map's real-world wall thickness in inches
#                           (default 5)
#   --retrace              Force re-tracing borders even if a previous trace
#                           is saved for this --output prefix
#   --preview-only         Just run pcd_to_pgm.py's z-histogram preview and
#                           exit (use this first to choose --z-min/--z-max)
#
# Output (final, ready for nav2_map_server / diy_motion_planner):
#   <output>_traced_smooth.pgm
#   <output>_traced_smooth.yaml
# Intermediate artifacts (kept for inspection/debugging, safe to ignore):
#   <output>_raw.pgm/.yaml            — pcd_to_pgm.py's raw projection
#   <output>_traced.pgm/.yaml         — draw_map_borders.py's hand-traced result
#   <output>_traced_borders.yaml      — the SAVED trace (reused on future runs)
#   <output>_traced_preview.png       — visual overlay of the trace
#   <output>_traced_diff.png          — preprocess_map.py's before/after diff
#
# Then pick waypoints on the final map:
#   python3 scripts/pick_waypoints.py --map <output>_traced_smooth.yaml \
#       --output maps/test_waypoints.yaml --overlay maps/test_waypoints_preview.png --loop
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PCD=""
OUTPUT=""
Z_MIN=""
Z_MAX=""
RESOLUTION="0.05"
POINT_RADIUS_PX="3"
KEEP_WALLS="2"
WALL_THICKNESS_IN="5"
RETRACE="false"
PREVIEW_ONLY="false"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pcd)              PCD="$2"; shift 2 ;;
        --output)            OUTPUT="$2"; shift 2 ;;
        --z-min)             Z_MIN="$2"; shift 2 ;;
        --z-max)             Z_MAX="$2"; shift 2 ;;
        --resolution)        RESOLUTION="$2"; shift 2 ;;
        --point-radius-px)   POINT_RADIUS_PX="$2"; shift 2 ;;
        --keep-walls)        KEEP_WALLS="$2"; shift 2 ;;
        --wall-thickness-in) WALL_THICKNESS_IN="$2"; shift 2 ;;
        --retrace)           RETRACE="true"; shift ;;
        --preview-only)      PREVIEW_ONLY="true"; shift ;;
        *)
            echo "[build_course_map] ERROR: Unknown argument '$1'" >&2
            exit 1
            ;;
    esac
done

if [[ -z "${PCD}" ]]; then
    echo "[build_course_map] ERROR: --pcd is required" >&2
    exit 1
fi
if [[ ! -f "${PCD}" ]]; then
    echo "[build_course_map] ERROR: File not found: ${PCD}" >&2
    exit 1
fi

cd "${REPO_ROOT}"

if [[ "${PREVIEW_ONLY}" == "true" ]]; then
    echo "[build_course_map] Step 1/3 preview: z-histogram for ${PCD}"
    python3 scripts/pcd_to_pgm.py --input "${PCD}" --preview-only
    echo ""
    echo "[build_course_map] Pick --z-min/--z-max from the histogram above, "
    echo "then re-run without --preview-only."
    exit 0
fi

if [[ -z "${OUTPUT}" ]]; then
    echo "[build_course_map] ERROR: --output is required (unless --preview-only)" >&2
    exit 1
fi
if [[ -z "${Z_MIN}" || -z "${Z_MAX}" ]]; then
    echo "[build_course_map] ERROR: --z-min/--z-max are required (unless --preview-only)." >&2
    echo "  Run first with --preview-only to see the real z-histogram:" >&2
    echo "    scripts/build_course_map.sh --pcd ${PCD} --preview-only" >&2
    exit 1
fi

RAW_PREFIX="${OUTPUT}_raw"
TRACED_PREFIX="${OUTPUT}_traced"
BORDERS_FILE="${TRACED_PREFIX}_borders.yaml"
TRACED_PREVIEW="${TRACED_PREFIX}_preview.png"

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║  Build Course Map — pcd -> raw -> traced -> smoothed       ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo "  pcd            : ${PCD}"
echo "  output prefix   : ${OUTPUT}"
echo "  z-slab          : [${Z_MIN}, ${Z_MAX}]"
echo "  resolution      : ${RESOLUTION} m/px"
echo "  wall thickness  : ${WALL_THICKNESS_IN} in (final map only)"
echo ""

echo "[build_course_map] Step 1/3: pcd -> raw 2D projection"
python3 scripts/pcd_to_pgm.py \
    --input "${PCD}" \
    --output "${RAW_PREFIX}" \
    --z-min "${Z_MIN}" --z-max "${Z_MAX}" \
    --resolution "${RESOLUTION}" \
    --point-radius-px "${POINT_RADIUS_PX}"

echo ""
echo "[build_course_map] Step 2/3: trace borders"
if [[ -f "${BORDERS_FILE}" && "${RETRACE}" != "true" ]]; then
    echo "  Reusing saved trace: ${BORDERS_FILE} (pass --retrace to redraw)"
else
    echo "  Opening interactive tracer — trace the outer wall ('o'), then"
    echo "  each inner obstacle ('i'). Close the window when done."
fi
RETRACE_FLAG=()
if [[ "${RETRACE}" == "true" ]]; then
    RETRACE_FLAG=(--retrace)
fi
python3 scripts/draw_map_borders.py \
    --map "${RAW_PREFIX}.yaml" \
    --output "${TRACED_PREFIX}" \
    --points-file "${BORDERS_FILE}" \
    --overlay "${TRACED_PREVIEW}" \
    "${RETRACE_FLAG[@]}"

echo ""
echo "[build_course_map] Step 3/3: smooth + mark inner ring unavailable"
python3 scripts/preprocess_map.py \
    --input "${TRACED_PREFIX}.pgm" \
    --yaml "${TRACED_PREFIX}.yaml" \
    --keep-walls "${KEEP_WALLS}" \
    --isolate-largest-free \
    --wall-thickness-in "${WALL_THICKNESS_IN}" \
    --no-display

FINAL_PGM="${TRACED_PREFIX}_smooth.pgm"
FINAL_YAML="${TRACED_PREFIX}_smooth.yaml"

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║  Done                                                       ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo "  Final map: ${FINAL_PGM}"
echo "             ${FINAL_YAML}"
echo ""
echo "  Next: pick waypoints on the final map"
echo "    python3 scripts/pick_waypoints.py --map ${FINAL_YAML} \\"
echo "        --output maps/test_waypoints.yaml \\"
echo "        --overlay maps/test_waypoints_preview.png --loop"
echo ""
