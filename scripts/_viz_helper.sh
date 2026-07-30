#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/_viz_helper.sh — Shared visualization launcher
# ══════════════════════════════════════════════════════════════════════════════
# Source this file from test scripts to get viz argument parsing and launch.
#
# Usage in a test script:
#   source "${SCRIPT_DIR}/_viz_helper.sh"
#   viz_parse_args "$@"        # parses --viz flag, leaves remaining args in REMAINING_ARGS
#   viz_start <rviz_config>    # launches selected viz tool in background
#
# After viz_parse_args:
#   VIZ_MODE  = rviz | foxglove | none  (default: none)
#   REMAINING_ARGS = original args minus --viz and its value
# ══════════════════════════════════════════════════════════════════════════════

VIZ_MODE="none"
REMAINING_ARGS=()

viz_parse_args() {
    local args=("$@")
    local i=0
    while [[ $i -lt ${#args[@]} ]]; do
        case "${args[$i]}" in
            --viz)
                i=$(( i + 1 ))
                VIZ_MODE="${args[$i]:-rviz}"
                ;;
            --viz=*)
                VIZ_MODE="${args[$i]#--viz=}"
                ;;
            *)
                REMAINING_ARGS+=("${args[$i]}")
                ;;
        esac
        i=$(( i + 1 ))
    done

    case "${VIZ_MODE}" in
        rviz|foxglove|none) ;;
        *)
            echo "[viz] ERROR: Unknown --viz mode '${VIZ_MODE}'. Use: rviz | foxglove | none" >&2
            exit 1
            ;;
    esac
}

viz_start() {
    local rviz_config="${1:-}"

    case "${VIZ_MODE}" in
        rviz)
            if [[ -z "${rviz_config}" || ! -f "${rviz_config}" ]]; then
                echo "[viz] WARNING: RViz config not found: '${rviz_config}' — launching blank RViz" >&2
                rviz2 &
            else
                echo "[viz] Launching RViz: ${rviz_config}"
                rviz2 -d "${rviz_config}" &
            fi
            VIZ_PID=$!
            echo "[viz] RViz started (PID=${VIZ_PID})"
            ;;
        foxglove)
            if ! ros2 pkg list 2>/dev/null | grep -q foxglove_bridge; then
                echo "[viz] WARNING: foxglove_bridge not found — install with:" >&2
                echo "       sudo apt install ros-humble-foxglove-bridge" >&2
                echo "[viz] Skipping Foxglove." >&2
                return
            fi
            echo "[viz] Launching Foxglove bridge on ws://localhost:8765"
            echo "[viz] Open https://app.foxglove.dev → connect to ws://<robot-ip>:8765"
            ros2 launch foxglove_bridge foxglove_bridge_launch.xml &
            VIZ_PID=$!
            echo "[viz] Foxglove bridge started (PID=${VIZ_PID})"
            ;;
        none)
            echo "[viz] No visualization requested (use --viz rviz or --viz foxglove)"
            ;;
    esac
}
