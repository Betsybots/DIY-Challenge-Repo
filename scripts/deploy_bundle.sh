#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# scripts/deploy_bundle.sh — Deploy calibration & map bundle to robot over SSH
# ══════════════════════════════════════════════════════════════════════════════
# Copies the competition-critical file bundle from the developer laptop to the
# target robot (Jetson or Pi) via rsync over SSH.
#
# Bundle contents deployed to robot:
#   calibration/                       IMU + extrinsic YAML results
#   src/localization/config/       FAST-LIO2 + EKF configs (with calibration)
#   src/diy_robot_description/urdf/    Updated URDF with real extrinsic transforms
#   src/challenge_bringup/config/      nav2_params + collision monitor
#   src/challenge_bringup/maps/        Static map YAML + PGM
#
# Usage:
#   ./scripts/deploy_bundle.sh <robot_host> [--user <username>] [--dry-run]
#
# Examples:
#   ./scripts/deploy_bundle.sh jetson.local
#   ./scripts/deploy_bundle.sh 192.168.1.100 --user ubuntu
#   ./scripts/deploy_bundle.sh jetson.local --dry-run
# ══════════════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── Parse arguments ───────────────────────────────────────────────────────────
ROBOT_HOST=""
ROBOT_USER="ubuntu"
DRY_RUN=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --user)    ROBOT_USER="$2"; shift 2 ;;
        --dry-run) DRY_RUN=true; shift ;;
        --*)       echo "[deploy_bundle] Unknown flag: $1" >&2; exit 1 ;;
        *)
            if [[ -z "${ROBOT_HOST}" ]]; then
                ROBOT_HOST="$1"
            fi
            shift
            ;;
    esac
done

if [[ -z "${ROBOT_HOST}" ]]; then
    echo "Usage: $0 <robot_host> [--user <username>] [--dry-run]" >&2
    exit 1
fi

REMOTE="${ROBOT_USER}@${ROBOT_HOST}"
REMOTE_REPO_ROOT="\${HOME}/ros2_ws/src/DIY-Challenge-Repo"
RSYNC_OPTS=(-avz --progress --checksum)
[[ "${DRY_RUN}" == "true" ]] && RSYNC_OPTS+=(--dry-run)

echo ""
echo "╔══════════════════════════════════╗"
echo "║  DIY Challenge — Deploy Bundle   ║"
echo "╚══════════════════════════════════╝"
echo "  Target : ${REMOTE}"
echo "  Source : ${REPO_ROOT}"
[[ "${DRY_RUN}" == "true" ]] && echo "  Mode   : DRY RUN (no files transferred)"
echo ""

# ── Verify SSH reachability ───────────────────────────────────────────────────
if ! ssh -q -o ConnectTimeout=5 "${REMOTE}" exit 2>/dev/null; then
    echo "[deploy_bundle] ERROR: Cannot SSH to ${REMOTE}" >&2
    echo "[deploy_bundle] Ensure the robot is on the same network and SSH keys are set up." >&2
    exit 1
fi
echo "[deploy_bundle] SSH connection OK"

# ── Ensure remote repo exists ─────────────────────────────────────────────────
# shellcheck disable=SC2029
ssh "${REMOTE}" "mkdir -p \$(eval echo ${REMOTE_REPO_ROOT})"

# ── Deploy file bundles ───────────────────────────────────────────────────────

_deploy() {
    local src="$1"
    local dest_rel="$2"
    local label="${3:-}"
    if [[ ! -e "${REPO_ROOT}/${src}" ]]; then
        echo "[deploy_bundle] SKIP (not found locally): ${src}"
        return
    fi
    echo "[deploy_bundle] Deploying: ${label:-${src}}"
    # shellcheck disable=SC2029
    rsync "${RSYNC_OPTS[@]}" \
        "${REPO_ROOT}/${src}" \
        "${REMOTE}:$(ssh "${REMOTE}" echo "${REMOTE_REPO_ROOT}")/${dest_rel}"
}

# Calibration results
_deploy "calibration/" "calibration/" "Calibration results"

# FAST-LIO2 + EKF + navsat configs
_deploy "src/localization/config/" "src/localization/config/" "Localization configs"

# URDF (updated extrinsics)
_deploy "src/diy_robot_description/urdf/" "src/diy_robot_description/urdf/" "URDF"

# Nav2 params + collision monitor
_deploy "src/challenge_bringup/config/" "src/challenge_bringup/config/" "Nav2 params"

# Maps
_deploy "src/challenge_bringup/maps/" "src/challenge_bringup/maps/" "Maps"

echo ""
echo "[deploy_bundle] ✓ Deployment complete."
echo ""
echo "Next steps on the robot (${REMOTE}):"
echo "  1. cd ~/ros2_ws"
echo "  2. colcon build --packages-select localization diy_robot_description challenge_bringup"
echo "  3. source install/setup.bash"
echo "  4. ./ros2_ws/src/DIY-Challenge-Repo/scripts/health_check.sh jetson"
