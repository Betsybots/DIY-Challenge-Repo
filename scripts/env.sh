#!/usr/bin/env bash

# ══════════════════════════════════════════════════════════════════════════════
# scripts/env.sh — Environment bootstrap for DIY Challenge Robot
# ══════════════════════════════════════════════════════════════════════════════
#
# Usage:
#   source ~/DIY-Challenge-Repo/scripts/env.sh laptop
#
# IMPORTANT:
#   This file is meant to be SOURCED.
#   Do not use "set -e" or "set -u" here because they can affect the
#   interactive shell that sourced this file.
# ══════════════════════════════════════════════════════════════════════════════

# ------------------------------------------------------------------------------
# Resolve repository root
# ------------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ------------------------------------------------------------------------------
# Determine profile
# ------------------------------------------------------------------------------

_profile="${1:-${DIY_ROBOT_PROFILE:-}}"

if [[ -z "${_profile}" ]]; then
    _host="$(hostname)"

    case "${_host}" in
        jetson*|nano*|orin*)
            _profile="jetson"
            ;;
        raspi*|rpi*|pi*)
            _profile="raspi"
            ;;
        *)
            _profile="laptop"
            ;;
    esac

    echo "[env.sh] Auto-detected profile: ${_profile} (hostname=${_host})"
fi

# ------------------------------------------------------------------------------
# Load profile
# ------------------------------------------------------------------------------

_profile_file="${REPO_ROOT}/profiles/${_profile}.env"

if [[ ! -f "${_profile_file}" ]]; then
    echo "[env.sh] ERROR: Profile file not found:"
    echo "          ${_profile_file}"
    echo
    echo "[env.sh] Available profiles:"
    ls -1 "${REPO_ROOT}/profiles/" 2>/dev/null || true
    return 1 2>/dev/null || exit 1
fi

# Profile files should not be able to terminate the interactive shell.
source "${_profile_file}"

# ------------------------------------------------------------------------------
# Source ROS 2 Humble
# ------------------------------------------------------------------------------

if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
else
    echo "[env.sh] WARNING: ROS 2 Humble setup not found:"
    echo "          /opt/ros/humble/setup.bash"
fi

# ------------------------------------------------------------------------------
# Source micro-ROS workspace
# ------------------------------------------------------------------------------

if [[ -n "${DIY_MICRO_ROS_WS:-}" ]] &&
   [[ -f "${DIY_MICRO_ROS_WS}/install/setup.bash" ]]; then

    source "${DIY_MICRO_ROS_WS}/install/setup.bash"
fi

# ------------------------------------------------------------------------------
# Source Hesai workspace
# ------------------------------------------------------------------------------

if [[ -n "${DIY_HESAI_WS:-}" ]]; then

    if [[ -f "${DIY_HESAI_WS}/install/setup.bash" ]]; then
        source "${DIY_HESAI_WS}/install/setup.bash"

    elif [[ -f "${DIY_HESAI_WS}/install/local_setup.bash" ]]; then
        source "${DIY_HESAI_WS}/install/local_setup.bash"
    fi

fi

# ------------------------------------------------------------------------------
# Source third-party workspace
# ------------------------------------------------------------------------------

if [[ -n "${DIY_THIRD_PARTY_WS:-}" ]] &&
   [[ -f "${DIY_THIRD_PARTY_WS}/install/setup.bash" ]]; then

    source "${DIY_THIRD_PARTY_WS}/install/setup.bash"
fi

# ------------------------------------------------------------------------------
# Source main DIY workspace
# ------------------------------------------------------------------------------

if [[ -n "${DIY_ROS_WS:-}" ]] &&
   [[ -f "${DIY_ROS_WS}/install/setup.bash" ]]; then

    source "${DIY_ROS_WS}/install/setup.bash"
fi

# ------------------------------------------------------------------------------
# ROS logging
# ------------------------------------------------------------------------------

if [[ -n "${DIY_LOG_LEVEL:-}" ]]; then
    export RCUTILS_LOGGING_SEVERITY_THRESHOLD="${DIY_LOG_LEVEL}"
fi

# ------------------------------------------------------------------------------
# Final status
# ------------------------------------------------------------------------------

echo "[env.sh] Environment ready. Profile=${DIY_ROBOT_PROFILE:-<not set>}"
echo "[env.sh] ROS_DISTRO=${ROS_DISTRO:-<not set>}, WS=${DIY_ROS_WS:-<not set>}"