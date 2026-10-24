#!/usr/bin/env bash
# ==============================================================================
# NIDAR RescueSwarm — Stop PX4 SITL & Gazebo Fleet
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PID_FILE="${REPO_ROOT}/.sitl_fleet.pids"

FORCE="${1:-}"

if [ -f "${PID_FILE}" ]; then
    PIDS=$(cat "${PID_FILE}" 2>/dev/null | tr '\n' ' ' || true)
    if [ -n "${PIDS}" ]; then
        echo "Stopping tracked RescueSwarm fleet processes (PIDs: ${PIDS})..."
        # shellcheck disable=SC2086
        kill -TERM ${PIDS} 2>/dev/null || true
        sleep 2
        # shellcheck disable=SC2086
        kill -KILL ${PIDS} 2>/dev/null || true
    fi
    rm -f "${PID_FILE}"
    echo "Tracked fleet simulation stopped cleanly."
else
    echo "No tracked fleet PID file found (${PID_FILE})."
    if [ "${FORCE}" == "--force" ]; then
        echo "Executing force cleanup (--force specified)..."
        pkill -TERM -x px4 || true
        pkill -TERM -f "gz sim" || true
        pkill -TERM -x MicroXRCEAgent || true
        sleep 1
        pkill -KILL -x px4 || true
        pkill -KILL -f "gz sim" || true
        pkill -KILL -x MicroXRCEAgent || true
        echo "Forced cleanup complete."
    else
        echo "No action taken. To forcefully terminate lingering simulator processes on this machine, run:"
        echo "  ./scripts/sitl/stop_px4_gazebo_fleet.sh --force"
    fi
fi
