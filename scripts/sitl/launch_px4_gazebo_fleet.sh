#!/usr/bin/env bash
# ==============================================================================
# NIDAR RescueSwarm — 6-Drone PX4 SITL + Gazebo Harmonic Fleet Launcher
# ==============================================================================
# Launches 6 isolated PX4 SITL instances in Gazebo Sim (Harmonic / gz-sim8):
#
# Drone | Role     | Instance | SysID | Model       | Offboard Port | Start Position (X, Y, Z)
# ------|----------|----------|-------|-------------|---------------|-------------------------
#  S1   | Search   |    0     |   1   | x500_gimbal | udp:127.0.0.1:14540 |  0.0,  0.0, 0.1
#  S2   | Search   |    1     |   2   | x500_gimbal | udp:127.0.0.1:14541 |  5.0,  0.0, 0.1
#  S3   | Search   |    2     |   3   | x500_gimbal | udp:127.0.0.1:14542 | 10.0,  0.0, 0.1
#  D1   | Delivery |    3     |   4   | x500        | udp:127.0.0.1:14543 |  0.0,  5.0, 0.1
#  D2   | Delivery |    4     |   5   | x500        | udp:127.0.0.1:14544 |  5.0,  5.0, 0.1
#  D3   | Delivery |    5     |   6   | x500        | udp:127.0.0.1:14545 | 10.0,  5.0, 0.1
# ==============================================================================

set -euo pipefail

HEADLESS="${1:-}"
IS_HEADLESS=false
if [ "${HEADLESS}" == "--headless" ]; then
    IS_HEADLESS=true
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "================================================================================"
echo "  NIDAR RescueSwarm — 6-Drone PX4 SITL + Gazebo Fleet Launcher"
echo "================================================================================"
echo "  Repository Root: ${REPO_ROOT}"

# 1. Environment Preflight Checks
if ! command -v gz &>/dev/null; then
    echo "[ERROR] Gazebo 'gz' command not found. Please install Gazebo Harmonic (gz-sim8)."
    exit 1
fi
GZ_VER=$(gz sim --versions 2>/dev/null | head -n 1 || echo "unknown")
echo "  Gazebo Sim Version: ${GZ_VER}"

PX4_DIR="${PX4_AUTOPILOT_DIR:-}"
if [ -z "${PX4_DIR}" ] || [ ! -d "${PX4_DIR}" ]; then
    if [ -d "${REPO_ROOT}/../PX4-Autopilot" ]; then
        PX4_DIR="$(cd "${REPO_ROOT}/../PX4-Autopilot" && pwd)"
    elif [ -d "$HOME/PX4-Autopilot" ]; then
        PX4_DIR="$HOME/PX4-Autopilot"
    fi
fi

if [ ! -d "${PX4_DIR}" ]; then
    echo "[ERROR] PX4-Autopilot directory not found at '${PX4_DIR}'."
    echo "Please set PX4_AUTOPILOT_DIR=/path/to/PX4-Autopilot"
    exit 1
fi
echo "  PX4-Autopilot Path: ${PX4_DIR}"

BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
if [ ! -f "${BUILD_DIR}/bin/px4" ]; then
    echo "  [INFO] Compiling px4_sitl_default once before launching fleet..."
    (cd "${PX4_DIR}" && DONT_RUN=1 make px4_sitl_default)
fi

PID_FILE="${REPO_ROOT}/.sitl_fleet.pids"

# 2. Cleanup previous repo simulation instances if tracked
if [ -f "${PID_FILE}" ]; then
    echo "  Found existing fleet PID file. Cleaning up previous fleet instances..."
    PREV_PIDS=$(cat "${PID_FILE}" 2>/dev/null | tr '\n' ' ' || true)
    if [ -n "${PREV_PIDS}" ]; then
        # shellcheck disable=SC2086
        kill -TERM ${PREV_PIDS} 2>/dev/null || true
        sleep 1
        # shellcheck disable=SC2086
        kill -KILL ${PREV_PIDS} 2>/dev/null || true
    fi
    rm -f "${PID_FILE}"
fi
sleep 1

# 3. Setup Trap for clean shutdown on exit
cleanup() {
    echo ""
    echo "================================================================================"
    echo "  [SHUTDOWN] Terminating 6-Drone PX4 SITL Fleet..."
    echo "================================================================================"
    if [ -f "${PID_FILE}" ]; then
        PIDS=$(cat "${PID_FILE}" 2>/dev/null | tr '\n' ' ' || true)
        if [ -n "${PIDS}" ]; then
            echo "  Sending SIGTERM to tracked fleet processes: ${PIDS}"
            # shellcheck disable=SC2086
            kill -TERM ${PIDS} 2>/dev/null || true
            sleep 2
            # shellcheck disable=SC2086
            kill -KILL ${PIDS} 2>/dev/null || true
        fi
        rm -f "${PID_FILE}"
    fi
    echo "  Fleet terminated cleanly."
}
trap cleanup EXIT INT TERM

# 4. Start Gazebo World
WORLD_FILE="${REPO_ROOT}/worlds/rescue_swarm_field.sdf"
if [ ! -f "${WORLD_FILE}" ]; then
    WORLD_FILE="${PX4_DIR}/Tools/simulation/gz/worlds/default.sdf"
fi
WORLD_NAME="rescue_swarm_field"
if [ -f "${WORLD_FILE}" ]; then
    DETECTED_NAME=$(grep -oP '<world name="\K[^"]+' "${WORLD_FILE}" 2>/dev/null || true)
    if [ -n "${DETECTED_NAME}" ]; then
        WORLD_NAME="${DETECTED_NAME}"
    fi
fi
echo "  Launching Gazebo with world '${WORLD_NAME}': ${WORLD_FILE}"

# Setup Gazebo resource paths
export GZ_SIM_RESOURCE_PATH="${PX4_DIR}/Tools/simulation/gz/models:${REPO_ROOT}/worlds"
export PX4_GZ_STANDALONE=1

if [ "${IS_HEADLESS}" = true ]; then
    gz sim -s -r "${WORLD_FILE}" &
else
    gz sim -r "${WORLD_FILE}" &
fi
GZ_PID=$!
echo "${GZ_PID}" > "${PID_FILE}"
sleep 3

# 5. Launch 6 Isolated PX4 SITL Instances
# Format: INSTANCE:SYSID:MODEL:AUTOSTART:POS_X:POS_Y:NAME:ROLE
DRONE_CONFIGS=(
    "0:1:x500_gimbal:4019:0.0:0.0:S1:search"
    "1:2:x500_gimbal:4019:5.0:0.0:S2:search"
    "2:3:x500_gimbal:4019:10.0:0.0:S3:search"
    "3:4:x500:4001:0.0:5.0:D1:delivery"
    "4:5:x500:4001:5.0:5.0:D2:delivery"
    "5:6:x500:4001:10.0:5.0:D3:delivery"
)

echo "--------------------------------------------------------------------------------"
echo "  Spawning 6 isolated vehicles into Gazebo..."
echo "--------------------------------------------------------------------------------"

for cfg in "${DRONE_CONFIGS[@]}"; do
    IFS=":" read -r INST SYSID MODEL AUTOSTART POS_X POS_Y NAME ROLE <<< "${cfg}"
    INSTANCE_DIR="${BUILD_DIR}/instance_${INST}"
    mkdir -p "${INSTANCE_DIR}"

    echo "  -> Spawning ${NAME} (${ROLE}) [Instance ${INST}, SysID ${SYSID}, Model ${MODEL}, Autostart ${AUTOSTART}] at (${POS_X}, ${POS_Y})"

    # Spawn entity into Gazebo Harmonic via gz service
    gz service -s "/world/${WORLD_NAME}/create" \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 2000 \
        --req "sdf_filename: '${PX4_DIR}/Tools/simulation/gz/models/${MODEL}/model.sdf', name: '${MODEL}_${INST}', pose: {position: {x: ${POS_X}, y: ${POS_Y}, z: 0.15}}}" || true

    # Start isolated PX4 instance
    (
        cd "${INSTANCE_DIR}"
        export PX4_SYS_AUTOSTART="${AUTOSTART}"
        export PX4_SIM_MODEL="gz_${MODEL}"
        export PX4_GZ_MODEL_NAME="${MODEL}_${INST}"
        export PX4_SIM_INSTANCE="${INST}"
        exec "${BUILD_DIR}/bin/px4" -i "${INST}" -d "${BUILD_DIR}/etc" > "${INSTANCE_DIR}/px4.log" 2>&1
    ) &
    PX4_PID=$!
    echo "${PX4_PID}" >> "${PID_FILE}"
    sleep 1
done

echo "================================================================================"
echo "  [READY] All 6 PX4 SITL instances started successfully!"
echo "  PIDs recorded in ${PID_FILE}"
echo "  To verify vehicle connectivity, run:"
echo "    python scripts/sitl/verify_fleet_connectivity.py"
echo "  Press Ctrl+C to terminate the fleet."
echo "================================================================================"

# Wait for children
wait
