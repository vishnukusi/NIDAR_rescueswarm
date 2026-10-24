#!/usr/bin/env bash
# ==============================================================================
# NIDAR RescueSwarm — 6-Drone ArduCopter SITL Fleet Launcher
# ==============================================================================
# Launches 6 isolated ArduCopter SITL instances across dedicated ports:
#
# Drone | Instance | Base TCP | SysID | MAVLink UDP Output | Initial GPS Offset
# ------|----------|----------|-------|--------------------|-------------------
#  S1   |    0     |   5760   |   1   |  udp:127.0.0.1:14550 | 12.972000, 77.594500
#  S2   |    1     |   5770   |   2   |  udp:127.0.0.1:14551 | 12.972050, 77.594500
#  S3   |    2     |   5780   |   3   |  udp:127.0.0.1:14552 | 12.972100, 77.594500
#  D1   |    3     |   5790   |   4   |  udp:127.0.0.1:14553 | 12.972000, 77.594550
#  D2   |    4     |   5800   |   5   |  udp:127.0.0.1:14554 | 12.972050, 77.594550
#  D3   |    5     |   5810   |   6   |  udp:127.0.0.1:14555 | 12.972100, 77.594550
#
# CRITICAL PREREQUISITE:
# - Do NOT launch all 6 drones until S1 single-vehicle validation is confirmed!
# - Ensure S1 passes 'scripts/verify_s1_real_sitl.py' first.
# ==============================================================================

set -euo pipefail

# 1. Gatekeeper: Ensure single-vehicle S1 validation has been completed
CONFIRMED="${1:-}"
if [ "${CONFIRMED}" != "--confirmed" ]; then
    echo "================================================================================"
    echo "  [SAFETY GATE] 6-Drone Fleet SITL Launch Guard"
    echo "================================================================================"
    echo "  Per RescueSwarm protocol, do NOT launch the 6-vehicle swarm until"
    echo "  single search drone S1 has passed all real MAVLink SITL checks!"
    echo ""
    echo "  Step 1: Run single-vehicle SITL:"
    echo "    ./scripts/launch_ardupilot_s1_sitl.sh"
    echo "  Step 2: Verify S1 via real MAVLink connection:"
    echo "    PYTHONPATH=src .venv/bin/python scripts/verify_s1_real_sitl.py --connection udp:127.0.0.1:14550"
    echo ""
    echo "  If S1 has PASSED, run this script with --confirmed to launch the 6-vehicle fleet:"
    echo "    $0 --confirmed"
    echo "================================================================================"
    exit 0
fi

# 2. Resolve ArduPilot Root Directory
ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/ardupilot}"
if [ ! -d "${ARDUPILOT_ROOT}" ]; then
    if [ -d "../ardupilot" ]; then
        ARDUPILOT_ROOT="$(cd ../ardupilot && pwd)"
    fi
fi

if [ ! -d "${ARDUPILOT_ROOT}" ]; then
    echo "[ERROR] ArduPilot directory not found at '${ARDUPILOT_ROOT}'."
    echo "Please clone ArduPilot or export ARDUPILOT_ROOT=/path/to/ardupilot"
    exit 1
fi

# 3. Ensure SITL binary is compiled before spawning parallel instances
if [ ! -f "${ARDUPILOT_ROOT}/build/sitl/bin/arducopter" ]; then
    echo "  [INFO] Compiling ArduCopter SITL once before launching multi-vehicle swarm..."
    (cd "${ARDUPILOT_ROOT}/ArduCopter" && sim_vehicle.py -v ArduCopter --build-target copter --no-start)
fi

# Define 6 drones: Name:Instance:SysID:Port:Lat:Lon
DRONES=(
    "S1:0:1:14550:12.972000:77.594500"
    "S2:1:2:14551:12.972050:77.594500"
    "S3:2:3:14552:12.972100:77.594500"
    "D1:3:4:14553:12.972000:77.594550"
    "D2:4:5:14554:12.972050:77.594550"
    "D3:5:6:14555:12.972100:77.594550"
)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RESCUE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${RESCUE_ROOT}/logs/sitl_fleet"
mkdir -p "${LOG_DIR}"

echo "================================================================================"
echo "  NIDAR RescueSwarm — Spawning 6-Drone ArduCopter SITL Fleet"
echo "================================================================================"

PIDS=()

cleanup() {
    echo ""
    echo "Shutting down all 6 SITL instances..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    pkill -f "ArduCopter.*-I" 2>/dev/null || true
    pkill -f "mavproxy.py.*1455" 2>/dev/null || true
    echo "All SITL instances stopped cleanly."
}
trap cleanup EXIT INT TERM

cd "${ARDUPILOT_ROOT}/ArduCopter"

for drone in "${DRONES[@]}"; do
    IFS=":" read -r NAME INSTANCE SYSID PORT LAT LON <<< "${drone}"
    LOCATION="${LAT},${LON},920,0"
    LOG_FILE="${LOG_DIR}/${NAME}_sitl.log"

    echo "  -> Spawning ${NAME}: Instance=${INSTANCE}, SysID=${SYSID}, Port=${PORT}, Origin=[${LAT}, ${LON}]"

    sim_vehicle.py \
        -v ArduCopter \
        -I "${INSTANCE}" \
        --sysid "${SYSID}" \
        --custom-location="${LOCATION}" \
        --out "udp:127.0.0.1:${PORT}" \
        --no-rebuild \
        > "${LOG_FILE}" 2>&1 &

    PID=$!
    PIDS+=("$PID")
    echo "     PID: ${PID} | Logs: ${LOG_FILE}"
    sleep 2 # Stagger launch by 2 seconds to avoid port race conditions
done

echo "================================================================================"
echo "  All 6 ArduCopter SITL instances launched successfully!"
echo "  To monitor, inspect files in '${LOG_DIR}/'."
echo "  To run RescueSwarm fleet integration:"
echo "    PYTHONPATH=src .venv/bin/python scripts/run_full_mission_sim.py"
echo "  Press Ctrl+C to stop all instances cleanly."
echo "================================================================================"

wait
