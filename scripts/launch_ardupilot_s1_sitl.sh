#!/usr/bin/env bash
# ==============================================================================
# NIDAR RescueSwarm — Single-Drone S1 ArduCopter SITL Launcher
# ==============================================================================
# Launches ArduCopter SITL for Search Drone S1:
# - Instance ID: 0 (Internal TCP: 5760)
# - System ID:   1 (MAVLink sysid=1)
# - Output:      udp:127.0.0.1:14550 (forwarded via MAVProxy)
# - Location:    12.972000, 77.594500, 920m MSL, Heading 0°
#
# Target Environment: Ubuntu 22.04 LTS
#
# Prerequisites on Ubuntu:
#   sudo apt update && sudo apt install -y git python3-pip python3-venv tmux
#   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
#   cd ardupilot
#   Tools/environment_install/install-prereqs-ubuntu.sh -y
#   source ~/.profile
# ==============================================================================

set -euo pipefail

# 1. Resolve ArduPilot Root Directory
ARDUPILOT_ROOT="${ARDUPILOT_ROOT:-$HOME/ardupilot}"
if [ ! -d "${ARDUPILOT_ROOT}" ]; then
    if [ -d "../ardupilot" ]; then
        ARDUPILOT_ROOT="$(cd ../ardupilot && pwd)"
    fi
fi

if [ ! -d "${ARDUPILOT_ROOT}" ]; then
    echo "[ERROR] ArduPilot directory not found at '${ARDUPILOT_ROOT}'."
    echo "Please clone ArduPilot or export ARDUPILOT_ROOT=/path/to/ardupilot"
    echo "  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git $HOME/ardupilot"
    exit 1
fi

VEHICLE="ArduCopter"
INSTANCE=0
SYSID=1
PORT=14550
LOCATION="12.972000,77.594500,920,0"

# 2. Add autotest to PATH if needed
if ! command -v sim_vehicle.py &> /dev/null; then
    if [ -f "${ARDUPILOT_ROOT}/Tools/autotest/sim_vehicle.py" ]; then
        export PATH="${ARDUPILOT_ROOT}/Tools/autotest:${PATH}"
    else
        echo "[ERROR] 'sim_vehicle.py' not found in PATH or at ${ARDUPILOT_ROOT}/Tools/autotest/sim_vehicle.py"
        exit 1
    fi
fi

# 3. Check whether SITL binary has been compiled
BUILD_FLAGS=()
if [ -f "${ARDUPILOT_ROOT}/build/sitl/bin/arducopter" ]; then
    echo "  [INFO] Found pre-compiled SITL binary at ${ARDUPILOT_ROOT}/build/sitl/bin/arducopter."
    BUILD_FLAGS+=("--no-rebuild")
else
    echo "  [INFO] SITL binary not found. sim_vehicle.py will compile ArduCopter SITL first (initial run only)..."
fi

echo "================================================================================"
echo "  NIDAR RescueSwarm — Search Drone S1 ArduCopter SITL Launcher"
echo "================================================================================"
echo "  ArduPilot Root:    ${ARDUPILOT_ROOT}"
echo "  Vehicle Type:      ${VEHICLE}"
echo "  Instance ID:       ${INSTANCE} (Internal TCP: 5760)"
echo "  Target System ID:  ${SYSID}"
echo "  MAVLink UDP Port:  ${PORT} (udp:127.0.0.1:${PORT})"
echo "  Origin GPS:        ${LOCATION}"
echo "================================================================================"
echo "  OPERATOR INSTRUCTIONS:"
echo "  1. Once MAVProxy starts, wait ~15-20s for EKF3 to attain GPS lock."
echo "  2. In a separate terminal, run the S1 verification suite:"
echo "       PYTHONPATH=src .venv/bin/python scripts/verify_s1_real_sitl.py --connection udp:127.0.0.1:14550"
echo "  3. To stop SITL safely:"
echo "       Press Ctrl+C in this terminal, or run: pkill -f 'arducopter|sim_vehicle.py|mavproxy.py'"
echo "================================================================================"

# Execute from ArduCopter directory to keep parameter and eeprom logs contained
cd "${ARDUPILOT_ROOT}/ArduCopter"

exec sim_vehicle.py \
    -v "${VEHICLE}" \
    -I "${INSTANCE}" \
    --sysid "${SYSID}" \
    --custom-location="${LOCATION}" \
    --out "udp:127.0.0.1:${PORT}" \
    "${BUILD_FLAGS[@]}" \
    "$@"
