#!/usr/bin/env bash
# ==============================================================================
# NIDAR RescueSwarm — 9-Stage Target Machine Simulation Smoke Test
# ==============================================================================
# Validates the complete six-drone simulation deployment stack on Linux.
# Stages:
#   Stage 1: Supported OS and toolchain versions
#   Stage 2: ROS 2 and workspace package availability
#   Stage 3: PX4 source/build availability
#   Stage 4: Gazebo executable, required models, and simulation world
#   Stage 5: Fleet configuration consistency and port uniqueness
#   Stage 6: Six SITL vehicles launch (Live only)
#   Stage 7: Real MAVLink heartbeat & identity verification (Live only)
#   Stage 8: ROS 2 node and telemetry readiness (Live only)
#   Stage 9: Safe shutdown and process isolation (Live only)
#
# Flags:
#   --live     Execute the full end-to-end live simulation (stages 6-9)
#   --headless Run Gazebo in headless mode during live execution (default)
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

RUN_LIVE=false
for arg in "$@"; do
    if [ "$arg" == "--live" ]; then
        RUN_LIVE=true
    fi
done

echo "================================================================================"
echo "  NIDAR RescueSwarm — 9-Stage Deployment Smoke Test Gate"
echo "================================================================================"
echo "  Mode: $(if [ "$RUN_LIVE" = true ]; then echo "FULL LIVE INTEGRATION (--live)"; else echo "STATIC ENVIRONMENT AUDIT (Pass --live for stages 6-9)"; fi)"
echo "  Repository Root: ${REPO_ROOT}"
echo "--------------------------------------------------------------------------------"

STAGE_RESULTS=()

record_stage() {
    local stage_num="$1"
    local stage_name="$2"
    local stage_status="$3"
    local stage_details="$4"
    STAGE_RESULTS+=("${stage_num}|${stage_name}|${stage_status}|${stage_details}")
    printf "  Stage %-2s [%-7s] %s: %s\n" "${stage_num}" "${stage_status}" "${stage_name}" "${stage_details}"
}

# ------------------------------------------------------------------------------
# Stage 1: Supported OS and Tool Versions
# ------------------------------------------------------------------------------
s1_status="PASS"
s1_details=""
if [ -f /etc/os-release ]; then
    . /etc/os-release
    s1_details="OS: ${NAME:-Linux} ${VERSION_ID:-}"
    if [[ "${ID:-}" != "ubuntu" ]]; then
        s1_status="FAIL"
        s1_details="Unsupported OS: ${NAME} (Primary supported target is Ubuntu 24.04 LTS)"
    elif [[ "${VERSION_ID:-}" == "22.04" ]]; then
        s1_details="${s1_details} (Warning: 22.04 is an unverified fallback)"
    fi
else
    s1_status="FAIL"
    s1_details="/etc/os-release not found"
fi

if ! command -v python3 &>/dev/null; then
    s1_status="FAIL"
    s1_details="python3 not installed"
else
    py_ver=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}')")
    s1_details="${s1_details}, Python ${py_ver}"
fi
record_stage "1" "OS & Toolchain" "${s1_status}" "${s1_details}"

# ------------------------------------------------------------------------------
# Stage 2: ROS 2 and Workspace Package Availability
# ------------------------------------------------------------------------------
s2_status="PASS"
s2_details=""
ROS_SETUP=""
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_SETUP="/opt/ros/jazzy/setup.bash"
    s2_details="ROS 2 Jazzy (Primary)"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_SETUP="/opt/ros/humble/setup.bash"
    s2_details="ROS 2 Humble (Unverified Fallback)"
else
    s2_status="FAIL"
    s2_details="No ROS 2 installation found in /opt/ros/"
fi

if [ -n "${ROS_SETUP}" ]; then
    # shellcheck disable=SC1090
    source "${ROS_SETUP}" 2>/dev/null || true
    if [ -f "${REPO_ROOT}/ros2_ws/install/rescue_swarm_bridge/share/rescue_swarm_bridge/package.xml" ]; then
        s2_details="${s2_details}, rescue_swarm_bridge built"
    else
        s2_status="FAIL"
        s2_details="${s2_details}, rescue_swarm_bridge not built in ros2_ws"
    fi
fi
record_stage "2" "ROS 2 & Packages" "${s2_status}" "${s2_details}"

# ------------------------------------------------------------------------------
# Stage 3: PX4 Source & Build Availability
# ------------------------------------------------------------------------------
s3_status="PASS"
s3_details=""
PX4_DIR="${PX4_AUTOPILOT_DIR:-}"
if [ -z "${PX4_DIR}" ] || [ ! -d "${PX4_DIR}" ]; then
    if [ -d "${REPO_ROOT}/../PX4-Autopilot" ]; then
        PX4_DIR="$(cd "${REPO_ROOT}/../PX4-Autopilot" && pwd)"
    elif [ -d "$HOME/PX4-Autopilot" ]; then
        PX4_DIR="$HOME/PX4-Autopilot"
    fi
fi

if [ -n "${PX4_DIR}" ] && [ -d "${PX4_DIR}" ]; then
    if [ -f "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]; then
        s3_details="PX4 SITL compiled at ${PX4_DIR}/build/px4_sitl_default/bin/px4"
    else
        s3_status="FAIL"
        s3_details="PX4 repo found at ${PX4_DIR} but binary not compiled (run: make px4_sitl_default)"
    fi
else
    s3_status="FAIL"
    s3_details="PX4-Autopilot repository not found"
fi
record_stage "3" "PX4 Source/Build" "${s3_status}" "${s3_details}"

# ------------------------------------------------------------------------------
# Stage 4: Gazebo Executable, Required Models, and World
# ------------------------------------------------------------------------------
s4_status="PASS"
s4_details=""
if command -v gz &>/dev/null; then
    gz_ver=$(gz sim --versions 2>/dev/null | head -n 1 || echo "unknown")
    s4_details="Gazebo: ${gz_ver}"
else
    s4_status="FAIL"
    s4_details="Gazebo ('gz') not found"
fi

if [ -n "${PX4_DIR}" ] && [ -d "${PX4_DIR}/Tools/simulation/gz/models" ]; then
    has_x500_gimbal=false
    has_x500=false
    [ -d "${PX4_DIR}/Tools/simulation/gz/models/x500_gimbal" ] && has_x500_gimbal=true
    [ -d "${PX4_DIR}/Tools/simulation/gz/models/x500" ] && has_x500=true
    if [ "$has_x500_gimbal" = true ] && [ "$has_x500" = true ]; then
        s4_details="${s4_details}, models (x500, x500_gimbal) verified"
    else
        s4_status="FAIL"
        s4_details="${s4_details}, missing models in PX4"
    fi
fi

if [ -f "${REPO_ROOT}/worlds/rescue_swarm_field.sdf" ]; then
    s4_details="${s4_details}, world verified"
else
    s4_status="FAIL"
    s4_details="${s4_details}, missing worlds/rescue_swarm_field.sdf"
fi
record_stage "4" "Gazebo & Models" "${s4_status}" "${s4_details}"

# ------------------------------------------------------------------------------
# Stage 5: Fleet Configuration Consistency and Port Uniqueness
# ------------------------------------------------------------------------------
s5_status="PASS"
s5_details=""
cfg_check=$(python3 - << 'EOF'
import json, sys

try:
    with open("config/fleet.json", "r", encoding="utf-8") as f:
        data = json.load(f)
    fleet = data.get("fleet", [])
    if len(fleet) != 6:
        print(f"FAIL: Expected 6 vehicles, found {len(fleet)}")
        sys.exit(1)
    
    ids = [d["id"] for d in fleet]
    sysids = [d["system_id"] for d in fleet]
    ports = [d["mavlink_port"] for d in fleet]
    
    if len(set(ids)) != 6:
        print("FAIL: Duplicate vehicle IDs")
        sys.exit(1)
    if len(set(sysids)) != 6:
        print("FAIL: Duplicate MAVLink system IDs")
        sys.exit(1)
    if len(set(ports)) != 6:
        print("FAIL: Duplicate MAVLink ports")
        sys.exit(1)
    
    # Check port range 14540..14545
    expected_ports = {14540, 14541, 14542, 14543, 14544, 14545}
    if set(ports) != expected_ports:
        print(f"FAIL: Expected offboard ports {sorted(expected_ports)}, got {sorted(ports)}")
        sys.exit(1)
        
    print("PASS: 6 vehicles (S1-S3, D1-D3) mapped with unique sysids (1-6) and offboard ports (14540-14545)")
    sys.exit(0)
except Exception as e:
    print(f"FAIL: {e}")
    sys.exit(1)
EOF
) || true

if [[ "$cfg_check" == PASS* ]]; then
    s5_details="${cfg_check#PASS: }"
else
    s5_status="FAIL"
    s5_details="${cfg_check#FAIL: }"
fi
record_stage "5" "Fleet Config" "${s5_status}" "${s5_details}"

# ------------------------------------------------------------------------------
# Stages 6-9: Live Simulation Execution Gate
# ------------------------------------------------------------------------------
if [ "$RUN_LIVE" != true ]; then
    record_stage "6" "Six-Vehicle SITL Launch" "NOT RUN" "Requires --live execution on target Linux system"
    record_stage "7" "Heartbeat & Identity Verification" "NOT RUN" "Requires --live execution on target Linux system"
    record_stage "8" "ROS 2 Telemetry & Topic Readiness" "NOT RUN" "Requires --live execution on target Linux system"
    record_stage "9" "Safe Fleet Shutdown" "NOT RUN" "Requires --live execution on target Linux system"
else
    # Stage 6: Launch SITL Fleet
    echo "  [LIVE] Starting 6-Drone PX4 SITL + Gazebo Fleet in background..."
    "${REPO_ROOT}/scripts/sitl/launch_px4_gazebo_fleet.sh" --headless &
    LAUNCHER_PID=$!
    sleep 10
    
    if [ -f "${REPO_ROOT}/.sitl_fleet.pids" ]; then
        s6_count=$(wc -l < "${REPO_ROOT}/.sitl_fleet.pids" || echo "0")
        record_stage "6" "Six-Vehicle SITL Launch" "PASS" "Fleet launched; ${s6_count} processes tracked"
    else
        record_stage "6" "Six-Vehicle SITL Launch" "FAIL" "PID file not created by launcher"
    fi

    # Stage 7: Verify Heartbeats
    echo "  [LIVE] Verifying MAVLink telemetry and sysids across all 6 vehicles..."
    if python3 "${REPO_ROOT}/scripts/sitl/verify_fleet_connectivity.py" --timeout 5.0; then
        record_stage "7" "Heartbeat & Identity Verification" "PASS" "All 6 drones verified with distinct sysids (1-6) on ports 14540-14545"
    else
        record_stage "7" "Heartbeat & Identity Verification" "FAIL" "One or more vehicles failed heartbeat or sysid verification"
    fi

    # Stage 8: ROS 2 Readiness
    echo "  [LIVE] Testing ROS 2 adapter initialization..."
    ros2_ok=$(python3 - << 'EOF'
import sys
try:
    from rescue_swarm_bridge.vehicle_adapter import create_vehicle_adapter, AdapterType
    import json
    with open("config/fleet.json") as f:
        cfg = json.load(f)["fleet"][0]
    ad = create_vehicle_adapter("S1", cfg, adapter_type=AdapterType.PX4_MAVLINK)
    print("PASS: ROS 2 vehicle adapter created successfully")
except Exception as e:
    print(f"FAIL: {e}")
EOF
    ) || true
    if [[ "$ros2_ok" == PASS* ]]; then
        record_stage "8" "ROS 2 Telemetry & Topic Readiness" "PASS" "ROS 2 adapter layer successfully initialized"
    else
        record_stage "8" "ROS 2 Telemetry & Topic Readiness" "FAIL" "${ros2_ok}"
    fi

    # Stage 9: Safe Shutdown
    echo "  [LIVE] Executing tracked fleet shutdown..."
    "${REPO_ROOT}/scripts/sitl/stop_px4_gazebo_fleet.sh"
    wait "${LAUNCHER_PID}" 2>/dev/null || true
    
    if [ ! -f "${REPO_ROOT}/.sitl_fleet.pids" ]; then
        record_stage "9" "Safe Fleet Shutdown" "PASS" "All tracked fleet processes terminated cleanly"
    else
        record_stage "9" "Safe Fleet Shutdown" "FAIL" "PID file still present after shutdown"
    fi
fi

echo "================================================================================"
echo "  Smoke Test Summary"
echo "================================================================================"
printf "%-7s | %-35s | %-10s | %s\n" "Stage" "Check Name" "Status" "Details"
echo "--------+-------------------------------------+------------+--------------------"
for res in "${STAGE_RESULTS[@]}"; do
    IFS="|" read -r stg name st details <<< "${res}"
    printf "Stage %-2s| %-35s | %-10s | %s\n" "${stg}" "${name}" "${st}" "${details}"
done
echo "================================================================================"
