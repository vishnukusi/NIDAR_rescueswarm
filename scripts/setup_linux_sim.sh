#!/usr/bin/env bash
# ==============================================================================
# NIDAR RescueSwarm — Linux Simulation Setup & Build Automation
# ==============================================================================
# Prepares a clean Ubuntu Linux system for running the 6-Drone PX4 SITL + Gazebo
# simulation and building the ROS 2 autonomy bridge package.
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "================================================================================"
echo "  NIDAR RescueSwarm — Linux Simulation Environment Setup"
echo "================================================================================"
echo "  Repository Root: ${REPO_ROOT}"

# 1. Check OS Distribution
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo "  Operating System: ${NAME} ${VERSION_ID} (${UBUNTU_CODENAME:-unknown})"
    if [[ "${ID:-}" != "ubuntu" ]]; then
        echo "  [WARNING] Target distribution is Ubuntu 24.04 (Noble) or 22.04 (Jammy). Detected: ${NAME}"
    fi
else
    echo "  [WARNING] /etc/os-release not found. Assuming Linux environment."
fi

# 2. Check Python 3 Version
if ! command -v python3 &>/dev/null; then
    echo "[ERROR] python3 command not found. Please install Python 3.10 or higher."
    echo "  Fix: sudo apt update && sudo apt install -y python3 python3-pip python3-venv"
    exit 1
fi

PY_VER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "  Python Version: ${PY_VER}"
PY_MAJ=$(echo "${PY_VER}" | cut -d. -f1)
PY_MIN=$(echo "${PY_VER}" | cut -d. -f2)

if [ "${PY_MAJ}" -lt 3 ] || ([ "${PY_MAJ}" -eq 3 ] && [ "${PY_MIN}" -lt 10 ]); then
    echo "[ERROR] Python version >= 3.10 is required. Found: ${PY_VER}"
    exit 1
fi

# 3. Setup Python Virtual Environment & Dependencies
VENV_DIR="${REPO_ROOT}/.venv"
NEEDS_RECREATE=false
if [ -d "${VENV_DIR}" ]; then
    if [ -f "${VENV_DIR}/pyvenv.cfg" ]; then
        if ! grep -qi "include-system-site-packages = true" "${VENV_DIR}/pyvenv.cfg"; then
            echo "  Existing virtual environment does not include system-site-packages. Updating..."
            sed -i 's/include-system-site-packages = false/include-system-site-packages = true/gI' "${VENV_DIR}/pyvenv.cfg" || NEEDS_RECREATE=true
        fi
    fi
fi

if [ ! -d "${VENV_DIR}" ] || [ "${NEEDS_RECREATE}" = true ]; then
    echo "  Creating Python virtual environment with --system-site-packages at ${VENV_DIR}..."
    rm -rf "${VENV_DIR}"
    python3 -m venv --system-site-packages "${VENV_DIR}"
fi

echo "  Activating virtual environment..."
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

echo "  Installing/Verifying Python requirements from requirements.txt..."
pip install --upgrade pip
pip install -r "${REPO_ROOT}/requirements.txt"
echo "  [NOTE] 'pip install -e .' installs the rescue-swarm-sim Python core package."
echo "  [NOTE] ROS 2 (rclpy, sensor_msgs) is provided by system packages and accessed via --system-site-packages."
pip install -e "${REPO_ROOT}"

# 4. Check Gazebo Harmonic
echo "--------------------------------------------------------------------------------"
echo "  Verifying Gazebo Simulator Installation..."
MISSING_PREREQS=0
if command -v gz &>/dev/null; then
    GZ_VER=$(gz sim --versions 2>/dev/null | head -n 1 || echo "unknown")
    echo "  [PASS] Gazebo Sim found: ${GZ_VER}"
else
    MISSING_PREREQS=$((MISSING_PREREQS + 1))
    echo "  [ACTION REQUIRED] Gazebo Sim ('gz') is not installed."
    echo "  To install Gazebo Harmonic on Ubuntu 24.04 LTS (Noble):"
    echo "    sudo apt-get update"
    echo "    sudo apt-get install -y lsb-release wget gnupg"
    echo "    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"
    echo "    echo 'deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable \$(lsb_release -cs) main' | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null"
    echo "    sudo apt-get update && sudo apt-get install -y gz-harmonic"
fi

# 5. Check ROS 2 Distribution & Colcon Build
echo "--------------------------------------------------------------------------------"
echo "  Verifying ROS 2 Installation..."
ROS_SETUP=""
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_SETUP="/opt/ros/jazzy/setup.bash"
    echo "  [PASS] Found ROS 2 Jazzy at ${ROS_SETUP}"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_SETUP="/opt/ros/humble/setup.bash"
    echo "  [PASS] Found ROS 2 Humble at ${ROS_SETUP} (Notice: Ubuntu 22.04 / Humble is an unverified fallback)"
fi

if [ -n "${ROS_SETUP}" ]; then
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"

    # Verify rclpy import inside virtualenv
    if python3 -c "import rclpy" 2>/dev/null; then
        echo "  [PASS] ROS 2 Python bindings (rclpy) successfully imported inside virtualenv."
    else
        echo "  [WARNING] Unable to import 'rclpy' inside virtual environment."
        echo "  Ensure ROS 2 python packages are installed on host system."
    fi

    if command -v colcon &>/dev/null; then
        echo "  Building rescue_swarm_bridge ROS 2 package..."
        (
            cd "${REPO_ROOT}/ros2_ws"
            colcon build --symlink-install
        )
        echo "  [PASS] ROS 2 workspace build completed successfully."
    else
        MISSING_PREREQS=$((MISSING_PREREQS + 1))
        echo "  [ACTION REQUIRED] colcon build tool is not installed."
        echo "  Fix: sudo apt install -y python3-colcon-common-extensions"
    fi
else
    MISSING_PREREQS=$((MISSING_PREREQS + 1))
    echo "  [ACTION REQUIRED] ROS 2 is not sourced or installed in /opt/ros."
    echo "  Primary supported distribution: ROS 2 Jazzy on Ubuntu 24.04 LTS:"
    echo "    sudo apt update && sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions"
fi

# 6. Check PX4-Autopilot Repository
echo "--------------------------------------------------------------------------------"
echo "  Verifying PX4-Autopilot Source Repository..."
PX4_DIR="${PX4_AUTOPILOT_DIR:-}"
if [ -z "${PX4_DIR}" ] || [ ! -d "${PX4_DIR}" ]; then
    if [ -d "${REPO_ROOT}/../PX4-Autopilot" ]; then
        PX4_DIR="$(cd "${REPO_ROOT}/../PX4-Autopilot" && pwd)"
    elif [ -d "$HOME/PX4-Autopilot" ]; then
        PX4_DIR="$HOME/PX4-Autopilot"
    fi
fi

if [ -n "${PX4_DIR}" ] && [ -d "${PX4_DIR}" ]; then
    echo "  [PASS] Found PX4-Autopilot at: ${PX4_DIR}"
    if [ ! -f "${PX4_DIR}/build/px4_sitl_default/bin/px4" ]; then
        echo "  [NOTE] PX4 SITL default target has not been compiled yet."
        echo "  To compile PX4 SITL (one-time build):"
        echo "    cd ${PX4_DIR} && DONT_RUN=1 make px4_sitl_default"
    else
        echo "  [PASS] PX4 SITL binary compiled at ${PX4_DIR}/build/px4_sitl_default/bin/px4"
    fi
else
    MISSING_PREREQS=$((MISSING_PREREQS + 1))
    echo "  [ACTION REQUIRED] PX4-Autopilot repository not found."
    echo "  To clone PX4 Autopilot:"
    echo "    git clone --recursive https://github.com/PX4/PX4-Autopilot.git ${HOME}/PX4-Autopilot"
    echo "    export PX4_AUTOPILOT_DIR=${HOME}/PX4-Autopilot"
fi

echo "================================================================================"
if [ "${MISSING_PREREQS}" -gt 0 ]; then
    echo "  [PARTIAL SETUP] Python virtual environment configured successfully."
    echo "  [ACTION REQUIRED] ${MISSING_PREREQS} system prerequisite(s) require manual installation."
    echo "  Install the missing items shown above, then re-run ./scripts/setup_linux_sim.sh"
    echo "  to complete the ROS 2 workspace compilation."
    echo "================================================================================"
    exit 1
else
    echo "  [PASS] All simulation prerequisites verified and ROS 2 workspace built!"
    echo "  To run the full preflight audit: python3 scripts/preflight_check.py"
    echo "================================================================================"
    exit 0
fi
