# NIDAR RescueSwarm — Full Simulation Project

This repository is the software foundation for a six-drone NIDAR RescueSwarm system.

## Target fleet

- Search drones: `S1`, `S2`, `S3`
- Delivery drones: `D1`, `D2`, `D3`

The long-term target is an integrated PX4 SITL + ROS 2 + Gazebo simulation. This first milestone keeps core mission logic independent of ROS/Gazebo so it can be developed and unit-tested on macOS.

## Architecture

```text
Gazebo cameras / simulated vehicle state
                 |
          perception adapter
       (your actual best.pt model)
                 |
          detection + geotag
                 |
           rescue task queue
                 |
           swarm manager
       /                     \
 search coverage         delivery allocator
       |                     |
  S1 / S2 / S3            D1 / D2 / D3
                 |
          mission manager
                 |
          GCS / telemetry
```

## Repository layout

```text
rescue_swarm_sim/
├── docs/                    # architecture and integration notes
├── config/                  # fleet and mission configuration
├── assets/models/            # keep model weights local; do not commit private weights
├── src/rescue_swarm_core/    # portable mission logic (Python)
└── tests/                   # unit tests
```

## First milestone: run locally
Requires Python 3.10+. Core logic tests run in software without external simulators.

```bash
# 1. Run complete unit test suite
python -m unittest discover -s tests -p "test_*.py"

# 2. Run the end-to-end autonomous multi-drone mission simulation (software-only)
python scripts/run_full_mission_sim.py --scenario nominal

# Run other deterministic scenarios:
python scripts/run_full_mission_sim.py --scenario failed_delivery_retry
python scripts/run_full_mission_sim.py --scenario no_survivors
python scripts/run_full_mission_sim.py --scenario search_drone_comms_loss
python scripts/run_full_mission_sim.py --json

# 3. Run autonomous pipeline demo
python scripts/run_autonomous_pipeline.py
```

### Distinction: Software Mission Simulation vs SITL Physics Simulation

- **Software Mission Simulation (`run_full_mission_sim.py`, `SimulatedMissionRunner`):**
  A discrete-time state-machine simulation executing the complete fleet lifecycle in software. Search sweep coverage, candidate visual observations, 3D ray-plane localization, SwarmManager deduplication, deterministic delivery allocation, safety monitoring (battery, geofence, proximity, comms loss), and delivery controller verification are simulated as software state transitions. No vehicle physics, aerodynamics, motor thrust, or physical kit drops are computed.

- **SITL Physics Simulation (ArduPilot / PX4 + Gazebo):**
  Full multi-vehicle physics simulation with synthetic MAVLink/DDS bridges, GPS lock, flight dynamics, wind/aerodynamics, and Gazebo contact/sensor topics. Software logic tests do NOT claim flight readiness or replace SITL physics testing.

### Safety Alert vs Incident Lifecycle Semantics
- **SafetyAlert (Tick-Level Notification):** Generated every simulation cycle (e.g., 1 Hz) where a safety condition violates configured boundaries (geofence, proximity, battery threshold, telemetry heartbeat timeout). Flight controllers consume these in real-time to execute immediate override actions (e.g. `RETURN_TO_LAUNCH`, `HOLD_POSITION`).
- **SafetyIncident (Deduplicated Lifecycle Entity):** Manages persistent safety violations across time. A single ongoing violation produces exactly one `SafetyIncident` tracking `start_time`, `last_seen_time`, `occurrence_count`, and severity escalations (e.g. `WARNING` -> `CRITICAL` -> `EMERGENCY`). When telemetry returns to nominal, the incident is closed (`resolved=True`). Clearing and re-triggering opens a distinct incident. This prevents unbounded duplicate alert history logs while maintaining continuous real-time controller action.

### Search Observation Modes: Scripted Timestamps vs Coverage-Dependent Footprints
- **Scripted Timestamps (Default):** Observations are injected at configured simulation timestamps for deterministic scenario playback.
- **Coverage-Dependent Footprint (`require_coverage_footprint=True`):** Software validation mode that computes the observing drone's ground sensor footprint (swath width derived from altitude and camera horizontal FOV) and verifies that candidate survivor coordinates lie within the sensor footprint and the drone's assigned coverage sector. Detections outside the sector or beyond camera swath range are rejected and recorded in `coverage_rejections`.

## Important development rules

1. Do not commit `best.pt`, secrets, private competition documents, or large binaries to a public repository.
2. Keep core mission logic independent of ROS 2 messages; adapters will translate between ROS messages and these core types.
3. Never send real flight commands based on this logic without simulation, safety review, and appropriate authorization.
4. A software-only logic test is not a flight-physics test. Gazebo/PX4 integration remains a separate milestone.
5. Configuration values are initial software defaults, not final aircraft/hardware specifications.

## Current status

- [x] Initial architecture and six-drone fleet definition (`config/fleet.json`, `config/mission_scenarios.json`)
- [x] Portable rescue task and fleet data types (`models.py`)
- [x] Deterministic delivery-drone allocator (`allocator.py`)
- [x] Mission lifecycle state machine (`mission.py`)
- [x] Perception interface and fail-fast `YOLOPerceptionAdapter` (`perception.py`)
- [x] 3D ray-casting survivor geotagging & camera geometry (`localization.py`)
- [x] Swarm manager with spatial deduplication & task lifecycle (`swarm_manager.py`)
- [x] Search coverage planner & boustrophedon sweep generator (`coverage_planner.py`)
- [x] Real-time safety manager with incident lifecycle & deduplication (`safety_manager.py`)
- [x] Delivery mission flight controller & autonomous drop verification (`delivery.py`)
- [x] Integrated 6-drone mission simulation runner & deterministic scenarios (`mission_sim.py`, `run_full_mission_sim.py`)
- [x] Comprehensive test suite (206 unit tests passing cleanly across all suites)
- [x] Standard packaging configuration (`pyproject.toml`)
- [x] Real ROS 2 integration package (`ros2_ws/src/rescue_swarm_bridge/`)
- [x] Isolated multi-vehicle adapters (`MockVehicleAdapter`, `PX4ROS2VehicleAdapter`, `PX4MAVLinkVehicleAdapter`)
- [x] PX4 SITL + Gazebo Harmonic launch orchestration (`scripts/sitl/launch_px4_gazebo_fleet.sh`)
- [x] Multi-vehicle connectivity & anti-cross-talk verification (`scripts/sitl/verify_fleet_connectivity.py`)
- [x] 9-Stage Target Machine Smoke Test Suite (`scripts/sitl/smoke_test_fleet.sh`)
- [x] Automated Linux simulation setup script (`scripts/setup_linux_sim.sh`)
- [x] Unified system environment preflight checker (`scripts/preflight_check.py`)

---

## Quick-Start Workflows

### Workflow A: Development Computer (Windows / macOS / Linux)
*No ROS 2, PX4, or Gazebo required. All 206 tests and simulation scenarios run in pure Python.*

```bash
# 1. Install dependencies (virtual environment recommended)
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\Activate.ps1
pip install -r requirements.txt
pip install -e .

# 2. Run automated environment preflight validator
python scripts/preflight_check.py

# 3. Run complete unit and regression test suite (206 tests)
python -m unittest discover -s tests -p "test_*.py"

# 4. Run discrete-time autonomous mission simulation
python scripts/run_full_mission_sim.py --scenario nominal

# 5. Run scenario repeatability verification
python scripts/verify_repeatability.py
```

---

### Workflow B: Designated Linux Simulation Environment (Senior Linux Machine)
*Target: Ubuntu 24.04 LTS (Noble) with ROS 2 Jazzy and Gazebo Harmonic (`gz-sim 8.15.0+`)*

```bash
# 1. Clone repository from GitHub
git clone https://github.com/<org>/rescue_swarm_sim.git
cd rescue_swarm_sim

# 2. Run Linux setup script
# (Configures .venv with --system-site-packages, installs pip dependencies,
# checks system dependencies, and builds ros2_ws if Jazzy is installed.
# If system packages are missing, it provides explicit sudo apt commands)
chmod +x scripts/*.sh scripts/sitl/*.sh
./scripts/setup_linux_sim.sh

# 3. Source environment and verify preflight
source .venv/bin/activate
python3 scripts/preflight_check.py

# 4. Run static smoke checks (Stages 1-5: OS, ROS 2, PX4, Gazebo, Fleet Config)
./scripts/sitl/smoke_test_fleet.sh

# 5. One-time build of PX4 SITL (if not already compiled)
cd ${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}
DONT_RUN=1 make px4_sitl_default
cd -

# 6. Option A: Run the end-to-end 9-Stage automated live smoke test:
./scripts/sitl/smoke_test_fleet.sh --live

# Option B: Manual multi-terminal execution:
# Terminal 1: Launch the 6-Drone PX4 SITL + Gazebo Harmonic Fleet
./scripts/sitl/launch_px4_gazebo_fleet.sh --headless

# Terminal 2: Verify real MAVLink connectivity across all 6 drones (offboard ports 14540-14545):
python3 scripts/sitl/verify_fleet_connectivity.py

# Terminal 3: Launch ROS 2 Mission Coordinator & Fleet Autonomy:
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch rescue_swarm_bridge six_drone_mission.launch.py adapter_type:=px4_mavlink

# Terminal 4: Cleanly terminate all simulation processes and PX4 instances:
./scripts/sitl/stop_px4_gazebo_fleet.sh
```

---

## Documentation Index

- [Portability Audit Findings](docs/PORTABILITY_AUDIT.md) — Inventory of modules, mocks, and resolved issues.
- [Supported Simulation Environment](docs/SIMULATION_ENVIRONMENT.md) — Pinned OS, ROS 2, Gazebo Harmonic, and PX4 stack details.
- [GitHub Deployment & Clean-Clone Checklist](docs/GITHUB_DEPLOYMENT_CHECKLIST.md) — Pre-push hygiene and deployment checklist.
- [Readiness Report](docs/PX4_GAZEBO_READINESS_REPORT.md) — Verification results, test metrics, and readiness classification.
