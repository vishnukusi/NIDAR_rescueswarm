# NIDAR RescueSwarm — Phase A: Repository Inventory and Baseline Report

## Date: 2026-10-24
## Role: Autonomous Senior Robotics Software & Integration Engineer

---

## 1. Environment & Host Identification

| Property | Host System (Windows) | Target Linux / WSL Subsystem |
| :--- | :--- | :--- |
| **Operating System** | Windows 11 Home / Pro (x86_64) | Ubuntu 24.04 LTS (Noble Numbat) via WSL 2 (`6.18.54.1-microsoft-standard-WSL2`) |
| **Python Version** | Python 3.10.0 | Python 3.12.3 |
| **Git Version** | 2.38.1.windows.1 | Available |
| **Gazebo Simulator** | Not installed natively on Windows | **Gazebo Sim 8.15.0 (Harmonic)** installed (`/usr/bin/gz`) |
| **PX4 Autopilot** | Not installed natively on Windows | Cloned repository at `/home/vishnu/PX4-Autopilot` (commit `7c8d6daca9`) |
| **ROS 2 Distribution** | Not installed natively on Windows | Not yet installed in `/opt/ros` (Targeting ROS 2 Jazzy Jalisco for Ubuntu 24.04 or Humble in Ubuntu 22.04 container) |
| **Colcon Build Tool** | Not installed natively on Windows | Installable via `python3-colcon-common-extensions` |

---

## 2. Repository Inventory

Repository Root: `C:\Users\chaya\Downloads\rescue_swarm_sim 3\rescue_swarm_sim 3`

### Directory Layout
- **`src/rescue_swarm_core/`**: Pure Python core mission logic, independent of ROS 2 / Gazebo:
  - `models.py`: Data models (`Drone`, `RescueTask`, `GeoPoint`, `DroneRole`, `DroneStatus`, `TaskStatus`, `MissionState`).
  - `allocator.py`: Deterministic delivery allocation policy with battery eligibility (`minimum_battery_fraction=0.30`).
  - `mission.py`: Swarm mission state machine.
  - `perception.py`: Bounding boxes, detections, `YOLOPerceptionAdapter` using Ultralytics YOLOv8.
  - `localization.py`: 3D ray-plane intersection, camera calibration (`CameraCalibration`), nadir geometry, Haversine ground distances.
  - `association.py`: Spatial survivor deduplication (`SurvivorAssociator`), association decisions, scoped tracklet mapping.
  - `coverage_planner.py`: Lawnmower/boustrophedon sweep generator (`SearchArea`, `SearchSector`, `plan_coverage`).
  - `safety_manager.py`: Watchdog monitoring, geofence, battery failsafe, proximity detection, and `SafetyIncident` lifecycle management.
  - `delivery.py`: `DeliveryMissionController` state machine (`TAKEOFF`, `EN_ROUTE`, `APPROACH`, `DESCENT_TO_DROP`, `PAYLOAD_RELEASE`, `VERIFY_DROP`, `ASCENT`, `RETURNING_HOME`, `COMPLETED`).
  - `flight_bridge.py`: `MAVLinkFlightBridge` for vehicle-isolated pymavlink communication with ArduCopter/PX4 (OFFBOARD mode, local NED setpoints).
  - `fleet_flight_bridge.py`: `FleetFlightBridgeManager` coordinating isolated MAVLink ports for 6 drones (S1–S3: 14550–14552, D1–D3: 14553–14555).
  - `approach_controller.py`: Visual servoing approach and hover controller for search drones.
  - `mavlink_emulator.py`: In-memory loopback MAVLink emulator for standalone tests without physical flight bridges.
  - `mission_sim.py`: Discrete-time six-drone simulation runner (`SimulatedMissionRunner`, `MissionScenarioConfig`, `MissionSimulationReport`).

- **`config/`**:
  - `fleet.json`: 6-drone fleet definition (S1–S3 search, D1–D3 delivery) and mission constraints.
  - `mission_scenarios.json`: Scenario parameters.

- **`scripts/`**:
  - `run_full_mission_sim.py`: Standalone CLI runner for discrete-time software simulations.
  - `verify_repeatability.py`: Deterministic multi-run comparison utility across all scenarios.
  - `run_autonomous_pipeline.py`: End-to-end perception -> geotag -> allocation -> delivery pipeline demo.
  - `run_gcs_server.py`: Local Ground Control Station web dashboard.

- **`tests/`**:
  - 18 test modules validating models, allocator, perception, association, localization, coverage, safety lifecycle, delivery controllers, and mission simulation.

- **`assets/`**:
  - `models/best.pt`: Local YOLOv8 weights (kept local, uncommitted in `.gitignore`).

---

## 3. Baseline Test Suite Execution

- **Command**: `python -m unittest discover -s tests -p "test_*.py"`
- **Result**: `Ran 189 tests in 27.291s`
- **Passed**: 189
- **Failures**: 0
- **Errors**: 0
- **Skips**: 0

**Phase A Acceptance Criteria: MET.**
