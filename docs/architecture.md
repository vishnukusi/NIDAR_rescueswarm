# System architecture

## Responsibility boundaries

- **Mission manager:** mission lifecycle and overall progress (`rescue_swarm_core.mission`).
- **Swarm manager:** connects search and delivery fleets, maintains survivor records, prevents duplicate tasks via spatial deduplication (`rescue_swarm_core.swarm_manager`).
- **Perception adapter:** runs the team's real `best.pt` model on camera frames with structured outputs and fail-fast validation (`rescue_swarm_core.perception`).
- **Approach & hover controller:** visual servoing translation controller adapted from the embedded drone project (`docs/Drone_project`), providing proportional steering, upper-body head point tracking, exponential moving average smoothing, deadband centering, and waypoint pause/resume coordination for S1, S2, and S3 (`rescue_swarm_core.approach_controller`).
- **Geotagging & localization:** combines bounding box detections with vehicle pose, altitude AGL, camera orientation (mount pitch, roll, and yaw angles), and 3D ray-plane intersection (`rescue_swarm_core.localization`).
- **Rescue-task queue:** records each candidate survivor and coordinates delivery status (`rescue_swarm_core.models.RescueTask`).
- **Delivery allocator:** selects the eligible delivery drone with the highest battery (`rescue_swarm_core.allocator`).
- **Search coverage planner:** partitions operational area and generates boustrophedon sweep paths for S1/S2/S3 (`rescue_swarm_core.coverage_planner`).
- **Safety manager:** monitors battery failsafes, altitude limits, lateral geofence, and fleet anti-collision separation (`rescue_swarm_core.safety_manager`).
- **Delivery controller:** coordinates flight profiles, descent to 8m drop altitude, payload release, and verification with distinct semantics for simulated demonstration timers vs. physical sensor/simulation impact events (`rescue_swarm_core.delivery`).
- **Ground Control Station (GCS):** real-time tactical radar visualizer, telemetry cards, and simulation controls (`gcs/index.html`, `gcs/app.js`).
- **Vehicle adapter (future):** connects high-level requests to ROS 2 / PX4 interfaces.

## Current task lifecycle

`DETECTED -> APPROACHING/HOVERING -> GEOLOCATED -> ASSIGNED -> EN_ROUTE -> RELEASED -> VERIFIED`

1. Search drone observes person candidate with `best.pt`.
2. Coverage lawnmower waypoints are suspended while `SearchDroneApproachController` commands visual servoing translation.
3. Upon deadband centering, search drone enters `HOVERING`, holds position directly overhead, and captures refined camera pose.
4. Target is geolocated via 3D ray-plane intersection and registered with `SwarmManager`.
5. Swarm manager checks spatial deduplication and assigns highest-battery delivery drone (D1, D2, or D3).
6. Search drone completes inspection and resumes search coverage waypoints.
7. Delivery drone transits to target, descends to 8m drop altitude, triggers payload release, confirms delivery (via physical sensor, simulation contact topic, or demo timer), and initiates Return-To-Launch (RTL).

## Local Environment Setup (.venv)

A dedicated Python virtual environment is established at repository root:

```bash
# 1. Activate dedicated virtual environment
source .venv/bin/activate

# 2. Run unit tests
python -m unittest discover -s tests -v

# 3. Run autonomous pipeline with genuine model inference
PYTHONPATH=src python scripts/run_autonomous_pipeline.py

# 4. Run full fleet mission simulation
PYTHONPATH=src python scripts/run_full_mission_sim.py

# 5. Launch GCS Dashboard
python scripts/run_gcs_server.py
```

