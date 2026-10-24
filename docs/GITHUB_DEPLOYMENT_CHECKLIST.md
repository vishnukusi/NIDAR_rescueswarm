# NIDAR RescueSwarm — GitHub Deployment & Clean-Clone Verification Checklist

This checklist must be followed when pushing the software to GitHub and when deploying on our seniors' target Linux simulation system.

---

## Part 1: Pre-Push Hygiene Audit (Developer Machine)

Before creating a GitHub commit or pushing code, verify every item below:

- [x] **Private Model Weights:** `assets/models/best.pt` is ignored by `.gitignore` (`*.pt`) and is NOT staged in Git.
- [x] **Large Media Files:** Raw video recordings (`*.mp4`, `*.avi`) are ignored and kept local.
- [x] **No Machine-Specific Paths:** All scripts and source files use repository-relative paths, `$REPO_ROOT`, or environment variables (`$PX4_AUTOPILOT_DIR`). No hardcoded `/home/<username>` paths exist.
- [x] **No Secrets or Tokens:** No API keys, credentials, or personal tokens are stored in configuration or source code.
- [x] **Build & Cache Artifacts Ignored:** Virtual environments (`.venv/`), Python byte-code (`__pycache__/`, `*.pyc`), test caches (`.pytest_cache/`), and ROS 2 build products (`ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`) are ignored.
- [x] **Self-Contained Software Tests:** All 206 unit and regression tests pass without requiring private model weights or external simulator hardware.

---

## Part 2: Clean-Clone Deployment Checklist (Senior Linux System)

When our seniors clone the repository onto their designated Ubuntu 24.04 LTS system:

### Phase 1: Repository Checkout & Environment Setup
```bash
# 1. Clone repository from GitHub
git clone https://github.com/<org>/rescue_swarm_sim.git
cd rescue_swarm_sim

# 2. Run automated Linux simulation setup script
# (Configures virtual environment with --system-site-packages, installs Python dependencies,
# checks Gazebo and ROS 2 status, and builds ros2_ws if prerequisites are met)
chmod +x scripts/*.sh scripts/sitl/*.sh
./scripts/setup_linux_sim.sh
```

### Phase 2: Offline Software Verification (No Hardware / No SITL Needed)
```bash
# 3. Activate the virtual environment
source .venv/bin/activate

# 4. Run the comprehensive environment preflight check
python scripts/preflight_check.py

# 5. Run the complete 206-test autonomous test suite
python -m unittest discover -s tests -p "test_*.py"

# 6. Verify deterministic mission scenarios and JSON report repeatability
python scripts/verify_repeatability.py

# 7. Run static preflight smoke checks (Stages 1-5)
./scripts/sitl/smoke_test_fleet.sh
```
*Expected Result:* 206/206 tests pass, 7/7 repeatable scenarios pass, Stages 1-5 pass.

### Phase 3: Perception Model Placement (Optional for Genuine Camera Detections)
```bash
# 8. (Optional) If running genuine visual detection rather than synthetic fixtures,
# place the authorized trained YOLOv8 model at:
# assets/models/best.pt
```

### Phase 4: ROS 2 Package Build
```bash
# 9. Source ROS 2 Jazzy (or Humble)
source /opt/ros/jazzy/setup.bash

# 10. Build the rescue_swarm_bridge package
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

### Phase 5: PX4 SITL & Gazebo Harmonic Multi-Vehicle Simulation
```bash
# 11. Ensure PX4 Autopilot SITL is compiled (one-time build)
# (If PX4 is cloned at $HOME/PX4-Autopilot or ../PX4-Autopilot)
cd ${PX4_AUTOPILOT_DIR:-$HOME/PX4-Autopilot}
DONT_RUN=1 make px4_sitl_default
cd -

# Option A: Run the end-to-end 9-Stage automated live smoke test:
./scripts/sitl/smoke_test_fleet.sh --live

# Option B: Manual multi-terminal execution:
# Terminal 1: Launch the 6-Drone PX4 SITL + Gazebo Harmonic Fleet
./scripts/sitl/launch_px4_gazebo_fleet.sh --headless

# Terminal 2: Verify real MAVLink connectivity across all 6 drones (offboard ports 14540-14545):
python3 scripts/sitl/verify_fleet_connectivity.py

# Terminal 3: Launch ROS 2 Autonomy Mission Manager & Swarm Coordinator:
ros2 launch rescue_swarm_bridge six_drone_mission.launch.py adapter_type:=px4_mavlink

# Terminal 4: Launch the local GCS web dashboard (optional, view in browser at http://localhost:8080):
python3 scripts/run_gcs_server.py
```

### Phase 6: Clean Shutdown
```bash
# 12. Cleanly terminate all tracked simulation processes (reads .sitl_fleet.pids):
./scripts/sitl/stop_px4_gazebo_fleet.sh
```

---

## Part 3: Troubleshooting Guide

| Issue | Root Cause | Solution |
|---|---|---|
| `gz: command not found` | Gazebo Harmonic not installed | Run: `sudo apt install -y gz-harmonic` |
| `colcon: command not found` | ROS 2 build tools missing | Run: `sudo apt install -y python3-colcon-common-extensions` |
| `PX4-Autopilot not found` | Autopilot repo not cloned | Run: `git clone --recursive https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot` and `export PX4_AUTOPILOT_DIR=$HOME/PX4-Autopilot` |
| `UDP port in use (14540-14545 / 14550-14555)` | Lingering SITL instance | Run: `./scripts/sitl/stop_px4_gazebo_fleet.sh` to kill residual processes |
| `best.pt not found (WARNING)` | Model weights private | Unit tests will automatically use test fixtures; place authorized weights in `assets/models/best.pt` for live YOLO inference |
