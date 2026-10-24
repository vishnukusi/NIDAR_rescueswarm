# NIDAR RescueSwarm — Supported Simulation Environment Specification

This document defines the **single officially supported baseline environment** for running the full six-drone PX4 SITL + Gazebo Harmonic + ROS 2 simulation.

---

## 1. Supported Environment Matrix

| Component | Pinned Version / Release | Role in RescueSwarm | Rationales & Source Verification |
|---|---|---|---|
| **Host Operating System** | **Ubuntu 24.04 LTS (Noble Numbat)** *(Single Primary Supported)* | Operating system hosting PX4 SITL, Gazebo Harmonic, and ROS 2 Jazzy | Ubuntu 24.04 is the single official target LTS release with native simultaneous packaging for both ROS 2 Jazzy and Gazebo Harmonic (`gz-sim 8.15.0+`). *(Note: Ubuntu 22.04 LTS with ROS 2 Humble is an unverified fallback because Humble defaults to Gazebo Fortress, requiring non-standard source builds of `ros_gz` to connect to PX4 v1.15's Gazebo Harmonic airframes).* |
| **ROS 2 Distribution** | **ROS 2 Jazzy Jalisco** | Multi-agent coordination, topic communication, perception bridging | Jazzy Jalisco is the Tier 1 standard LTS for Ubuntu 24.04. Utilizes `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, and `nav_msgs`. |
| **Flight Autopilot Stack** | **PX4 Autopilot v1.15.x** *(tested commit: `7c8d6daca9`)* | Autonomous flight controller SITL instances (S1–S3, D1–D3) | PX4 v1.15 features native Gazebo Harmonic integration (`gz_x500`), removing deprecated Gazebo Classic plugins. Verified airframes 4019 and 4001. |
| **Physics Simulator** | **Gazebo Harmonic (`gz-sim 8.15.0+`)** | Multi-vehicle aerodynamics, ground world geometry, camera sensor simulation | Standard simulator packaged in Ubuntu 24.04 (`gz-harmonic`). Verified binary at `/usr/bin/gz`. |
| **Python Runtime** | **Python 3.12.3** *(Host dev: Python 3.10+)* | Autonomy core, YOLO perception, SwarmManager, vehicle adapters | Compatible with `rclpy`, PyTorch 2.0+, Ultralytics, and `pymavlink`. |
| **Bridge Transport** | **Dual-Tier: MAVLink UDP + micro-XRCE-DDS** | Vehicle command and telemetry links | Dedicated UDP ports (Offboard: 14540–14545, GCS: 14550–14555) guarantee vehicle isolation; micro-XRCE agent provides ROS 2 uORB bridging. |

---

## 2. Six-Drone Identity & Transport Topology

Each simulated vehicle is assigned an independent instance ID, system ID, dedicated communication port, ROS 2 namespace, and Gazebo model:

```text
               +-------------------------------------------------+
               |       ROS 2 Mission Manager & Swarm Node        |
               +-------------------------------------------------+
                 |        |        |        |        |        |
             [/s1]|   [/s2]|   [/s3]|   [/d1]|   [/d2]|   [/d3]|
                 v        v        v        v        v        v
               +----+   +----+   +----+   +----+   +----+   +----+
               | S1 |   | S2 |   | S3 |   | D1 |   | D2 |   | D3 |  (Vehicle Adapters)
               +----+   +----+   +----+   +----+   +----+   +----+
                 |        |        |        |        |        |
        Offb:  14540    14541    14542    14543    14544    14545  (Offboard Telemetry / Setpoints)
        GCS:   14550    14551    14552    14553    14554    14555  (GCS / UI Monitoring Link)
        SysID:   1        2        3        4        5        6
        Inst:    0        1        2        3        4        5
                 |        |        |        |        |        |
                 v        v        v        v        v        v
               +-------------------------------------------------+
               |       6 x PX4 SITL Autopilot Instances          |
               +-------------------------------------------------+
                 |        |        |        |        |        |
                 v        v        v        v        v        v
               +-------------------------------------------------+
               |          Gazebo Harmonic Simulator              |
               |  (S1-S3: x500_gimbal | D1-D3: x500 quadrotor)   |
               |  World: worlds/rescue_swarm_field.sdf           |
               +-------------------------------------------------+
```

### Port and Identity Invariant Table

| Drone | Fleet Role | Instance ID | System ID | Airframe Autostart | Offboard Port | GCS Port | ROS 2 Namespace | Gazebo Model | Sensor Payload |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---|
| **S1** | Search | `0` | `1` | `4019` (`gz_x500_gimbal`) | `14540` | `14550` | `/s1` | `x500_gimbal` | 1920x1080 Nadir Camera (84° HFOV) |
| **S2** | Search | `1` | `2` | `4019` (`gz_x500_gimbal`) | `14541` | `14551` | `/s2` | `x500_gimbal` | 1920x1080 Nadir Camera (84° HFOV) |
| **S3** | Search | `2` | `3` | `4019` (`gz_x500_gimbal`) | `14542` | `14552` | `/s3` | `x500_gimbal` | 1920x1080 Nadir Camera (84° HFOV) |
| **D1** | Delivery | `3` | `4` | `4001` (`gz_x500`) | `14543` | `14553` | `/d1` | `x500` | Simulated Medical Drop Kit (0.2 kg) |
| **D2** | Delivery | `4` | `5` | `4001` (`gz_x500`) | `14544` | `14554` | `/d2` | `x500` | Simulated Medical Drop Kit (0.2 kg) |
| **D3** | Delivery | `5` | `6` | `4001` (`gz_x500`) | `14545` | `14555` | `/d3` | `x500` | Simulated Medical Drop Kit (0.2 kg) |

---

## 3. Package Dependencies

### 3.1 Ubuntu System Packages (via `apt`)
```bash
sudo apt update && sudo apt install -y \
    build-essential \
    cmake \
    git \
    ninja-build \
    python3-dev \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    ros-jazzy-desktop \
    gz-harmonic
```

### 3.2 Python Virtual Environment Packages (via `pip`)
From `requirements.txt`:
- `torch>=2.0.0`
- `torchvision>=0.15.0`
- `ultralytics>=8.0.0` (ByteTrack & YOLOv8 inference)
- `pymavlink>=2.4.40` (MAVLink protocol encoding/decoding)
- `numpy>=1.24.0`
- `pillow>=9.5.0`
- `opencv-python>=4.8.0`
- `pyyaml>=6.0`
- `requests>=2.30.0`

---

## 4. Hardware & Simulation Boundaries

1. **Camera Sensor:**
   - In Gazebo simulation, search drones S1–S3 use the `x500_gimbal` model featuring a simulated RGB camera publishing ROS 2 image messages (`sensor_msgs/Image`).
   - The perception adapter runs genuine YOLO inference on these simulated camera frames.
2. **Delivery Actuation:**
   - Physical payload deployment mechanisms are **not implemented** in hardware or simulated physics plugins.
   - Delivery outcomes are governed by the `SimulatedDeliveryController` state machine and explicitly logged as `[SIMULATION_ONLY]`.
3. **Safety Enforcement:**
   - Autopilot mode overrides (e.g. `HOLD` or `RTL`) issue real MAVLink/ROS 2 flight commands to the simulated autopilot and verify observed telemetry mode feedback before confirming state resolution.
4. **Payload Mass vs Vehicle Dynamics:**
   - The mission requirement specifies a 200 g (0.2 kg) emergency medical survivor kit, which is configured in `config/fleet.json` and tracked in software delivery allocations (maximum capacity 500 g).
   - In Gazebo simulation, the quadrotor all-up vehicle mass is defined by the default `x500` physics model (~2.0 kg dry mass including airframe, avionics, and battery).
   - Payload release does not physically detach an entity in the Gazebo SDF physics tree; the drop and verification are evaluated via software state machine criteria (arrival radius, hover descent duration, and dispersion bounds).

---

## 5. Verification Check on Target Linux System

When the repository is cloned on the target Linux system, run:
```bash
# 1. Verify environment prerequisites
python3 scripts/preflight_check.py

# 2. Verify all software unit tests pass cleanly (206 tests)
python3 -m unittest discover -s tests -p "test_*.py"

# 3. Run the automated 9-stage deployment smoke test gate:
# Static checks (Stages 1-5):
./scripts/sitl/smoke_test_fleet.sh

# Full live execution (Stages 1-9 including Gazebo SITL spawn & heartbeat checks):
./scripts/sitl/smoke_test_fleet.sh --live
```
All commands must exit with return code `0` before authorizing live mission execution.
