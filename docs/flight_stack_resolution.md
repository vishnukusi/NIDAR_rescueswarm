# Flight-Stack & Simulator Mismatch Resolution

## 1. Identified Architecture Mismatch

The previous audit and code inspection identified two divergent flight-stack paradigms across the repository:

1. **Embedded Drone Project (`docs/Drone_project/`)**:
   - Autopilot: **ArduCopter**
   - Protocol: **MAVLink v2** via `pymavlink`
   - Command Interface: `SET_POSITION_TARGET_LOCAL_NED` (Type mask: velocity X/Y/Z enabled, yaw locked, position/acceleration ignored)
   - Modes: `GUIDED`, `RTL`, `LAND`
   - Connection: MAVProxy / ArduCopter SITL (`udp:127.0.0.1:14550`)

2. **Planned Integration Blueprint (`docs/ros2_gazebo_integration.md`)**:
   - Autopilot: **PX4 Autopilot v1.14+**
   - Middleware: **ROS 2 Humble** via Micro-XRCE-DDS
   - Command Interface: Naively assumed ROS 2 topics (`/cmd_vel` or `TrajectorySetpoint`)

### The Critical Pitfall
Neither ArduPilot nor PX4 natively accepts raw `/cmd_vel` (`geometry_msgs/Twist`) topics without an active companion offboard translation node:
- **PX4 Offboard**: Requires continuous streaming (>2 Hz) of `px4_msgs/OffboardControlMode` alongside `px4_msgs/TrajectorySetpoint` with synchronized monotonic timestamps. Dropping packets causes PX4 to immediately fail-safe into `HOLD` or `RTL`.
- **ArduCopter Guided**: Rejects raw velocity topics unless packaged in MAVLink `SET_POSITION_TARGET_LOCAL_NED` commands.

---

## 2. Selected Architecture & Unified Adapter

To preserve the team's working visual servoing controller while supporting both ArduPilot and PX4 targets on the Linux machine, RescueSwarm establishes **MAVLink `SET_POSITION_TARGET_LOCAL_NED` with Local Yaw Lock** as the universal flight-control abstraction:

```text
+-----------------------------------------------------------------------------------------+
|                                    RESCUESWARM CORE                                     |
|                                                                                         |
|  Search Drone Controller            Swarm Manager              Safety Manager           |
|  (SearchDroneApproachController)    (Deduplication/Tasks)      (Geofence/Battery/Proximity)|
+-----------------------------------------------------------------------------------------+
                                             |
                                             v
+-----------------------------------------------------------------------------------------+
|                  FLIGHT CONTROLLER BRIDGE (rescue_swarm_core.flight_bridge)              |
|                                                                                         |
|  MAVLinkFlightBridge                                                                    |
|  ├── Mode Management: APM "GUIDED" (custom_mode=4) / PX4 "OFFBOARD" (custom_mode=6)    |
|  ├── Setpoint Generator: MAVLink SET_POSITION_TARGET_LOCAL_NED (Vn, Ve, Vd, yaw)       |
|  ├── Telemetry Parser: GLOBAL_POSITION_INT, ATTITUDE, HEARTBEAT                         |
|  └── Safety Preemption: SafetyAction.HOLD / RTL / EMERGENCY_LAND immediate override    |
+-----------------------------------------------------------------------------------------+
                                             |
                      +----------------------+----------------------+
                      |                                             |
                      v (UDP 14550..14555)                          v (UDP 14540..14545)
          +-----------------------+                     +-----------------------+
          |    ARDUPILOT SITL     |                     |       PX4 SITL        |
          |       (Gazebo)        |                     |   (Gazebo Classic /   |
          |                       |                     |     Gazebo Sim)       |
          +-----------------------+                     +-----------------------+
```

### Key Advantages:
1. **Zero Controller Rewrite**: Reuses the exact visual servoing math, deadbands, EMA filtering, and yaw locking already proven in `docs/Drone_project/`.
2. **Dual-Stack Portability**: Both ArduPilot SITL and PX4 SITL natively accept MAVLink `SET_POSITION_TARGET_LOCAL_NED`. Switching between ArduPilot and PX4 is a single configuration flag (`AutopilotType.ARDUPILOT` vs `AutopilotType.PX4`).
3. **Companion Isolation**: Runs on macOS locally via loopback emulation and seamlessly binds to real SITL ports on the senior's Linux machine without code modifications.

---

## 3. Fleet Multi-Drone Port & Endpoint Mapping

To scale from single-drone validation (S1) to all 6 vehicles without command cross-talk:

| Vehicle | Role | Autopilot System ID | Primary SITL Port (ArduPilot / MAVProxy) | PX4 MAVLink Port |
| :---: | :---: | :---: | :---: | :---: |
| **S1** | Search 1 | `1` | `udp:127.0.0.1:14550` | `udp:127.0.0.1:14540` |
| **S2** | Search 2 | `2` | `udp:127.0.0.1:14551` | `udp:127.0.0.1:14541` |
| **S3** | Search 3 | `3` | `udp:127.0.0.1:14552` | `udp:127.0.0.1:14542` |
| **D1** | Delivery 1 | `4` | `udp:127.0.0.1:14553` | `udp:127.0.0.1:14543` |
| **D2** | Delivery 2 | `5` | `udp:127.0.0.1:14554` | `udp:127.0.0.1:14544` |
| **D3** | Delivery 3 | `6` | `udp:127.0.0.1:14555` | `udp:127.0.0.1:14545` |

---

## 4. Linux SITL Setup & Verification Procedure

### 4.1 Prerequisites & Toolchain Setup (Ubuntu 22.04 LTS)

Execute once on the Ubuntu development host:

```bash
# 1. Update system and install essential build dependencies
sudo apt update && sudo apt install -y git python3-pip python3-venv tmux net-tools

# 2. Clone ArduPilot source tree (with submodules)
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git $HOME/ardupilot
cd $HOME/ardupilot

# 3. Run ArduPilot prerequisites installer
Tools/environment_install/install-prereqs-ubuntu.sh -y
source ~/.profile

# 4. Pre-compile ArduCopter SITL binary
cd $HOME/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --build-target copter --no-start

# 5. Setup Python virtual environment for RescueSwarm
cd ~/rescue_swarm_sim
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

---

### 4.2 Single-Drone S1 Real SITL Verification (Run This First!)

Do **NOT** start the 6-vehicle swarm simulation before verifying single drone S1!

#### Step A: Launch S1 SITL in Terminal 1
```bash
cd ~/rescue_swarm_sim
./scripts/launch_ardupilot_s1_sitl.sh
```
* **Instance ID**: `0` (Internal SITL TCP: `5760`)
* **MAVLink System ID**: `1`
* **Output Endpoint**: `udp:127.0.0.1:14550`
* **Wait**: Watch the MAVProxy console for ~15–20s until EKF3 publishes:
  `EKF3 IMU0 is using GPS` and `Origin set to ...`

#### Step B: Verify the Active UDP Socket in Terminal 2 (Optional Sanity Check)
```bash
ss -ulnp | grep 14550
# Or: netstat -ulnp | grep 14550
```
Confirms MAVProxy is actively sending MAVLink packets on UDP port 14550.

#### Step C: Execute the Real MAVLink Verification Suite in Terminal 2
```bash
cd ~/rescue_swarm_sim
PYTHONPATH=src .venv/bin/python scripts/verify_s1_real_sitl.py --connection udp:127.0.0.1:14550
```
This script exercises:
1. Heartbeat reception (from `udp:127.0.0.1:14550`)
2. Vehicle identification (`System ID == 1`)
3. Real-time telemetry (`GLOBAL_POSITION_INT`, `ATTITUDE`, `SYS_STATUS`)
4. Preflight health checks
5. Mode change to `GUIDED` with confirmation
6. Propulsion arming with confirmation
7. Autonomous takeoff to 15.0m AGL
8. NED velocity setpoints (`SET_POSITION_TARGET_LOCAL_NED` for North and East)
9. Position hold / hover deceleration
10. `RTL` (Return-to-Launch) failsafe command and disarm

#### Step D: Run the S1 Autonomous Pipeline with YOLOv8 Vision
```bash
PYTHONPATH=src .venv/bin/python scripts/run_s1_sitl_pipeline.py --autopilot ardupilot --connection udp:127.0.0.1:14550
```

#### Step E: Safe Simulator Shutdown
* Press `Ctrl+C` in Terminal 1, or run:
```bash
pkill -f "arducopter|sim_vehicle.py|mavproxy.py"
```

---

### 4.3 Multi-Vehicle 6-Drone Fleet Simulation (Only After S1 Passes!)

Once S1 passes all checks, launch the coordinated 6-vehicle fleet:

#### Terminal 1: Launch all 6 SITL Instances across Dedicated Ports
```bash
./scripts/launch_ardupilot_swarm_sitl.sh --confirmed
```
* S1 (SysID 1) $\to$ Port 14550
* S2 (SysID 2) $\to$ Port 14551
* S3 (SysID 3) $\to$ Port 14552
* D1 (SysID 4) $\to$ Port 14553
* D2 (SysID 5) $\to$ Port 14554
* D3 (SysID 6) $\to$ Port 14555

#### Terminal 2: Run Full Fleet Mission Simulation
```bash
PYTHONPATH=src .venv/bin/python scripts/run_full_mission_sim.py
```
