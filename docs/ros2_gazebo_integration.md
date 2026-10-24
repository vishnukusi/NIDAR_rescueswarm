# ROS 2, PX4 SITL & Gazebo Swarm Integration (Step 5 Deliverable)

This document provides the complete integration blueprint for deploying the NIDAR RescueSwarm simulation onto a Linux system (Ubuntu 22.04 LTS / ROS 2 Humble / PX4 v1.14+ / Gazebo).

---

## 1. Architecture Overview

```text
+-----------------------------------------------------------------------------------+
|                                 GAZEBO SIMULATOR                                  |
|  Search Drones: S1, S2, S3 (RGB Nadir Camera)  |  Delivery Drones: D1, D2, D3     |
+-----------------------------------------------------------------------------------+
             | (Camera ROS 2 plugin)                           ^ (MAVLink / Offboard setpoints)
             v                                                 |
+--------------------------+                      +--------------------------+
|  YOLO Perception Node    |                      |  Flight Controller Bridge|
|  (Inference on best.pt)  |                      |  (MAVLinkFlightBridge)   |
+--------------------------+                      +--------------------------+
             |                                                 ^
             v /rescue/detections                              | Local NED velocities
+--------------------------+                      +--------------------------+
|  Geotagging & Ray-Cast   |                      |  Swarm Manager & Queue   |
|  (/rescue/survivor_geotag)--------------------->|  (Deduplication/Allocate)|
+--------------------------+                      +--------------------------+

> **Note on Autopilot Flight Stack**: See [flight_stack_resolution.md](file:///Users/teju/Desktop/rescue_swarm_sim%203/docs/flight_stack_resolution.md) for the complete resolution of the ArduCopter vs PX4 flight-stack interface. RescueSwarm standardizes on MAVLink `SET_POSITION_TARGET_LOCAL_NED` with yaw lock, supporting both ArduPilot GUIDED mode and PX4 OFFBOARD mode. Do not attempt direct velocity control via generic unmapped ROS 2 `/cmd_vel` topics.
```

---

## 2. Multi-Drone PX4 SITL & Gazebo Startup

Run Micro-XRCE-DDS Agent and spawn 6 vehicle instances with distinct system IDs and namespace ports:

```bash
#!/usr/bin/env bash
# launch_px4_swarm_sitl.sh

# 1. Start Micro XRCE-DDS Agent for ROS 2 / PX4 communication
MicroXRCEAgent udp4 -p 8888 &

# 2. Spawn 3 Search Drones (S1, S2, S3) with Downward Nadir Cameras
# Model: x500_gimbal or iris with nadir camera plugin
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=x500_gimbal ./build/px4_sitl_default/bin/px4 -i 1 -d &
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=x500_gimbal ./build/px4_sitl_default/bin/px4 -i 2 -d &
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=x500_gimbal ./build/px4_sitl_default/bin/px4 -i 3 -d &

# 3. Spawn 3 Delivery Drones (D1, D2, D3) with Payload Gripper / Release Actuator
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=x500_drop ./build/px4_sitl_default/bin/px4 -i 4 -d &
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=x500_drop ./build/px4_sitl_default/bin/px4 -i 5 -d &
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=x500_drop ./build/px4_sitl_default/bin/px4 -i 6 -d &
```

---

## 3. ROS 2 Custom Message Definitions

### `rescue_swarm_interfaces/msg/SurvivorDetection.msg`
```text
std_msgs/Header header
string observing_drone_id
int32 class_id
string class_name
float32 confidence
float32 xmin
float32 ymin
float32 xmax
float32 ymax
```

### `rescue_swarm_interfaces/msg/SurvivorGeoRecord.msg`
```text
std_msgs/Header header
string task_id
string detected_by
sensor_msgs/NavSatFix coordinates
float32 uncertainty_radius_m
float32 confidence
string assigned_delivery_drone
string task_status
```

### `rescue_swarm_interfaces/msg/FleetStatus.msg`
```text
std_msgs/Header header
string mission_state
int32 survivors_found
int32 deliveries_verified
string[] online_drones
float32[] battery_levels
```

---

## 4. ROS 2 Node Architecture

### Node 1: `perception_node.py`
Subscribes to `/s{1..3}/camera/image_raw`, runs inference using `best.pt`, and publishes detections:
- **Subscribe**: `/s1/camera/image_raw` (`sensor_msgs/Image`)
- **Publish**: `/rescue/detections` (`rescue_swarm_interfaces/SurvivorDetection`)

### Node 2: `geotagging_node.py`
Fuses image detections with observing drone telemetry (`/s1/fmu/out/vehicle_odometry`):
- Executes 3D ray-casting with terrain ground intersection (`rescue_swarm_core.localization`).
- **Publish**: `/rescue/survivor_records` (`rescue_swarm_interfaces/SurvivorGeoRecord`)

### Node 3: `swarm_manager_node.py`
Maintains global survivor records with spatial deduplication ($\le 10\text{m}$):
- Allocates highest-battery delivery drone (`D1`, `D2`, or `D3`).
- Tracks state: `ASSIGNED` $\rightarrow$ `EN_ROUTE` $\rightarrow$ `RELEASED` $\rightarrow$ `VERIFIED`.

### Node 4: `delivery_controller_node.py`
Drives delivery drone trajectory setpoints:
- Climbs to $25\,\text{m}$ cruise altitude $\rightarrow$ navigates to target $\rightarrow$ descends to $8\,\text{m}$ drop altitude $\rightarrow$ sends payload drop actuator command $\rightarrow$ commands Return-To-Launch (RTL).
- **Publish**: `/d{N}/fmu/in/trajectory_setpoint` (`px4_msgs/TrajectorySetpoint`)
- **Publish**: `/d{N}/payload/release` (`std_msgs/Bool`)

---

## 5. Verification on Linux Machine

```bash
# 1. Source ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/rescue_swarm_ws/install/setup.bash

# 2. Launch complete multi-drone rescue stack
ros2 launch rescue_swarm_bringup multi_drone_rescue.launch.py
```
