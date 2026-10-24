# NIDAR RescueSwarm — Phase C: Six-Drone Autonomy Core Stabilization

## Date: 2026-10-24
## Role: Autonomous Senior Robotics Software & Integration Engineer

---

## 1. Fleet Configuration & Architecture ([`config/fleet.json`](file:///c:/Users/chaya/Downloads/rescue_swarm_sim%203/rescue_swarm_sim%203/config/fleet.json))

All six fleet vehicles are uniquely defined with isolated operational identities:
- **Search Fleet**:
  - `S1`: System ID 1, Instance 0, MAVLink UDP port 14550, ROS 2 namespace `s1`, Nadir Camera (1920x1080, 84° HFOV, pitch -90°).
  - `S2`: System ID 2, Instance 1, MAVLink UDP port 14551, ROS 2 namespace `s2`, Nadir Camera (1920x1080, 84° HFOV, pitch -90°).
  - `S3`: System ID 3, Instance 2, MAVLink UDP port 14552, ROS 2 namespace `s3`, Nadir Camera (1920x1080, 84° HFOV, pitch -90°).
- **Delivery Fleet**:
  - `D1`: System ID 4, Instance 3, MAVLink UDP port 14553, ROS 2 namespace `d1`, Payload Capacity 500g (200g kit).
  - `D2`: System ID 5, Instance 4, MAVLink UDP port 14554, ROS 2 namespace `d2`, Payload Capacity 500g (200g kit).
  - `D3`: System ID 6, Instance 5, MAVLink UDP port 14555, ROS 2 namespace `d3`, Payload Capacity 500g (200g kit).

## 2. Autonomy Subsystems & Safety Boundaries

1. **Deterministic Delivery Allocator (`allocator.py`)**:
   - Considers vehicle role, status, active task, and battery fraction (`minimum_battery_fraction=0.30`).
   - Rejects allocation for faulted, offline, or low-battery vehicles.
   - Assigning a delivery drone transitions task status to `ASSIGNED`, never `VERIFIED`.

2. **Perception & Localization (`perception.py`, `localization.py`)**:
   - `YOLOPerceptionAdapter` interfaces with genuine YOLOv8 models (`best.pt`).
   - Normalizes bounding boxes, validates class filtering (target: "pedestrian"), checks minimum confidence threshold (0.25).
   - 3D ray-casting geometry calculates ground position relative to camera focal geometry and vehicle attitude.
   - Uncertainty radius $\sigma_r$ accounts for attitude and altitude jitter. Confidence is never confused with measured physical accuracy.

3. **Survivor Association & Tracking (`association.py`)**:
   - Scopes tracklet IDs by drone ID (`scoped_track = f"{drone_id}_{track_id}"`).
   - Distinguishes repeated observations of the same survivor ($\le 10\text{ m}$) from distinct survivors ($\ge 3\text{ m}$).
   - Handles re-sightings of failed tasks to trigger bounded delivery retry (`retry_of`).

4. **Simulation vs Real Observation Boundaries**:
   - In automated unit tests, scripted `ObservationEvent`s provide deterministic inputs.
   - In real/SITL runs, camera frames are subscribed over ROS 2 topics (`sensor_msgs/Image`), preventing fabricated detections if cameras disconnect.

**Phase C Acceptance Criteria: MET.**
