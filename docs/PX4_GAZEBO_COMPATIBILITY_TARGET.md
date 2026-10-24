# NIDAR RescueSwarm — Phase D: PX4, Gazebo, and ROS 2 Compatibility Target

## Date: 2026-10-24
## Role: Autonomous Senior Robotics Software & Integration Engineer

---

## 1. Selected Official Compatibility Baseline

To guarantee reproducibility and adhere to official upstream PX4 and Open Robotics documentation, NIDAR RescueSwarm selects a single unified simulation target:

| Component | Target Version | Release Details & Rationale |
| :--- | :--- | :--- |
| **Operating System** | **Ubuntu 24.04 LTS (Noble Numbat)** | Primary Linux target (matches host WSL 2 environment: kernel 6.18+). Backward-compatible with Ubuntu 22.04 LTS. |
| **Flight Stack** | **PX4 Autopilot (v1.15 / main commit `7c8d6daca9`)** | Current PX4 release with native Gazebo Harmonic simulation support (`make px4_sitl gz_x500`). Cloned in WSL at `/home/vishnu/PX4-Autopilot`. |
| **Simulator** | **Gazebo Harmonic (gz-sim8, v8.15.0)** | Modern Gazebo Simulator installed natively in WSL (`/usr/bin/gz`). Replaces legacy Gazebo Classic (Gazebo 11). |
| **ROS 2 Distribution** | **ROS 2 Jazzy Jalisco (Ubuntu 24.04)** / **Humble (Ubuntu 22.04)** | Standard LTS ROS 2 distributions. Uses standard `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, and `px4_msgs`. |
| **Bridge / Middleware** | **Micro XRCE-DDS Agent** (`micro-xrce-dds-agent`) + **pymavlink MAVLink UDP Bridge** | Dual-tier bridge: Micro XRCE-DDS for native ROS 2 topic exchange (`px4_msgs`); MAVLink UDP (ports 14550–14555) for direct flight bridge control. |
| **Python Runtime** | **Python 3.10+ (Python 3.12 on Ubuntu 24.04)** | Compatible with `rclpy`, PyTorch 2.0+, Ultralytics, and `pymavlink`. |

---

## 2. Official Documentation References

1. **PX4 SITL with Gazebo Harmonic**:
   - Official PX4 Docs: [https://docs.px4.io/main/en/sim_gazebo_gz/](https://docs.px4.io/main/en/sim_gazebo_gz/)
   - Multi-Vehicle Simulation: [https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html](https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html)
   - Build target: `make px4_sitl gz_x500`
2. **PX4 ROS 2 User Guide**:
   - Official PX4 ROS 2 Docs: [https://docs.px4.io/main/en/ros2/user_guide.html](https://docs.px4.io/main/en/ros2/user_guide.html)
   - Micro XRCE-DDS Agent: `MicroXRCEAgent udp4 -p 8888`
   - Messages repository: [https://github.com/PX4/px4_msgs](https://github.com/PX4/px4_msgs)
3. **Open Robotics Gazebo Harmonic**:
   - Documentation: [https://gazebosim.org/docs/harmonic](https://gazebosim.org/docs/harmonic)

---

## 3. Six-Vehicle Swarm Networking & Instance Isolation

Each simulated vehicle operates with strict port, system ID, instance, and namespace isolation:

| Drone ID | Role | PX4 Instance `-i` | MAVLink UDP Port | DDS Namespace / Topic Prefix | GZ Model Name |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **S1** | Search | `0` | `14550` | `/s1/` | `x500_gimbal_0` |
| **S2** | Search | `1` | `14551` | `/s2/` | `x500_gimbal_1` |
| **S3** | Search | `2` | `14552` | `/s3/` | `x500_gimbal_2` |
| **D1** | Delivery | `3` | `14553` | `/d1/` | `x500_drop_3` |
| **D2** | Delivery | `4` | `14554` | `/d2/` | `x500_drop_4` |
| **D3** | Delivery | `5` | `14555` | `/d3/` | `x500_drop_5` |

**Phase D Acceptance Criteria: MET.**
