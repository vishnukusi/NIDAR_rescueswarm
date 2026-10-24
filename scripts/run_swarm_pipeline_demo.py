#!/usr/bin/env python3
"""Demonstration of the end-to-end RescueSwarm pipeline.

Simulates:
1. Search drones (S1, S2, S3) patrolling with camera calibrations & telemetry.
2. Structured detections projected onto terrain via 3D ray-casting (never naively using drone GPS).
3. SwarmManager spatial deduplication (S1 and S2 spotting the same survivor).
4. Deterministic task allocation to response/delivery drones (D1, D2, D3).
5. Delivery task lifecycle tracking (ASSIGNED -> EN_ROUTE -> RELEASED -> VERIFIED).
"""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core import (
    BoundingBox,
    CameraCalibration,
    Detection,
    DroneTelemetry,
    GeoPoint,
    MissionState,
    SwarmManager,
    TaskStatus,
    estimate_survivor_location,
)


def main() -> None:
    print("=" * 70)
    print("  NIDAR RescueSwarm — Autonomous Multi-Drone Coordination Pipeline")
    print("=" * 70)

    # 1. Initialize Swarm Manager from fleet config
    config_path = ROOT / "config" / "fleet.json"
    manager = SwarmManager.from_config_file(config_path)

    # Set initial mission state and battery fractions
    manager.mission_manager.transition_to(MissionState.INITIALIZING)
    manager.mission_manager.transition_to(MissionState.SEARCHING)

    # Set delivery drone battery states
    manager.get_drone("D1").battery_fraction = 0.72
    manager.get_drone("D2").battery_fraction = 0.91
    manager.get_drone("D3").battery_fraction = 0.83

    print(f"\n[1] Fleet Initialized:")
    print(f"    Search Fleet:   S1, S2, S3")
    print(f"    Delivery Fleet: D1 (72%), D2 (91%), D3 (83%)")
    print(f"    Mission State:  {manager.mission_manager.state.value.upper()}")

    # 2. Camera setup on search drones
    camera = CameraCalibration(
        image_width=1920,
        image_height=1080,
        hfov_deg=84.0,
        mount_pitch_deg=-90.0,  # Downward-looking nadir gimbal
    )

    # 3. Search Drone S1 detects a survivor slightly to the right of frame
    s1_telemetry = DroneTelemetry(
        drone_id="S1",
        position=GeoPoint(latitude=12.971600, longitude=77.594600, altitude_m=35.0),
        altitude_agl_m=30.0,
        heading_deg=0.0,  # Heading North
    )

    s1_detection = Detection(
        class_id=0,
        class_name="person",
        confidence=0.91,
        bbox=BoundingBox(xmin=0.70, ymin=0.45, xmax=0.78, ymax=0.55),  # East offset
    )

    # Calculate real ground coordinates using ray-casting geometry
    geo_estimate_1 = estimate_survivor_location(s1_detection, s1_telemetry, camera)

    print(f"\n[2] Search Drone S1 Detection:")
    print(f"    Drone GPS:       Lat {s1_telemetry.position.latitude:.6f}, Lon {s1_telemetry.position.longitude:.6f} at {s1_telemetry.altitude_agl_m}m AGL")
    print(f"    BBox Center:     ({s1_detection.bbox.center_x:.2f}, {s1_detection.bbox.center_y:.2f})")
    print(f"    Ray Intersection Offset: North={geo_estimate_1.offset_north_m:+.2f}m, East={geo_estimate_1.offset_east_m:+.2f}m")
    print(f"    Estimated Survivor Location: Lat {geo_estimate_1.location.latitude:.6f}, Lon {geo_estimate_1.location.longitude:.6f} (±{geo_estimate_1.uncertainty_radius_m:.1f}m)")

    task_1, is_new_1 = manager.process_detection(s1_detection, geo_estimate_1, detected_by_drone_id="S1")
    print(f"    Swarm Manager Action: Task {task_1.task_id} created (New={is_new_1}) -> Assigned to {task_1.assigned_drone_id}")

    # 4. Search Drone S2 patrolled near the same area and spots the same survivor from a different angle
    s2_telemetry = DroneTelemetry(
        drone_id="S2",
        position=GeoPoint(latitude=12.971620, longitude=77.594720, altitude_m=32.0),
        altitude_agl_m=28.0,
        heading_deg=270.0,  # Heading West
    )

    s2_detection = Detection(
        class_id=0,
        class_name="person",
        confidence=0.96,
        bbox=BoundingBox(xmin=0.48, ymin=0.48, xmax=0.52, ymax=0.52),
    )

    geo_estimate_2 = estimate_survivor_location(s2_detection, s2_telemetry, camera)
    task_2, is_new_2 = manager.process_detection(s2_detection, geo_estimate_2, detected_by_drone_id="S2")

    print(f"\n[3] Search Drone S2 Sighting (Spatially Overlapping):")
    print(f"    Estimated Target: Lat {geo_estimate_2.location.latitude:.6f}, Lon {geo_estimate_2.location.longitude:.6f}")
    print(f"    Deduplication Result: Is New Task = {is_new_2} (Matched {task_2.task_id})")
    print(f"    Updated Task Confidence: {task_2.confidence:.2f}")
    print(f"    Confirming Drones: {manager.survivor_records[task_2.task_id].confirming_drones}")

    # 5. Search Drone S3 detects a second distinct survivor 180m away
    s3_telemetry = DroneTelemetry(
        drone_id="S3",
        position=GeoPoint(latitude=12.973200, longitude=77.594600, altitude_m=30.0),
        altitude_agl_m=30.0,
        heading_deg=0.0,
    )
    s3_detection = Detection(
        class_id=0,
        class_name="person",
        confidence=0.88,
        bbox=BoundingBox(xmin=0.48, ymin=0.48, xmax=0.52, ymax=0.52),
    )
    geo_estimate_3 = estimate_survivor_location(s3_detection, s3_telemetry, camera)
    task_3, is_new_3 = manager.process_detection(s3_detection, geo_estimate_3, detected_by_drone_id="S3")

    print(f"\n[4] Search Drone S3 Detection (Distinct Location):")
    print(f"    Estimated Target: Lat {geo_estimate_3.location.latitude:.6f}, Lon {geo_estimate_3.location.longitude:.6f}")
    print(f"    Swarm Manager Action: Task {task_3.task_id} created (New={is_new_3}) -> Assigned to {task_3.assigned_drone_id}")

    # 6. Execute and verify delivery mission for Task 1
    print(f"\n[5] Executing Delivery Mission for Task {task_1.task_id}:")
    manager.update_task_status(task_1.task_id, TaskStatus.EN_ROUTE)
    print(f"    Status: {task_1.status.value.upper()} (Drone {task_1.assigned_drone_id})")
    manager.update_task_status(task_1.task_id, TaskStatus.RELEASED)
    print(f"    Status: {task_1.status.value.upper()} (Payload dropped)")
    manager.update_task_status(task_1.task_id, TaskStatus.VERIFIED)
    print(f"    Status: {task_1.status.value.upper()} (Delivery confirmed, Drone {task_1.assigned_drone_id} returned to READY)")

    # 7. Summary
    summary = manager.get_fleet_summary()
    print(f"\n[6] Swarm Fleet Summary:")
    print(f"    Mission State:       {summary['mission_state']}")
    print(f"    Total Survivors:     {summary['total_survivors_found']}")
    print(f"    Verified Deliveries: {summary['verified_deliveries']}")
    print(f"    Active Tasks:        {summary['active_tasks']}")
    for d_id, info in summary['drones'].items():
        if info['role'] == 'delivery':
            print(f"    {d_id}: status={info['status']:<7} active_task={info['active_task']} battery={info['battery']*100:.0f}%")

    print("\n" + "=" * 70)


if __name__ == "__main__":
    main()
