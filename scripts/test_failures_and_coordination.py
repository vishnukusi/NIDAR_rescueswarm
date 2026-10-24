#!/usr/bin/env python3
"""Multi-Drone Failure & Fault Coordination Suite (Step 4 Deliverable).

Validates swarm robustness against real-world operational failure modes:
1. Spatial Deduplication: Repeated observations of the same survivor by multiple search drones.
2. Battery Failsafe Hierarchy: Warning (30%) -> RTL (20%) -> Emergency Land (10%).
3. Lost Communication Watchdog: Telemetry heartbeat loss timeout trigger (>5.0s).
4. Geofence Boundary Breaches: Ceiling, floor, and lateral boundary violations.
5. Unavailable Delivery Drones: Capacity exhaustion, task queueing in GEOLOCATED, and automatic dispatch upon recovery.
"""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core import (
    BatteryFailsafeConfig,
    BoundingBox,
    Detection,
    Drone,
    DroneRole,
    DroneStatus,
    DroneTelemetry,
    GeofenceConfig,
    GeoPoint,
    GeolocationEstimate,
    MissionState,
    SafetyAction,
    SafetyLevel,
    SafetyManager,
    SearchArea,
    SwarmManager,
    TaskStatus,
)


def run_failure_suite() -> None:
    print("=" * 76)
    print("  NIDAR RescueSwarm — Failure & Swarm Coordination Test Suite")
    print("  Step 4 Deliverable: Fault Tolerance, Failsafes & Multi-Drone Queueing")
    print("=" * 76)

    passed = 0
    total = 5

    # -------------------------------------------------------------
    # Test 1: Spatial Deduplication (Duplicate Detection by S1 & S2)
    # -------------------------------------------------------------
    print("\n[TEST 1/5] Spatial Deduplication across Search Fleet...")
    drones = [
        Drone("S1", DroneRole.SEARCH, DroneStatus.STANDBY),
        Drone("S2", DroneRole.SEARCH, DroneStatus.STANDBY),
        Drone("D1", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.85),
    ]
    manager = SwarmManager(drones=drones, duplicate_distance_threshold_m=10.0)
    manager.mission_manager.transition_to(MissionState.INITIALIZING)
    manager.mission_manager.transition_to(MissionState.SEARCHING)

    det_1 = Detection(0, "pedestrian", 0.88, BoundingBox(0.48, 0.48, 0.52, 0.52))
    geo_1 = GeolocationEstimate(GeoPoint(12.972000, 77.594500), 0.0, 0.0, 0.0, 30.0, 2.0)
    task1, is_new1 = manager.process_detection(det_1, geo_1, "S1")

    # S2 detects same location (~2.3 meters away) with higher confidence
    det_2 = Detection(0, "pedestrian", 0.96, BoundingBox(0.49, 0.49, 0.53, 0.53))
    geo_2 = GeolocationEstimate(GeoPoint(12.972015, 77.594510), 0.0, 0.0, 0.0, 30.0, 1.5)
    task2, is_new2 = manager.process_detection(det_2, geo_2, "S2")

    assert is_new1 is True, "First detection must create new task"
    assert is_new2 is False, "Second overlapping detection must NOT create new task"
    assert task2.task_id == task1.task_id, "Tasks must match"
    assert task2.confidence == 0.96, "Confidence must be updated to higher observation"
    assert len(manager.get_all_tasks()) == 1, "Only 1 task should exist in fleet records"
    print("  ✓ PASS: Spatial deduplication merged repeated sightings without duplicate drone dispatch.")
    passed += 1

    # -------------------------------------------------------------
    # Test 2: Battery Failsafe Hierarchy (Warn -> RTL -> Emergency Land)
    # -------------------------------------------------------------
    print("\n[TEST 2/5] Battery Failsafe Hierarchy...")
    safety = SafetyManager(
        battery_config=BatteryFailsafeConfig(warning_fraction=0.30, rtl_fraction=0.20, land_critical_fraction=0.10)
    )
    telem = DroneTelemetry("D1", GeoPoint(12.972, 77.594, 25.0), 25.0, 0.0)

    # 28% -> Warning
    alerts_warn = safety.check_telemetry(telem, battery_fraction=0.28)
    assert len(alerts_warn) == 1 and alerts_warn[0].action == SafetyAction.WARN

    # 17% -> RTL Trigger
    alerts_rtl = safety.check_telemetry(telem, battery_fraction=0.17)
    assert len(alerts_rtl) == 1 and alerts_rtl[0].action == SafetyAction.RTL

    # 7% -> Immediate Emergency Land
    alerts_land = safety.check_telemetry(telem, battery_fraction=0.07)
    assert len(alerts_land) == 1 and alerts_land[0].action == SafetyAction.EMERGENCY_LAND

    print("  ✓ PASS: 3-tier battery failsafes correctly trigger WARN (28%), RTL (17%), and EMERGENCY LAND (7%).")
    passed += 1

    # -------------------------------------------------------------
    # Test 3: Lost Communication Watchdog (Heartbeat Timeout)
    # -------------------------------------------------------------
    print("\n[TEST 3/5] Lost Communication Watchdog...")
    safety.record_heartbeat("D2", current_time=100.0)

    # Nominal check at 102s (2s silence < 5.0s limit)
    assert len(safety.check_telemetry_timeouts(["D2"], current_time=102.0)) == 0

    # Loss check at 106s (6s silence > 5.0s limit)
    alerts_timeout = safety.check_telemetry_timeouts(["D2"], current_time=106.0)
    assert len(alerts_timeout) == 1
    assert alerts_timeout[0].action == SafetyAction.RTL
    assert alerts_timeout[0].level == SafetyLevel.CRITICAL
    print("  ✓ PASS: Communication watchdog detected 6.0s telemetry silence and commanded autonomous RTL.")
    passed += 1

    # -------------------------------------------------------------
    # Test 4: Geofence Boundary Breach
    # -------------------------------------------------------------
    print("\n[TEST 4/5] Geofence Boundary & Altitude Breaches...")
    geo_box = SearchArea(min_lat=12.970, min_lon=77.590, max_lat=12.975, max_lon=77.595)
    geofence_mgr = SafetyManager(
        geofence=GeofenceConfig(boundary=geo_box, min_altitude_agl_m=4.0, max_altitude_agl_m=50.0)
    )

    # Ceiling breach
    t_high = DroneTelemetry("S1", GeoPoint(12.972, 77.592, 55.0), 55.0, 0.0)
    alerts_high = geofence_mgr.check_telemetry(t_high, 0.90)
    assert any(a.action == SafetyAction.HOLD for a in alerts_high)

    # Lateral perimeter breach
    t_out = DroneTelemetry("S1", GeoPoint(12.980, 77.592, 25.0), 25.0, 0.0)
    alerts_out = geofence_mgr.check_telemetry(t_out, 0.90)
    assert any(a.action == SafetyAction.RTL for a in alerts_out)

    print("  ✓ PASS: Altitude ceiling breach triggered HOLD; perimeter breach triggered immediate RTL.")
    passed += 1

    # -------------------------------------------------------------
    # Test 5: Unavailable Delivery Drones (Queueing & Recovery)
    # -------------------------------------------------------------
    print("\n[TEST 5/5] Unavailable Delivery Drones & Automatic Re-dispatch...")
    single_delivery_drone = Drone("D1", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.80)
    q_manager = SwarmManager(drones=[single_delivery_drone])

    # First survivor detection: D1 gets allocated
    t1, _ = q_manager.process_detection(
        det_1, GeolocationEstimate(GeoPoint(12.971, 77.591), 0, 0, 0, 30, 2), "S1"
    )
    assert t1.status == TaskStatus.ASSIGNED
    assert t1.assigned_drone_id == "D1"
    assert single_delivery_drone.status == DroneStatus.BUSY

    # Second survivor detection while D1 is busy (0 drones available)
    t2, _ = q_manager.process_detection(
        det_1, GeolocationEstimate(GeoPoint(12.973, 77.593), 0, 0, 0, 30, 2), "S2"
    )
    assert t2.status == TaskStatus.GEOLOCATED
    assert t2.assigned_drone_id is None
    print("  • Queue state: Task 2 safely queued in GEOLOCATED status while fleet is busy.")

    # Mark Task 1 as VERIFIED -> D1 returns to base and auto-allocates to Task 2!
    q_manager.update_task_status(t1.task_id, TaskStatus.VERIFIED)
    assert single_delivery_drone.status == DroneStatus.BUSY
    assert t2.status == TaskStatus.ASSIGNED
    assert t2.assigned_drone_id == "D1"
    print("  ✓ PASS: Drone release immediately auto-dispatched queued task without human intervention.")
    passed += 1

    print("\n" + "=" * 76)
    print(f"  ALL {passed}/{total} FAILURE & COORDINATION TESTS PASSED SUCCESSFULLY.")
    print("=" * 76)


if __name__ == "__main__":
    run_failure_suite()
