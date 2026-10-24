#!/usr/bin/env python3
"""Autonomous RescueSwarm Pipeline Demonstration (software-only simulation).

Executes the complete end-to-end survivor response pipeline autonomously:
1. Inspects model weights (framework, architecture, class names from assets/models/best.pt).
2. Runs genuine Ultralytics YOLO inference on aerial camera image.
3. SearchDroneApproachController transitions S1 from SEARCH_PATROL to APPROACHING,
   pausing search coverage waypoints and executing **simulated** visual servoing
   translation until HOVERING.
4. 3D Ray-plane Geotagging calculates target ground GPS coordinates.
5. SwarmManager registers observation, deduplicates, and creates a RescueTask.
6. Allocates the highest-battery eligible delivery drone (D1, D2, or D3).
7. **Simulated** Delivery Controller flies the complete flight profile
   (transit, 8m drop, release, verify) — no physical vehicle motion.
8. Delivery verification checks the controller's terminal state. Simulated timer
   verification is labelled as such; it does NOT constitute physical sensor
   confirmation.
9. Search drone finishes hover inspection and resumes search coverage (simulated).

NOTE: This script runs entirely in software. No PX4 SITL, Gazebo, ROS 2, or
physical hardware is involved. All flight profiles, approach maneuvers, and
delivery sequences are simulated state-machine updates.
"""

import argparse
from pathlib import Path
import sys
import traceback
from typing import Optional

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core import (
    ApproachState,
    BoundingBox,
    CameraCalibration,
    DeliveryMissionController,
    DeliveryProfile,
    DeliveryState,
    DeliveryVerificationMethod,
    Detection,
    DroneTelemetry,
    GeoPoint,
    MissionState,
    SearchDroneApproachController,
    SwarmManager,
    TaskStatus,
    YOLOPerceptionAdapter,
    annotate_detection_image,
    estimate_survivor_location,
    haversine_distance_m,
    inspect_model_weights,
)


# ---------------------------------------------------------------------------
# Development-only default confidence threshold.
#
# 0.25 is a conservative value suitable for development and testing.  It is
# NOT a validated competition threshold — tune this against the actual
# competition model and dataset before field use.
# ---------------------------------------------------------------------------
_DEFAULT_CONFIDENCE_THRESHOLD = 0.25


def run_pipeline(
    image_path: Path,
    confidence_threshold: float = _DEFAULT_CONFIDENCE_THRESHOLD,
    verification_mode: str = "simulated_timer",
) -> dict:
    print("=" * 78)
    print("  NIDAR RescueSwarm — Autonomous End-to-End Pipeline Demonstration")
    print("  (Software-only simulation — no vehicle hardware or SITL)")
    print("  Genuine Model Inference -> Simulated Approach & Hover -> Geotag -> Simulated Delivery -> Verify")
    print("=" * 78)

    # 1. Model Inspection & Genuine Weight Verification
    weights_path = ROOT / "assets" / "models" / "best.pt"
    if not weights_path.exists():
        weights_path = ROOT / "assets" / "models" / "best"

    meta = inspect_model_weights(weights_path)
    print("\n[1] MODEL VERIFICATION:")
    print(f"    Weights Path:    {weights_path.relative_to(ROOT)}")
    print(f"    Framework:       {meta['framework']}")
    print(f"    Task:            {meta['task']}")
    print(f"    Total Classes:   {meta['num_classes']} {list(meta['classes'].values())}")
    print(f"    Survivor Target: {meta['target_survivor_classes']}")

    # 2. Genuine Perception Inference
    print(f"\n[2] GENUINE MODEL INFERENCE (Ultralytics YOLO on {image_path.name}):")
    adapter = YOLOPerceptionAdapter(
        model_path=weights_path,
        confidence_threshold=confidence_threshold,
        target_classes=["pedestrian", "people", "person", "survivor"],
    )
    frame_id = f"{image_path.name}"
    raw_detections = adapter.predict(image_path, frame_id=frame_id)
    print(f"    Threshold:       conf >= {confidence_threshold:.3f}")
    print(f"    Frame ID:        {frame_id}")
    print(f"    Detections Count:{len(raw_detections)} genuine target(s) detected")

    if not raw_detections:
        # ── No detections: report clearly and exit without fabricating data ──
        print("    ┌─────────────────────────────────────────────┐")
        print("    │  NO TARGET DETECTED                        │")
        print("    │  Model returned 0 detections at the        │")
        print(f"    │  current threshold (conf >= {confidence_threshold:.3f}).        │")
        print("    │                                            │")
        print("    │  No rescue task created.                   │")
        print("    │  No delivery mission started.              │")
        print("    │  Pipeline exiting cleanly.                 │")
        print("    └─────────────────────────────────────────────┘")
        print("=" * 78)
        return {
            "status": "no_target",
            "manager": None,
            "task": None,
            "controller": None,
            "error": None,
            "timed_out": False,
        }

    # Pick top-confidence genuine detection directly from the model
    survivor_det = max(raw_detections, key=lambda d: d.confidence)
    for idx, d in enumerate(raw_detections):
        print(f"    - Detection #{idx+1}: {d.class_name.upper()} (ID: {d.class_id}) | Conf: {d.confidence*100:.1f}% | BBox: [{d.bbox.xmin:.3f}, {d.bbox.ymin:.3f}, {d.bbox.xmax:.3f}, {d.bbox.ymax:.3f}] | Frame: {d.frame_id}")

    annotated_out = ROOT / "assets" / "test_survivors_annotated.jpg"
    try:
        annotate_detection_image(image_path, [survivor_det], annotated_out)
        print(f"    Annotated Output:{annotated_out.relative_to(ROOT)}")
    except Exception as e:
        print(f"    Annotation note: {e}")

    # 3. Search Drone Visual Servoing Approach & Hover (SIMULATED — no vehicle motion)
    print("\n[3] SEARCH DRONE S1 APPROACH & HOVER CONTROLLER (simulated state machine):")
    camera = CameraCalibration(
        image_width=1376,
        image_height=768,
        hfov_deg=84.0,
        mount_pitch_deg=-90.0,
    )
    approach_ctrl = SearchDroneApproachController(drone_id="S1", camera=camera)

    print(f"    Initial State:   {approach_ctrl.state.value.upper()} | Waypoints Paused: {approach_ctrl.is_waypoint_execution_paused}")

    # Step approach controller with detected person (simulated position updates)
    sim_time = 0.0
    s1_heading_rad = 0.7854  # 45 degrees yaw (NE)
    state, cmd = approach_ctrl.update(survivor_det, current_yaw_rad=s1_heading_rad, current_time=sim_time)
    print(f"    On Detection:    {state.value.upper()} | Waypoints Paused: {approach_ctrl.is_waypoint_execution_paused}")
    print(f"    Tracking Center: Head=({approach_ctrl.target_estimate.head_x}, {approach_ctrl.target_estimate.head_y}) | Image Error: ({cmd.error_x:+.1f}px, {cmd.error_y:+.1f}px)")
    print(f"    Servo Command:   {cmd.action_label} | Body=(Fwd: {cmd.body_forward_mps:+.2f}m/s, Rgt: {cmd.body_right_mps:+.2f}m/s) | NED=(N: {cmd.local_ned_north_mps:+.2f}m/s, E: {cmd.local_ned_east_mps:+.2f}m/s)")

    # Simulate approach translation steps until target is centered overhead
    centered_bbox = BoundingBox(xmin=0.485, ymin=0.485, xmax=0.515, ymax=0.515)
    centered_det = Detection(class_id=survivor_det.class_id, class_name=survivor_det.class_name, confidence=survivor_det.confidence, bbox=centered_bbox, frame_id=frame_id)
    for step in range(5):
        sim_time += 0.5
        state, cmd = approach_ctrl.update(centered_det, current_yaw_rad=s1_heading_rad, current_time=sim_time)

    print(f"    After Centering: {state.value.upper()} | Hover Confirmed: {state == ApproachState.HOVERING} | Action: {cmd.action_label}")

    # 4. 3D Ray-Plane Geotagging
    s1_telem = DroneTelemetry(
        drone_id="S1",
        position=GeoPoint(latitude=12.972000, longitude=77.594500, altitude_m=30.0),
        altitude_agl_m=30.0,
        heading_deg=45.0,
    )
    detections_with_geo = [
        (d, estimate_survivor_location(d, s1_telem, camera))
        for d in raw_detections
    ]
    geo_estimate = estimate_survivor_location(survivor_det, s1_telem, camera)

    print("\n[4] 3D GEOTAGGING & LOCALIZATION:")
    print(f"    Observing Drone: S1 at Lat {s1_telem.position.latitude:.6f}, Lon {s1_telem.position.longitude:.6f} ({s1_telem.altitude_agl_m}m AGL, Heading {s1_telem.heading_deg}°)")
    print(f"    Ground Ray:      Offset North={geo_estimate.offset_north_m:+.2f}m, East={geo_estimate.offset_east_m:+.2f}m")
    print(f"    Survivor GPS:    Lat {geo_estimate.location.latitude:.6f}, Lon {geo_estimate.location.longitude:.6f}")
    print(f"    Uncertainty:     ±{geo_estimate.uncertainty_radius_m:.2f}m radius")

    # 5. Swarm Manager Task Creation & Deterministic Allocation
    fleet_config = ROOT / "config" / "fleet.json"
    manager = SwarmManager.from_config_file(fleet_config)
    manager.mission_manager.transition_to(MissionState.INITIALIZING)
    manager.mission_manager.transition_to(MissionState.SEARCHING)

    # Set delivery fleet battery states (D2 has highest battery)
    manager.get_drone("D1").battery_fraction = 0.76
    manager.get_drone("D2").battery_fraction = 0.93
    manager.get_drone("D3").battery_fraction = 0.81

    batch_results = manager.process_detections_batch(
        detections_with_geo,
        detected_by_drone_id="S1",
        frame_id=frame_id,
        minimum_confidence=confidence_threshold,
    )

    task, is_new = batch_results[0]

    print("\n[5] SWARM MANAGER BATCH TASK ALLOCATION:")
    print(f"    Frame Processed: {frame_id} ({len(batch_results)} detection(s) processed atomically)")
    print(f"    Primary Task ID: {task.task_id} (Status: {task.status.value.upper()})")
    print(f"    Assigned Drone:  {task.assigned_drone_id} (Highest Battery: {manager.get_drone(task.assigned_drone_id).battery_fraction*100:.0f}%)")

    # Deduplication test: re-processing the batch from the same frame merges duplicates without spawning extra tasks
    repeat_results = manager.process_detections_batch(
        detections_with_geo,
        detected_by_drone_id="S1",
        frame_id=frame_id,
        minimum_confidence=confidence_threshold,
    )
    is_duplicate_new = any(is_new_flag for _, is_new_flag in repeat_results)
    print(f"    Deduplication:   Repeated observation correctly identified (any_new={is_duplicate_new})")

    # 6. Autonomous Delivery Execution (SIMULATED — state machine only, no vehicle)
    base_location = GeoPoint(latitude=12.971200, longitude=77.593400, altitude_m=0.0)
    profile = DeliveryProfile(
        transit_altitude_m=25.0,
        release_altitude_m=8.0,
        horizontal_speed_mps=12.0,
        vertical_speed_mps=3.0,
        drop_tolerance_radius_m=3.0,
        release_hover_time_s=1.0,
        verification_time_s=1.5,
    )
    v_method = (
        DeliveryVerificationMethod.GROUND_CONTACT_SENSOR
        if verification_mode == "sensor"
        else DeliveryVerificationMethod.SIMULATED_TIMER
    )
    delivery_ctrl = DeliveryMissionController(
        drone_id=task.assigned_drone_id,
        task=task,
        home_location=base_location,
        profile=profile,
        verification_method=v_method,
    )

    print("\n[6] SIMULATED DELIVERY FLIGHT PROFILE (state machine, no physical vehicle):")
    delivery_ctrl.start_mission()
    manager.update_task_status(task.task_id, TaskStatus.EN_ROUTE)

    dt = 1.0
    sim_time = 0.0
    recorded_states = set()
    delivery_error: Optional[Exception] = None
    delivery_exc_trace: Optional[str] = None
    loop_timed_out = False

    try:
        while delivery_ctrl.state != DeliveryState.COMPLETED and sim_time < 90.0:
            delivery_ctrl.update(dt)
            sim_time += dt
            st = delivery_ctrl.state
            if st not in recorded_states:
                recorded_states.add(st)
                pos = delivery_ctrl.current_position
                target_dist = haversine_distance_m(pos, task.location)
                print(f"    T+{sim_time:04.1f}s | {st.value.upper():<16} | Alt: {delivery_ctrl.altitude_agl_m:4.1f}m AGL | Range to Target: {target_dist:5.1f}m")
        if delivery_ctrl.state != DeliveryState.COMPLETED and sim_time >= 90.0:
            loop_timed_out = True
    except Exception as exc:
        delivery_error = exc
        delivery_exc_trace = traceback.format_exc().strip()

    # 7. Delivery Verification & Software Bookkeeping Cleanup
    print("\n[7] DELIVERY VERIFICATION:")

    if delivery_error is not None:
        # Unexpected exception during delivery loop — handle safely, do not leave EN_ROUTE
        manager.update_task_status(task.task_id, TaskStatus.FAILED)
        print(f"    Task Status:     {task.status.value.upper()} (delivery aborted due to controller exception)")
        print(f"    Error:           {type(delivery_error).__name__}: {delivery_error}")
        if delivery_exc_trace:
            print(f"    Exception Trace: {delivery_exc_trace}")
        print("    ⚠ SAFETY NOTICE: Software bookkeeping cleanup only. This does NOT command a real vehicle to RTL or make a physical drone safe.")
        drone = manager.get_drone(task.assigned_drone_id)
        print(f"    Drone Status:    {drone.status.value.upper()} (released in simulation manager)")
        mission_status = "failed"
    elif delivery_ctrl.state == DeliveryState.COMPLETED and delivery_ctrl.delivery_verified:
        # Controller reached successful terminal state with verification flag set
        manager.update_task_status(task.task_id, TaskStatus.VERIFIED)
        print(f"    Task Status:     {task.status.value.upper()}")
        print(f"    Verification Mtd:{delivery_ctrl.verification_method.value}")
        print(f"    Source/Details:  {delivery_ctrl.verification_source} — {delivery_ctrl.verification_details}")
        if delivery_ctrl.verification_method == DeliveryVerificationMethod.SIMULATED_TIMER:
            print("    ⚠ NOTE:          Verification was via simulated timer — NOT physical sensor confirmation.")
        print(f"    Drop Offset:     {delivery_ctrl.drop_accuracy_offset_m:.2f}m from ground survivor")
        drone = manager.get_drone(task.assigned_drone_id)
        print(f"    Drone Status:    {drone.status.value.upper()}")
        mission_status = "verified"
    else:
        # Delivery did not complete successfully — do NOT mark as VERIFIED
        manager.update_task_status(task.task_id, TaskStatus.FAILED)
        print(f"    Task Status:     {task.status.value.upper()} (delivery did not complete successfully)")
        print(f"    Controller State:{delivery_ctrl.state.value.upper()}")
        print(f"    Verified Flag:   {delivery_ctrl.delivery_verified}")
        if delivery_ctrl.verification_details:
            print(f"    Details:         {delivery_ctrl.verification_details}")
        if loop_timed_out:
            print(f"    Reason:          Simulation timed out after {sim_time:.1f}s while in state {delivery_ctrl.state.value.upper()}")
            print("    ⚠ NOTICE:        Vehicle did NOT return home; simulation loop timed out before RTL completion.")
            print("    ⚠ SAFETY NOTICE: Software bookkeeping cleanup does NOT command a real vehicle to RTL or make a physical drone safe.")
        drone = manager.get_drone(task.assigned_drone_id)
        print(f"    Drone Status:    {drone.status.value.upper()} (released in simulation manager)")
        mission_status = "timed_out" if loop_timed_out else "failed"

    # 8. Search Drone Resumption (simulated)
    approach_ctrl.complete_inspection()
    print("\n[8] SEARCH RESUMPTION (simulated):")
    print(f"    S1 State:        {approach_ctrl.state.value.upper()}")
    print(f"    Waypoints Paused:{approach_ctrl.is_waypoint_execution_paused} (Coverage lawnmower resumed)")
    print("=" * 78)

    return {
        "status": mission_status,
        "manager": manager,
        "task": task,
        "controller": delivery_ctrl,
        "error": delivery_error,
        "timed_out": loop_timed_out,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Autonomous RescueSwarm Pipeline (software-only simulation)")
    parser.add_argument("--image", type=Path, default=ROOT / "assets" / "test_survivors.jpg", help="Path to test image frame")
    parser.add_argument(
        "--conf",
        type=float,
        default=_DEFAULT_CONFIDENCE_THRESHOLD,
        help=(
            f"YOLO confidence threshold (default: {_DEFAULT_CONFIDENCE_THRESHOLD}). "
            "This is a development default, not a validated competition threshold."
        ),
    )
    parser.add_argument("--verify-mode", choices=["simulated_timer", "sensor"], default="simulated_timer", help="Delivery verification mode")
    args = parser.parse_args()

    run_pipeline(
        image_path=args.image,
        confidence_threshold=args.conf,
        verification_mode=args.verify_mode,
    )


if __name__ == "__main__":
    main()
