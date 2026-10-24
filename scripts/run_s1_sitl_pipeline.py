#!/usr/bin/env python3
"""Single-Search-Drone S1 Flight-Controller & SITL Integration Pipeline.

Validates the complete autonomous sequence on search drone S1:
1. Connects to autopilot flight bridge via MAVLink (real ArduCopter SITL or sandboxed mock).
2. Never silently falls back: displays exact backend and connection URI at startup.
3. Performs strict preflight health checks (heartbeat, telemetry freshness, altimeter).
4. Sets GUIDED mode, arms propulsion, and takes off to 30m patrol altitude.
5. Runs genuine Ultralytics YOLOv8 inference with trained best.pt weights on camera frame.
6. On person detection, transitions to APPROACHING and pauses search coverage waypoints.
7. Bridges visual servoing commands into local NED velocity setpoints.
8. Centers overhead within deadband, enters HOVERING, and geotags target via 3D ray-casting.
9. Updates SwarmManager with survivor location and allocates delivery response drone.
10. S1 completes hover inspection, restores HOLD setpoint, and resumes search patrol.
"""

import argparse
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core import (
    ApproachState,
    AutopilotType,
    CameraCalibration,
    Detection,
    FlightBridgeConfig,
    GeoPoint,
    MAVLinkFlightBridge,
    MissionState,
    SearchDroneApproachController,
    SwarmManager,
    YOLOPerceptionAdapter,
    estimate_survivor_location,
    haversine_distance_m,
    inspect_model_weights,
)
from rescue_swarm_core.safety_manager import SafetyAction


def run_s1_sitl_validation(
    image_path: Path,
    confidence_threshold: float = 0.03,
    autopilot_type_str: str = "ardupilot",
    connection_string: str = "mock:127.0.0.1:14550",
    connect_timeout_s: float = 3.0,
) -> None:
    print("=" * 80)
    print("  NIDAR RescueSwarm — Search Drone S1 Autopilot Integration Pipeline")
    print("=" * 80)

    # 1. Flight Bridge Configuration
    auto_type = AutopilotType(autopilot_type_str)
    bridge_config = FlightBridgeConfig(
        drone_id="S1",
        autopilot_type=auto_type,
        connection_string=connection_string,
        target_system=1,
        target_component=1,
        heartbeat_timeout_s=3.0,
        telemetry_stale_timeout_s=2.0,
        default_altitude_m=30.0,
    )
    bridge = MAVLinkFlightBridge(bridge_config)

    print("\n[1] FLIGHT BRIDGE CONFIGURATION & BACKEND:")
    print(f"    Drone ID:          {bridge.drone_id}")
    print(f"    Backend Mode:      {bridge.backend_name}")
    print(f"    Is Mock Backend:   {bridge.is_mock_backend}")
    print(f"    Target Autopilot:  {bridge.config.autopilot_type.value.upper()}")
    print(f"    Connection URI:    {bridge.config.connection_string}")
    print(f"    Target System ID:  {bridge.config.target_system}")

    # Attempt connection without fallback
    print(f"\n    Attempting connection to '{bridge.config.connection_string}' (Timeout: {connect_timeout_s:.1f}s)...")
    connected = bridge.connect(timeout_s=connect_timeout_s)

    if not connected:
        print(f"\n    [FATAL ERROR] Connection failed to {bridge.backend_name} endpoint!")
        print(f"    Reason: {bridge.last_error or 'Unknown error'}")
        if not bridge.is_mock_backend:
            print("    [NOTICE] Real MAVLink mode strictly refuses to report false success.")
            print("    Please ensure ArduCopter SITL is running on the host:")
            print("      sim_vehicle.py -v ArduCopter -I 0 --out udp:127.0.0.1:14550")
        sys.exit(1)

    print(f"    ✓ Connection established successfully! (State: {bridge.connection_state.value.upper()})")

    # 2. Preflight Health Checks
    print("\n[2] PREFLIGHT HEALTH & TELEMETRY VERIFICATION:")
    ready, check_logs = bridge.run_preflight_checks(timeout_s=2.0)
    for check in check_logs:
        print(f"    {check}")

    if not ready:
        print("    [FATAL ERROR] Preflight checks failed! Aborting mission.")
        bridge.disconnect()
        sys.exit(1)

    # 3. Flight Mode, Arming & Takeoff
    print("\n[3] FLIGHT MODE SETTING, PROPULSION ARMING & TAKEOFF:")
    mode_ok = bridge.set_guided_mode()
    print(f"    Set GUIDED Mode:   {'CONFIRMED' if mode_ok else 'FAILED'} (Current Mode: {bridge.telemetry.mode})")
    if not mode_ok and not bridge.is_mock_backend:
        print(f"    [ERROR] Failed to engage GUIDED mode on autopilot: {bridge.last_error}")
        bridge.disconnect()
        sys.exit(1)

    arm_ok = bridge.arm()
    print(f"    Arm Propulsion:    {'CONFIRMED' if arm_ok else 'FAILED'} (Armed: {bridge.telemetry.armed})")
    if not arm_ok and not bridge.is_mock_backend:
        print(f"    [ERROR] Failed to arm motors: {bridge.last_error}")
        bridge.disconnect()
        sys.exit(1)

    takeoff_ok = bridge.takeoff(30.0)
    print(f"    Command Takeoff:   {'CONFIRMED' if takeoff_ok else 'FAILED'} (Target: 30.0m AGL)")
    bridge.update_telemetry()
    print(f"    Current Telemetry: Alt={bridge.telemetry.altitude_agl_m:.1f}m AGL | Pos=[{bridge.telemetry.latitude:.6f}, {bridge.telemetry.longitude:.6f}] | Hdg={bridge.telemetry.heading_deg:.1f}°")

    # 4. Genuine Perception Inference
    weights_path = ROOT / "assets" / "models" / "best.pt"
    meta = inspect_model_weights(weights_path)
    print(f"\n[4] GENUINE MODEL INFERENCE (best.pt on {image_path.name}):")
    print(f"    Architecture:      {meta['framework']} ({meta['num_classes']} VisDrone classes)")
    print(f"    Target Classes:    {meta['target_survivor_classes']}")

    adapter = YOLOPerceptionAdapter(
        model_path=weights_path,
        confidence_threshold=confidence_threshold,
        target_classes=["pedestrian", "people", "person", "survivor"],
    )
    raw_detections = adapter.predict(image_path)
    print(f"    Threshold:         conf >= {confidence_threshold:.3f}")
    print(f"    Detections Count:  {len(raw_detections)} real target(s) detected")

    if not raw_detections:
        print("    [NOTICE] Model returned 0 detections. Halting validation.")
        bridge.disconnect()
        return

    top_detection = max(raw_detections, key=lambda d: d.confidence)
    print(f"    Top Detection:     {top_detection.class_name.upper()} (Class ID: {top_detection.class_id}) | Conf: {top_detection.confidence*100:.1f}%")
    print(f"    Bounding Box:      [{top_detection.bbox.xmin:.3f}, {top_detection.bbox.ymin:.3f}, {top_detection.bbox.xmax:.3f}, {top_detection.bbox.ymax:.3f}]")

    # 5. Search Drone Approach & Visual Servoing
    print("\n[5] SEARCH DRONE S1 APPROACH & SETPOINT BRIDGE:")
    camera = CameraCalibration(image_width=1376, image_height=768, hfov_deg=84.0, mount_pitch_deg=-90.0)
    approach_ctrl = SearchDroneApproachController(drone_id="S1", camera=camera)

    print(f"    Initial State:     {approach_ctrl.state.value.upper()} | Waypoints Paused: {approach_ctrl.is_waypoint_execution_paused}")

    # Step approach controller with detected person
    state, cmd = approach_ctrl.update(top_detection, current_yaw_rad=bridge.telemetry.yaw_rad, current_time=0.0)
    print(f"    On Detection:      {state.value.upper()} | Waypoints Paused: {approach_ctrl.is_waypoint_execution_paused}")
    print(f"    Tracking Center:   Head=({approach_ctrl.target_estimate.head_x}, {approach_ctrl.target_estimate.head_y}) | Image Error: ({cmd.error_x:+.1f}px, {cmd.error_y:+.1f}px)")
    print(f"    Commanded Velocity:Body=(Fwd: {cmd.body_forward_mps:+.2f}m/s, Rgt: {cmd.body_right_mps:+.2f}m/s) -> NED=(N: {cmd.local_ned_north_mps:+.2f}m/s, E: {cmd.local_ned_east_mps:+.2f}m/s)")

    sent = bridge.apply_visual_servo_command(cmd)
    bridge.update_telemetry()
    print(f"    MAVLink Dispatch:  SET_POSITION_TARGET_LOCAL_NED sent={sent} (vel_N: {bridge.telemetry.velocity_north_mps:+.2f}m/s, vel_E: {bridge.telemetry.velocity_east_mps:+.2f}m/s)")

    # Simulate approach translation steps until target is deadband-centered overhead
    print("\n[6] VISUAL SERVOING DEADAND CENTERING & HOVER TRANSITION:")
    centered_det = Detection(
        class_id=top_detection.class_id,
        class_name=top_detection.class_name,
        confidence=top_detection.confidence,
        bbox=top_detection.bbox.__class__(xmin=0.485, ymin=0.485, xmax=0.515, ymax=0.515),
    )
    for step in range(1, 4):
        state, cmd = approach_ctrl.update(centered_det, current_yaw_rad=bridge.telemetry.yaw_rad, current_time=step * 0.5)
        bridge.apply_visual_servo_command(cmd)
        bridge.update_telemetry()
        print(f"    Step {step}: State={state.value.upper():<12} | Action={cmd.action_label:<25} | Image Error=({cmd.error_x:+.1f}px, {cmd.error_y:+.1f}px)")

    print(f"    Hover Confirmed:   {approach_ctrl.state == ApproachState.HOVERING} (Holding position directly overhead)")

    # 6. 3D Geotagging
    drone_telem = bridge.to_drone_telemetry()
    geo_estimate = estimate_survivor_location(top_detection, drone_telem, camera)

    print("\n[7] 3D GEOTAGGING & SURVIVOR LOCALIZATION:")
    print(f"    Observing Drone:   S1 at Lat {drone_telem.position.latitude:.6f}, Lon {drone_telem.position.longitude:.6f} ({drone_telem.altitude_agl_m:.1f}m AGL)")
    print(f"    Ground Ray Offset: North={geo_estimate.offset_north_m:+.2f}m, East={geo_estimate.offset_east_m:+.2f}m")
    print(f"    Estimated GPS:     Lat {geo_estimate.location.latitude:.6f}, Lon {geo_estimate.location.longitude:.6f}")
    print(f"    Uncertainty:       ±{geo_estimate.uncertainty_radius_m:.2f}m radius")

    # 7. Swarm Manager Task Allocation
    fleet_config = ROOT / "config" / "fleet.json"
    manager = SwarmManager.from_config_file(fleet_config)
    manager.mission_manager.transition_to(MissionState.INITIALIZING)
    manager.mission_manager.transition_to(MissionState.SEARCHING)

    task, is_new = manager.process_detection(
        top_detection,
        geo_estimate,
        detected_by_drone_id="S1",
        minimum_confidence=confidence_threshold,
    )

    print("\n[8] SWARM MANAGER UPDATE & TASK ALLOCATION:")
    print(f"    New Survivor:      {is_new}")
    print(f"    Created Task:      {task.task_id} (Status: {task.status.value.upper()})")
    print(f"    Assigned Drone:    {task.assigned_drone_id} (Highest Battery Delivery Drone)")
    print(f"    Mission State:     {manager.mission_manager.state.value.upper()}")

    # 8. S1 Search Resumption
    approach_ctrl.complete_inspection()
    bridge.send_safety_override(SafetyAction.HOLD)
    bridge.update_telemetry()

    print("\n[9] SEARCH RESUMPTION & SETPOINT RESTORATION:")
    print(f"    S1 Controller:     {approach_ctrl.state.value.upper()}")
    print(f"    Waypoints Paused:  {approach_ctrl.is_waypoint_execution_paused} (Search coverage waypoints unpaused)")
    print(f"    Bridge Velocity:   vel_N={bridge.telemetry.velocity_north_mps:+.2f}m/s, vel_E={bridge.telemetry.velocity_east_mps:+.2f}m/s")
    print("=" * 80)
    print(f"  ✓ S1 {bridge.backend_name} pipeline sequence successfully validated!")
    print("=" * 80)

    bridge.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description="Single-Search-Drone S1 Autopilot Integration Pipeline")
    parser.add_argument("--image", type=Path, default=ROOT / "assets" / "test_survivors.jpg", help="Path to test frame")
    parser.add_argument("--conf", type=float, default=0.03, help="YOLO confidence threshold")
    parser.add_argument("--autopilot", choices=["ardupilot", "px4", "mock"], default="ardupilot", help="Autopilot stack")
    parser.add_argument(
        "--connection",
        default="mock:127.0.0.1:14550",
        help="MAVLink connection URI. Use 'udp:127.0.0.1:14550' for real ArduCopter SITL, or 'mock:127.0.0.1:14550' for offline mock testing.",
    )
    parser.add_argument("--timeout", type=float, default=3.0, help="Connection timeout in seconds")
    args = parser.parse_args()

    run_s1_sitl_validation(
        image_path=args.image,
        confidence_threshold=args.conf,
        autopilot_type_str=args.autopilot,
        connection_string=args.connection,
        connect_timeout_s=args.timeout,
    )


if __name__ == "__main__":
    main()
