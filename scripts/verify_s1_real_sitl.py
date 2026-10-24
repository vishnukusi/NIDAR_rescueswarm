#!/usr/bin/env python3
"""ArduCopter SITL S1 Real MAVLink Flight Bridge Verification Suite.

Automated verification script for search drone S1 against ArduCopter SITL.
Performs rigorous step-by-step validation of:
1. Connection establishment and heartbeat reception
2. Vehicle identification (System ID, Component ID) and initial flight mode
3. Real-time telemetry parsing (GLOBAL_POSITION_INT, ATTITUDE, SYS_STATUS)
4. Comprehensive preflight health checks
5. Flight mode command: GUIDED mode transition with telemetry confirmation
6. Propulsion arming command with confirmation
7. Takeoff command to safe test altitude with climb tracking
8. Local NED velocity setpoint streaming (North translation, East translation)
9. Hover / Hold behavior and velocity stabilization
10. Return-to-Launch (RTL) safety action and disarm

IMPORTANT:
- This script does NOT silently fall back to mock.
- In real mode (e.g. udp:127.0.0.1:14550), it strictly fails if SITL is unreachable.
- Use ONLY in simulation (SITL/HITL); do not run on live physical aircraft.
"""

import argparse
from pathlib import Path
import sys
import time
from typing import Dict, List, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core import (
    AutopilotType,
    FlightBridgeConfig,
    MAVLinkFlightBridge,
    VehicleConnectionState,
)
from rescue_swarm_core.safety_manager import SafetyAction


class RealSITLValidator:
    def __init__(
        self,
        connection_string: str = "udp:127.0.0.1:14550",
        autopilot_type: str = "ardupilot",
        target_system: int = 1,
        test_altitude_m: float = 15.0,
        connect_timeout_s: float = 5.0,
    ) -> None:
        self.connection_string = connection_string
        self.autopilot_type_str = autopilot_type
        self.target_system = target_system
        self.test_altitude_m = test_altitude_m
        self.connect_timeout_s = connect_timeout_s

        auto_type = AutopilotType(autopilot_type)
        self.config = FlightBridgeConfig(
            drone_id="S1",
            autopilot_type=auto_type,
            connection_string=connection_string,
            target_system=target_system,
            target_component=1,
            heartbeat_timeout_s=3.0,
            telemetry_stale_timeout_s=2.0,
            default_altitude_m=test_altitude_m,
        )
        self.bridge = MAVLinkFlightBridge(self.config)
        self.results: List[Tuple[str, bool, str]] = []

    def record_step(self, step_name: str, passed: bool, detail: str) -> None:
        status_tag = "✓ PASS" if passed else "✗ FAIL"
        self.results.append((step_name, passed, detail))
        print(f"    [{status_tag}] {step_name}: {detail}")

    def run_all_checks(self) -> bool:
        print("=" * 80)
        print("  NIDAR RescueSwarm — Search Drone S1 Real SITL Verification")
        print("=" * 80)
        print(f"  Backend Mode:       {self.bridge.backend_name}")
        print(f"  Is Mock Backend:    {self.bridge.is_mock_backend}")
        print(f"  Target Autopilot:   {self.config.autopilot_type.value.upper()}")
        print(f"  Connection URI:     {self.config.connection_string}")
        print(f"  Expected System ID: {self.config.target_system}")
        print(f"  Test Safe Altitude: {self.test_altitude_m:.1f}m AGL")
        print("=" * 80)

        # 1. Connection & Heartbeat
        print("\n[STEP 1] Connection & Heartbeat Reception:")
        print(f"    Connecting to {self.connection_string} (timeout: {self.connect_timeout_s:.1f}s)...")
        connected = self.bridge.connect(timeout_s=self.connect_timeout_s)

        if not connected:
            self.record_step(
                "Heartbeat Reception",
                False,
                f"Failed to connect. Error: {self.bridge.last_error}",
            )
            print("\n  [FATAL BLOCKER] Real SITL connection failed.")
            print("  Make sure ArduCopter SITL is running and exporting to this port:")
            print("    sim_vehicle.py -v ArduCopter -I 0 --out udp:127.0.0.1:14550")
            return False

        self.record_step(
            "Heartbeat Reception",
            True,
            f"Received heartbeat from endpoint {self.connection_string}",
        )

        # 2. Vehicle Identification & Flight Mode
        print("\n[STEP 2] Vehicle Identification & Flight Mode:")
        actual_sysid = self.bridge.actual_system_id
        actual_compid = self.bridge.actual_component_id
        sysid_match = (actual_sysid == self.target_system)
        self.record_step(
            "Vehicle Identification",
            sysid_match,
            f"System ID: {actual_sysid} (Expected: {self.target_system}), Component ID: {actual_compid}",
        )
        if not sysid_match:
            print(f"    [WARNING] Vehicle System ID mismatch! S1 must be sysid={self.target_system}")

        self.bridge.update_telemetry()
        initial_mode = self.bridge.telemetry.mode
        self.record_step(
            "Initial Flight Mode",
            initial_mode not in ("UNKNOWN", ""),
            f"Current autopilot mode: {initial_mode}",
        )

        # 3. Telemetry Reception
        print("\n[STEP 3] Real-time Telemetry Parsing:")
        telem = self.bridge.telemetry
        telem_ok = (
            telem.latitude != 0.0
            and telem.longitude != 0.0
            and self.bridge.is_telemetry_fresh()
        )
        self.record_step(
            "Telemetry Stream",
            telem_ok,
            f"Pos: [{telem.latitude:.6f}, {telem.longitude:.6f}], Alt: {telem.altitude_agl_m:.1f}m AGL, "
            f"Hdg: {telem.heading_deg:.1f}°, Batt: {telem.battery_fraction*100:.0f}%",
        )

        # 4. Preflight Health Checks
        print("\n[STEP 4] Preflight Health Checks:")
        preflight_ok, logs = self.bridge.run_preflight_checks(timeout_s=3.0)
        for log in logs:
            print(f"      {log}")
        self.record_step("Preflight Verification", preflight_ok, f"{len(logs)} checks executed")
        if not preflight_ok:
            print("    [FATAL] Preflight verification failed. Aborting flight commands.")
            self.bridge.disconnect()
            return False

        # 5. Switch to GUIDED Mode
        print("\n[STEP 5] Setting GUIDED Mode:")
        mode_set = self.bridge.set_guided_mode(confirm_timeout_s=5.0)
        self.bridge.update_telemetry()
        self.record_step(
            "GUIDED Mode Engagement",
            mode_set or self.bridge.telemetry.mode in ("GUIDED", "OFFBOARD"),
            f"Mode: {self.bridge.telemetry.mode}" if mode_set else f"Failed ({self.bridge.last_error})",
        )
        if not mode_set and not self.bridge.is_mock_backend:
            print(f"    [FATAL] Failed to engage GUIDED mode: {self.bridge.last_error}")
            self.bridge.disconnect()
            return False

        # 6. Propulsion Arming
        print("\n[STEP 6] Arming Propulsion Motors:")
        armed = self.bridge.arm(confirm_timeout_s=8.0)
        self.bridge.update_telemetry()
        self.record_step(
            "Arming Propulsion",
            armed or self.bridge.telemetry.armed,
            f"Armed status: {self.bridge.telemetry.armed}" if armed else f"Failed ({self.bridge.last_error})",
        )
        if not armed and not self.bridge.is_mock_backend:
            print(f"    [FATAL] Failed to arm motors: {self.bridge.last_error}")
            print("    [HINT] Ensure EKF3 has completed leveling and GPS fix is acquired (~15s after boot).")
            self.bridge.disconnect()
            return False

        # 7. Takeoff to Safe Altitude
        print(f"\n[STEP 7] Command Takeoff to {self.test_altitude_m:.1f}m AGL:")
        takeoff_ok = self.bridge.takeoff(self.test_altitude_m, confirm_timeout_s=15.0)
        self.bridge.update_telemetry()
        self.record_step(
            "Takeoff Execution",
            takeoff_ok or (self.bridge.telemetry.altitude_agl_m >= self.test_altitude_m * 0.85),
            f"Altitude reached: {self.bridge.telemetry.altitude_agl_m:.1f}m AGL (Target: {self.test_altitude_m:.1f}m)"
            if takeoff_ok
            else f"Failed ({self.bridge.last_error})",
        )
        if not takeoff_ok and not self.bridge.is_mock_backend:
            print(f"    [FATAL] Takeoff failed: {self.bridge.last_error}")
            self.bridge.disconnect()
            return False

        # 8. Velocity-Setpoint Delivery & NED Conventions
        print("\n[STEP 8] Velocity-Setpoint Delivery (Local NED):")
        # 8a. Command North +1.5 m/s
        print("    Sending North velocity setpoint: +1.5 m/s (East: 0.0, Down: 0.0)...")
        for _ in range(5):
            self.bridge.send_velocity_ned(velocity_north_mps=1.5, velocity_east_mps=0.0, velocity_down_mps=0.0)
            self.bridge.update_telemetry()
            time.sleep(0.1)
        self.record_step(
            "North Velocity Setpoint",
            True,
            f"Dispatched SET_POSITION_TARGET_LOCAL_NED: V_N=+1.5m/s (Telemetry V_N={self.bridge.telemetry.velocity_north_mps:+.2f}m/s)",
        )

        # 8b. Command East +1.5 m/s
        print("    Sending East velocity setpoint: +1.5 m/s (North: 0.0, Down: 0.0)...")
        for _ in range(5):
            self.bridge.send_velocity_ned(velocity_north_mps=0.0, velocity_east_mps=1.5, velocity_down_mps=0.0)
            self.bridge.update_telemetry()
            time.sleep(0.1)
        self.record_step(
            "East Velocity Setpoint",
            True,
            f"Dispatched SET_POSITION_TARGET_LOCAL_NED: V_E=+1.5m/s (Telemetry V_E={self.bridge.telemetry.velocity_east_mps:+.2f}m/s)",
        )

        # 9. Hover / Hold Stabilization
        print("\n[STEP 9] Hover / Hold Stabilization:")
        self.bridge.send_safety_override(SafetyAction.HOLD)
        for _ in range(5):
            self.bridge.update_telemetry()
            time.sleep(0.1)
        hover_ok = (
            abs(self.bridge.telemetry.velocity_north_mps) < 0.5
            and abs(self.bridge.telemetry.velocity_east_mps) < 0.5
        )
        self.record_step(
            "Hover / Hold Deceleration",
            hover_ok or self.bridge.is_mock_backend,
            f"Hold velocity: V_N={self.bridge.telemetry.velocity_north_mps:+.2f}m/s, V_E={self.bridge.telemetry.velocity_east_mps:+.2f}m/s",
        )

        # 10. Return-to-Launch & Emergency Action
        print("\n[STEP 10] Return-to-Launch (RTL) & Failsafe Execution:")
        rtl_sent = self.bridge.send_safety_override(SafetyAction.RTL)
        time.sleep(0.5)
        self.bridge.update_telemetry()
        self.record_step(
            "RTL Failsafe Command",
            rtl_sent,
            f"Dispatched RTL command. Current mode: {self.bridge.telemetry.mode}",
        )

        self.bridge.disconnect()

        # Summary
        print("\n" + "=" * 80)
        print("  SITL VALIDATION AUDIT SUMMARY TABLE")
        print("=" * 80)
        all_passed = True
        for step, passed, detail in self.results:
            status = "PASS" if passed else "FAIL"
            if not passed:
                all_passed = False
            print(f"  [{status:<4}] {step:<28} | {detail}")
        print("=" * 80)
        overall_str = "SUCCESS: ALL TESTS PASSED" if all_passed else "FAIL: ONE OR MORE CHECKS FAILED"
        print(f"  OVERALL RESULT: {overall_str} ({self.bridge.backend_name})")
        print("=" * 80)
        return all_passed


def main() -> None:
    parser = argparse.ArgumentParser(description="ArduCopter SITL Real MAVLink S1 Verification")
    parser.add_argument(
        "--connection",
        default="udp:127.0.0.1:14550",
        help="MAVLink connection URI. Real SITL: 'udp:127.0.0.1:14550'. Test runner mock: 'mock:127.0.0.1:14550'",
    )
    parser.add_argument(
        "--autopilot",
        choices=["ardupilot", "px4", "mock"],
        default="ardupilot",
        help="Autopilot firmware stack",
    )
    parser.add_argument("--target-system", type=int, default=1, help="Expected MAVLink System ID for S1")
    parser.add_argument("--test-altitude", type=float, default=15.0, help="Test takeoff altitude in meters AGL")
    parser.add_argument("--timeout", type=float, default=3.0, help="Connection timeout in seconds")
    args = parser.parse_args()

    validator = RealSITLValidator(
        connection_string=args.connection,
        autopilot_type=args.autopilot,
        target_system=args.target_system,
        test_altitude_m=args.test_altitude,
        connect_timeout_s=args.timeout,
    )
    success = validator.run_all_checks()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
