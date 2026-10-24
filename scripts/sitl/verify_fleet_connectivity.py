#!/usr/bin/env python3
"""Preflight connectivity and identity verification for the 6-drone PX4 fleet.

Verifies:
1. All 6 vehicle endpoints are reachable.
2. Each vehicle responds with its distinct expected system_id (1 through 6).
3. Telemetry is genuinely fresh (max age <= 2.0s).
4. No cross-talk occurs across distinct UDP ports.
5. Produces machine-readable JSON status output.
"""

import argparse
import json
import os
import sys
import time
from typing import Any, Dict, List, Optional

# Ensure src is on path
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SRC_DIR = os.path.join(REPO_ROOT, "src")
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from rescue_swarm_core.flight_bridge import (
    AutopilotType,
    FlightBridgeConfig,
    MAVLinkFlightBridge,
)


def load_fleet_config(config_path: str) -> Dict[str, Any]:
    with open(config_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data.get("fleet"), list):
        return {d["id"]: d for d in data["fleet"] if "id" in d}
    elif isinstance(data.get("drones"), dict):
        return data["drones"]
    return {}


def verify_fleet(
    config_path: str = "config/fleet.json",
    timeout_s: float = 3.0,
    use_mock: bool = False,
) -> Dict[str, Any]:
    drones = load_fleet_config(config_path)
    expected_ids = ["S1", "S2", "S3", "D1", "D2", "D3"]

    report = {
        "timestamp": time.time(),
        "all_passed": True,
        "is_mock": use_mock,
        "vehicles": {},
        "summary": {
            "total_vehicles": len(expected_ids),
            "connected_count": 0,
            "failed_count": 0,
        },
    }

    print("=" * 70)
    print("  NIDAR RescueSwarm — 6-Drone Fleet Connectivity & Identity Verification")
    print(f"  Mode: {'MOCK EMULATION' if use_mock else 'REAL PX4 MAVLINK NETWORK'}")
    print("=" * 70)

    for drone_id in expected_ids:
        cfg = drones.get(drone_id, {})
        sysid = int(cfg.get("system_id", 1))
        port = int(cfg.get("mavlink_port", 14550))
        role = cfg.get("role", "unknown")

        print(f"  Checking {drone_id} ({role.upper()}) [SysID {sysid}, Port {port}]... ", end="", flush=True)

        if use_mock:
            # Deterministic software mock verification
            v_report = {
                "drone_id": drone_id,
                "role": role,
                "expected_sysid": sysid,
                "observed_sysid": sysid,
                "port": port,
                "connected": True,
                "telemetry_fresh": True,
                "battery": 1.0,
                "passed": True,
                "error": None,
            }
            report["vehicles"][drone_id] = v_report
            report["summary"]["connected_count"] += 1
            print("PASS (Mock)")
            continue

        # Real MAVLink UDP connection check
        uri = f"udp:127.0.0.1:{port}"
        fb_cfg = FlightBridgeConfig(
            drone_id=drone_id,
            autopilot_type=AutopilotType.PX4,
            connection_string=uri,
            target_system=sysid,
            target_component=1,
            heartbeat_timeout_s=timeout_s,
        )
        bridge = MAVLinkFlightBridge(fb_cfg)
        connected = bridge.connect(timeout_s=timeout_s)

        if connected:
            bridge.update_telemetry()
            telem = bridge.telemetry
            fresh = bridge.is_telemetry_fresh()
            observed_sysid = bridge.actual_system_id

            preflight_ok, preflight_msgs = bridge.run_preflight_checks(timeout_s=timeout_s)

            # Check for cross-talk: observed sysid must match expected
            sysid_match = (observed_sysid == sysid) if observed_sysid is not None else False
            v_ok = fresh and sysid_match and preflight_ok

            err_reasons = []
            if not sysid_match:
                err_reasons.append(f"SysID mismatch (expected {sysid}, got {observed_sysid})")
            if not fresh:
                err_reasons.append("Telemetry stale")
            if not preflight_ok:
                err_reasons.append("Preflight checks failed")
            err_msg = "; ".join(err_reasons) if err_reasons else None

            v_report = {
                "drone_id": drone_id,
                "role": role,
                "expected_sysid": sysid,
                "observed_sysid": observed_sysid,
                "port": port,
                "connected": True,
                "telemetry_fresh": fresh,
                "battery": telem.battery_fraction,
                "mode": telem.mode,
                "preflight_passed": preflight_ok,
                "preflight_details": preflight_msgs,
                "passed": v_ok,
                "error": err_msg,
            }
            if v_ok:
                report["summary"]["connected_count"] += 1
                print(f"PASS (SysID {observed_sysid}, {telem.mode}, Batt {telem.battery_fraction*100:.0f}%)")
            else:
                report["all_passed"] = False
                report["summary"]["failed_count"] += 1
                print(f"FAIL ({err_msg})")
        else:
            report["all_passed"] = False
            report["summary"]["failed_count"] += 1
            v_report = {
                "drone_id": drone_id,
                "role": role,
                "expected_sysid": sysid,
                "port": port,
                "connected": False,
                "passed": False,
                "error": f"Connection timeout on {uri}",
            }
            print("TIMEOUT (Unreachable)")

        bridge.disconnect()
        report["vehicles"][drone_id] = v_report

    print("-" * 70)
    print(f"  Summary: {report['summary']['connected_count']}/{report['summary']['total_vehicles']} Connected | Result: {'PASSED' if report['all_passed'] else 'FAILED'}")
    print("=" * 70)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify 6-drone PX4 fleet connectivity")
    parser.add_argument("--config", default="config/fleet.json", help="Path to fleet JSON config")
    parser.add_argument("--timeout", type=float, default=2.0, help="Per-vehicle connection timeout (seconds)")
    parser.add_argument("--mock", action="store_true", help="Run with mock loopback for CI/unit testing")
    parser.add_argument("--json-out", default="", help="Path to output JSON results")
    args = parser.parse_args()

    res = verify_fleet(config_path=args.config, timeout_s=args.timeout, use_mock=args.mock)

    if args.json_out:
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2)

    return 0 if res["all_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
