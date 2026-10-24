#!/usr/bin/env python3
"""Full-mission autonomous swarm simulation for NIDAR RescueSwarm (software-only).

This script runs entirely in software — no PX4 SITL, Gazebo, ROS 2, or
physical hardware is involved. All flight profiles, detections, and delivery
sequences are simulated state-machine updates.

Demonstrates:
1. Operational search area definition & 3-drone coverage partition (S1, S2, S3).
2. Autonomous sweep flight paths with ground swath footprint calculations.
3. Multi-drone synthetic test detections, 3D ray-plane localization, and spatial
   deduplication. Detections are test data — NOT live YOLO inference.
4. Deterministic delivery allocation (D1, D2, D3).
5. Simulated delivery mission flight profile (transit, 8m drop descent, payload
   release, verification, RTL) — no physical vehicle motion.
6. Continuous simulated safety monitoring (geofence, battery failsafe, fleet
   anti-collision separation, and watchdog comms loss).
"""

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Dict, Optional, Union

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core import (
    MissionScenarioConfig,
    MissionSimulationReport,
    SimulatedMissionRunner,
    create_scenario,
)


def run_full_mission_simulation(
    scenario: Optional[Union[str, MissionScenarioConfig]] = None,
    print_summary: bool = True,
) -> Dict[str, Any]:
    """Execute the full multi-drone software mission simulation.

    Args:
        scenario: Preset scenario name (e.g., 'nominal', 'single_delivery',
            'no_survivors', 'failed_delivery_retry') or a custom
            MissionScenarioConfig instance. When None, defaults to 'single_delivery'
            for backward compatibility with existing tests.
        print_summary: If True, prints a human-readable telemetry report to stdout.

    Returns:
        Dict[str, Any] containing backward-compatible keys:
            - "status": Overall mission status ("verified", "completed", "failed", "timed_out")
            - "manager": SwarmManager instance
            - "task": Primary RescueTask instance
            - "controller": Primary DeliveryMissionController instance
            - "error": Exception instance if an error occurred, else None
            - "timed_out": True if simulation reached max_duration_s before terminal state
            - "summary": Fleet summary dictionary
            - "report": MissionSimulationReport instance
    """
    if scenario is None:
        cfg = create_scenario("single_delivery")
    elif isinstance(scenario, str):
        cfg = create_scenario(scenario)
    elif isinstance(scenario, MissionScenarioConfig):
        cfg = scenario
    else:
        raise ValueError(f"Invalid scenario parameter: {type(scenario)}")

    runner = SimulatedMissionRunner(cfg)
    compat_result, report = runner.run()

    if print_summary:
        print(report.format_summary())

    return compat_result


def main() -> None:
    parser = argparse.ArgumentParser(
        description="NIDAR RescueSwarm — Autonomous Fleet Mission Simulation (software-only)"
    )
    parser.add_argument(
        "--scenario",
        default="nominal",
        help=(
            "Preset scenario name: 'nominal', 'single_delivery', 'no_survivors', "
            "'duplicate_observations', 'failed_delivery_retry', 'delivery_drone_unavailable', "
            "'search_drone_comms_loss', 'low_battery_allocation', 'mission_timeout', "
            "'controller_exception' (default: 'nominal')"
        ),
    )
    parser.add_argument(
        "--time-limit",
        type=float,
        default=None,
        help="Optional override for mission duration in seconds",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=42,
        help="Simulation random seed",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Output JSON summary to stdout",
    )
    args = parser.parse_args()

    cfg = create_scenario(args.scenario)
    if args.time_limit is not None:
        cfg.max_duration_s = args.time_limit
    cfg.seed = args.seed

    result = run_full_mission_simulation(scenario=cfg, print_summary=not args.json)

    if args.json:
        report: MissionSimulationReport = result["report"]
        print(json.dumps(report.to_dict(), indent=2))


if __name__ == "__main__":
    main()
