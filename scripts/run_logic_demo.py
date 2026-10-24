#!/usr/bin/env python3
"""Run a tiny software-only demonstration of the current core logic."""

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.allocator import assign_delivery_drone
from rescue_swarm_core.mission import MissionManager
from rescue_swarm_core.models import (
    Drone, DroneRole, DroneStatus, MissionState, RescueTask, TaskStatus
)


def main() -> None:
    config = json.loads((ROOT / "config" / "fleet.json").read_text())
    drones = [
        Drone(item["id"], DroneRole(item["role"]),
              status=DroneStatus.READY if item["role"] == "delivery" else DroneStatus.STANDBY,
              battery_fraction={"D1": 0.72, "D2": 0.91, "D3": 0.83}.get(item["id"], 1.0))
        for item in config["fleet"]
    ]

    mission = MissionManager()
    mission.transition_to(MissionState.INITIALIZING)
    mission.transition_to(MissionState.SEARCHING)

    task = RescueTask(
        task_id="survivor_001",
        confidence=0.94,
        image_x=0.51,
        image_y=0.47,
        detected_by="S2",
        status=TaskStatus.GEOLOCATED,
    )
    assigned = assign_delivery_drone(task, drones)

    mission.transition_to(MissionState.RESCUE_IN_PROGRESS)
    print("Software-only RescueSwarm logic demo")
    print(f"Fleet: {', '.join(d['id'] for d in config['fleet'])}")
    print(f"Mission state: {mission.state.value}")
    print(f"Task {task.task_id}: confidence={task.confidence:.2f}, detected_by={task.detected_by}")
    print(f"Assigned delivery drone: {assigned}")
    print("Note: no flight physics, camera inference, or real payload operation is simulated.")


if __name__ == "__main__":
    main()
