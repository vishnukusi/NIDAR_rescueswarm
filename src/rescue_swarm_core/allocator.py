"""Simple deterministic allocator for assigning rescue tasks to delivery drones.

This is an initial policy, not the final swarm optimizer. It selects the eligible
delivery drone with the highest battery fraction, breaking ties by drone ID.
Later versions can account for distance, payload capacity, route risk, reserve
battery, and workload.
"""

from typing import Iterable, Optional

from .models import Drone, DroneRole, DroneStatus, RescueTask, TaskStatus


class AllocationError(ValueError):
    """Raised when a task cannot be assigned safely under the current policy."""


def assign_delivery_drone(
    task: RescueTask,
    drones: Iterable[Drone],
    minimum_battery_fraction: float = 0.30,
) -> str:
    if task.status not in (TaskStatus.GEOLOCATED, TaskStatus.DETECTED):
        raise AllocationError(f"task {task.task_id} cannot be assigned from status {task.status.value}")
    if task.assigned_drone_id is not None:
        raise AllocationError(f"task {task.task_id} already has an assigned drone")
    if not 0.0 <= minimum_battery_fraction <= 1.0:
        raise ValueError("minimum_battery_fraction must be between 0 and 1")

    eligible = [
        drone for drone in drones
        if drone.role == DroneRole.DELIVERY
        and drone.status in (DroneStatus.READY, DroneStatus.STANDBY)
        and drone.active_task_id is None
        and drone.battery_fraction >= minimum_battery_fraction
    ]
    if not eligible:
        raise AllocationError("no eligible delivery drone is available")

    chosen = sorted(eligible, key=lambda drone: (-drone.battery_fraction, drone.drone_id))[0]
    chosen.status = DroneStatus.BUSY
    chosen.active_task_id = task.task_id
    task.assigned_drone_id = chosen.drone_id
    task.status = TaskStatus.ASSIGNED
    return chosen.drone_id
