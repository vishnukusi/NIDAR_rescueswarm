"""Small, transport-independent data models for the RescueSwarm core."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional


class DroneRole(str, Enum):
    SEARCH = "search"
    DELIVERY = "delivery"


class DroneStatus(str, Enum):
    OFFLINE = "offline"
    STANDBY = "standby"
    READY = "ready"
    BUSY = "busy"
    RETURNING = "returning"
    FAULT = "fault"


class TaskStatus(str, Enum):
    DETECTED = "detected"
    GEOLOCATED = "geolocated"
    ASSIGNED = "assigned"
    EN_ROUTE = "en_route"
    RELEASED = "released"
    VERIFIED = "verified"
    FAILED = "failed"


class MissionState(str, Enum):
    IDLE = "idle"
    INITIALIZING = "initializing"
    SEARCHING = "searching"
    RESCUE_IN_PROGRESS = "rescue_in_progress"
    RETURNING = "returning"
    COMPLETED = "completed"
    ABORTED = "aborted"


@dataclass(frozen=True)
class GeoPoint:
    latitude: float
    longitude: float
    altitude_m: float = 0.0

    def __post_init__(self) -> None:
        if not -90.0 <= self.latitude <= 90.0:
            raise ValueError("latitude must be in [-90, 90]")
        if not -180.0 <= self.longitude <= 180.0:
            raise ValueError("longitude must be in [-180, 180]")


@dataclass
class Drone:
    drone_id: str
    role: DroneRole
    status: DroneStatus = DroneStatus.STANDBY
    position: Optional[GeoPoint] = None
    battery_fraction: float = 1.0
    active_task_id: Optional[str] = None

    def __post_init__(self) -> None:
        if not self.drone_id:
            raise ValueError("drone_id cannot be empty")
        if not 0.0 <= self.battery_fraction <= 1.0:
            raise ValueError("battery_fraction must be between 0 and 1")


@dataclass
class RescueTask:
    task_id: str
    confidence: float
    image_x: float
    image_y: float
    detected_by: str
    status: TaskStatus = TaskStatus.DETECTED
    location: Optional[GeoPoint] = None
    assigned_drone_id: Optional[str] = None
    retry_of: Optional[str] = None

    def __post_init__(self) -> None:
        if not self.task_id:
            raise ValueError("task_id cannot be empty")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")
