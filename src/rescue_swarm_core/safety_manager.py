"""Safety and failsafe management for NIDAR RescueSwarm.

Monitors vehicle telemetry against operational safety constraints:
1. Geofence boundaries (lateral polygon / altitude AGL limits).
2. Battery threshold failsafes (warning -> RTL -> emergency land).
3. Swarm anti-collision proximity monitoring between all active drones.
4. Heartbeat telemetry loss and link watchdog timeouts.
"""

from dataclasses import dataclass, field
from enum import Enum
import time
from typing import Any, Dict, List, Optional, Tuple

from .coverage_planner import SearchArea
from .localization import DroneTelemetry, haversine_distance_m
from .models import Drone, DroneRole, DroneStatus, GeoPoint


class SafetyLevel(str, Enum):
    NORMAL = "normal"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


class SafetyAction(str, Enum):
    NONE = "none"
    WARN = "warn"
    HOLD = "hold"
    RTL = "return_to_launch"
    EMERGENCY_LAND = "emergency_land"


@dataclass(frozen=True)
class SafetyAlert:
    """Individual safety alert event raised by the safety manager."""
    level: SafetyLevel
    action: SafetyAction
    drone_id: str
    message: str
    timestamp: float = field(default_factory=time.time)


@dataclass
class SafetyIncident:
    """Tracked safety incident record representing a distinct lifecycle condition."""
    incident_id: str
    drone_id: str
    condition_type: str
    level: SafetyLevel
    action: SafetyAction
    message: str
    start_time: float
    last_seen_time: float
    end_time: Optional[float] = None
    occurrence_count: int = 1
    resolved: bool = False
    severity_escalated: bool = False
    escalations: List[Dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "incident_id": self.incident_id,
            "drone_id": self.drone_id,
            "condition_type": self.condition_type,
            "level": self.level.value,
            "action": self.action.value,
            "message": self.message,
            "start_time": self.start_time,
            "last_seen_time": self.last_seen_time,
            "end_time": self.end_time,
            "occurrence_count": self.occurrence_count,
            "resolved": self.resolved,
            "severity_escalated": self.severity_escalated,
            "escalations": self.escalations,
        }


@dataclass
class GeofenceConfig:
    """Geofence boundary and altitude constraints."""
    boundary: SearchArea
    min_altitude_agl_m: float = 3.0
    max_altitude_agl_m: float = 60.0
    margin_m: float = 2.0


@dataclass
class BatteryFailsafeConfig:
    """Battery state thresholds."""
    warning_fraction: float = 0.30
    rtl_fraction: float = 0.20
    land_critical_fraction: float = 0.10

    def __post_init__(self) -> None:
        if not (0.0 < self.land_critical_fraction < self.rtl_fraction < self.warning_fraction <= 1.0):
            raise ValueError("Battery thresholds must satisfy 0 < land_critical < rtl < warning <= 1.0")


class SafetyManager:
    """Real-time safety watchdog evaluating multi-drone fleet telemetry with incident lifecycle management."""

    SEVERITY_ORDER = {
        SafetyLevel.NORMAL: 0,
        SafetyLevel.WARNING: 1,
        SafetyLevel.CRITICAL: 2,
        SafetyLevel.EMERGENCY: 3,
    }

    def __init__(
        self,
        geofence: Optional[GeofenceConfig] = None,
        battery_config: Optional[BatteryFailsafeConfig] = None,
        min_separation_m: float = 6.0,
        heartbeat_timeout_s: float = 5.0,
        deduplicate_alerts: bool = True,
    ) -> None:
        self.geofence = geofence
        self.battery_config = battery_config or BatteryFailsafeConfig()
        self.min_separation_m = min_separation_m
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.deduplicate_alerts = deduplicate_alerts
        self.alert_history: List[SafetyAlert] = []
        self._last_heartbeat: Dict[str, float] = {}
        self.incidents: List[SafetyIncident] = []
        self.active_incidents: Dict[Tuple[str, str], SafetyIncident] = {}
        self._incident_counter: int = 0
        self.total_alerts_evaluated: int = 0

    def _record_condition(
        self,
        drone_id: str,
        condition_type: str,
        level: SafetyLevel,
        action: SafetyAction,
        message: str,
        timestamp: float,
    ) -> SafetyAlert:
        """Register or update an active safety incident and return the SafetyAlert for this tick."""
        self.total_alerts_evaluated += 1
        key = (drone_id, condition_type)
        alert = SafetyAlert(level=level, action=action, drone_id=drone_id, message=message, timestamp=timestamp)

        existing = self.active_incidents.get(key)
        if existing is None:
            # Brand new incident
            self._incident_counter += 1
            inc_id = f"INC-{self._incident_counter:03d}"
            incident = SafetyIncident(
                incident_id=inc_id,
                drone_id=drone_id,
                condition_type=condition_type,
                level=level,
                action=action,
                message=message,
                start_time=timestamp,
                last_seen_time=timestamp,
                occurrence_count=1,
                resolved=False,
            )
            self.incidents.append(incident)
            self.active_incidents[key] = incident
            self.alert_history.append(alert)
        else:
            # Persistent condition — update active incident record
            existing.last_seen_time = timestamp
            existing.occurrence_count += 1

            # Check for severity escalation
            curr_rank = self.SEVERITY_ORDER.get(level, 0)
            prev_rank = self.SEVERITY_ORDER.get(existing.level, 0)
            if curr_rank > prev_rank:
                existing.severity_escalated = True
                old_lvl = existing.level
                old_act = existing.action
                existing.level = level
                existing.action = action
                existing.message = message
                existing.escalations.append({
                    "time": timestamp,
                    "from_level": old_lvl.value,
                    "to_level": level.value,
                    "action": action.value,
                    "message": message,
                })
                escalated_alert = SafetyAlert(
                    level=level,
                    action=action,
                    drone_id=drone_id,
                    message=f"[ESCALATION] {message}",
                    timestamp=timestamp,
                )
                self.alert_history.append(escalated_alert)
            elif not self.deduplicate_alerts:
                self.alert_history.append(alert)

        return alert

    def _resolve_condition(self, drone_id: str, condition_type: str, timestamp: float) -> Optional[SafetyIncident]:
        """Mark an active condition as resolved when telemetry returns to nominal parameters."""
        key = (drone_id, condition_type)
        incident = self.active_incidents.pop(key, None)
        if incident is not None:
            incident.resolved = True
            incident.end_time = timestamp
        return incident

    def record_heartbeat(self, drone_id: str, current_time: Optional[float] = None) -> bool:
        """Record telemetry heartbeat receipt timestamp and resolve any active comms loss incident.

        Guards against false recovery: telemetry only counts as fresh when a genuinely newer
        valid update timestamp is received (now > last_heartbeat). Stale, duplicate, or retro-dated
        timestamps do not update the heartbeat or resolve an active timeout incident.
        """
        now = current_time if current_time is not None else time.time()
        last = self._last_heartbeat.get(drone_id)
        if last is not None and now <= last:
            return False
        self._last_heartbeat[drone_id] = now
        self._resolve_condition(drone_id, "telemetry_timeout", now)
        return True

    def check_telemetry(
        self,
        telemetry: DroneTelemetry,
        battery_fraction: float,
        current_time: Optional[float] = None,
    ) -> List[SafetyAlert]:
        """Perform full safety inspection for an individual drone's telemetry state."""
        now = current_time if current_time is not None else time.time()
        self.record_heartbeat(telemetry.drone_id, now)
        alerts: List[SafetyAlert] = []

        # 1. Battery Failsafe Checks
        if battery_fraction <= self.battery_config.land_critical_fraction:
            alerts.append(self._record_condition(
                drone_id=telemetry.drone_id,
                condition_type="battery_failsafe",
                level=SafetyLevel.EMERGENCY,
                action=SafetyAction.EMERGENCY_LAND,
                message=f"Critically low battery ({battery_fraction*100:.1f}% <= {self.battery_config.land_critical_fraction*100:.0f}%). Immediate land required.",
                timestamp=now,
            ))
        elif battery_fraction <= self.battery_config.rtl_fraction:
            alerts.append(self._record_condition(
                drone_id=telemetry.drone_id,
                condition_type="battery_failsafe",
                level=SafetyLevel.CRITICAL,
                action=SafetyAction.RTL,
                message=f"Low battery RTL threshold triggered ({battery_fraction*100:.1f}% <= {self.battery_config.rtl_fraction*100:.0f}%).",
                timestamp=now,
            ))
        elif battery_fraction <= self.battery_config.warning_fraction:
            alerts.append(self._record_condition(
                drone_id=telemetry.drone_id,
                condition_type="battery_failsafe",
                level=SafetyLevel.WARNING,
                action=SafetyAction.WARN,
                message=f"Battery low warning ({battery_fraction*100:.1f}%).",
                timestamp=now,
            ))
        else:
            self._resolve_condition(telemetry.drone_id, "battery_failsafe", now)

        # 2. Geofence Boundary Checks
        if self.geofence:
            # Altitude limits
            if telemetry.altitude_agl_m > self.geofence.max_altitude_agl_m:
                alerts.append(self._record_condition(
                    drone_id=telemetry.drone_id,
                    condition_type="geofence_ceiling",
                    level=SafetyLevel.CRITICAL,
                    action=SafetyAction.HOLD,
                    message=f"Ceiling breach: altitude {telemetry.altitude_agl_m:.1f}m > max {self.geofence.max_altitude_agl_m:.1f}m.",
                    timestamp=now,
                ))
            else:
                self._resolve_condition(telemetry.drone_id, "geofence_ceiling", now)

            if telemetry.altitude_agl_m < self.geofence.min_altitude_agl_m:
                alerts.append(self._record_condition(
                    drone_id=telemetry.drone_id,
                    condition_type="geofence_floor",
                    level=SafetyLevel.WARNING,
                    action=SafetyAction.WARN,
                    message=f"Floor warning: altitude {telemetry.altitude_agl_m:.1f}m < min {self.geofence.min_altitude_agl_m:.1f}m.",
                    timestamp=now,
                ))
            else:
                self._resolve_condition(telemetry.drone_id, "geofence_floor", now)

            # Lateral boundary containment
            if not self.geofence.boundary.contains(telemetry.position):
                alerts.append(self._record_condition(
                    drone_id=telemetry.drone_id,
                    condition_type="geofence_lateral",
                    level=SafetyLevel.EMERGENCY,
                    action=SafetyAction.RTL,
                    message=f"Lateral geofence breached at Lat {telemetry.position.latitude:.6f}, Lon {telemetry.position.longitude:.6f}.",
                    timestamp=now,
                ))
            else:
                self._resolve_condition(telemetry.drone_id, "geofence_lateral", now)

        return alerts

    def check_fleet_proximity(
        self,
        telemetries: Dict[str, DroneTelemetry],
        current_time: Optional[float] = None,
    ) -> List[SafetyAlert]:
        """Verify minimum separation distances between all active drones in the fleet."""
        now = current_time if current_time is not None else time.time()
        alerts: List[SafetyAlert] = []
        drone_ids = list(telemetries.keys())
        conflicting_drones = set()

        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                d1 = drone_ids[i]
                d2 = drone_ids[j]
                t1 = telemetries[d1]
                t2 = telemetries[d2]

                horizontal_dist = haversine_distance_m(t1.position, t2.position)
                vertical_dist = abs(t1.altitude_agl_m - t2.altitude_agl_m)

                # 3D spatial separation
                total_separation = (horizontal_dist**2 + vertical_dist**2) ** 0.5

                if total_separation < self.min_separation_m:
                    conflicting_drones.add(d1)
                    conflicting_drones.add(d2)
                    msg = (
                        f"Proximity conflict between {d1} and {d2}: separation {total_separation:.1f}m "
                        f"< minimum {self.min_separation_m:.1f}m."
                    )
                    alerts.append(self._record_condition(
                        drone_id=d1,
                        condition_type="proximity_conflict",
                        level=SafetyLevel.CRITICAL,
                        action=SafetyAction.HOLD,
                        message=msg,
                        timestamp=now,
                    ))
                    alerts.append(self._record_condition(
                        drone_id=d2,
                        condition_type="proximity_conflict",
                        level=SafetyLevel.CRITICAL,
                        action=SafetyAction.HOLD,
                        message=msg,
                        timestamp=now,
                    ))

        # Clear resolved proximity incidents for drones not in conflict
        for d_id in drone_ids:
            if d_id not in conflicting_drones:
                self._resolve_condition(d_id, "proximity_conflict", now)

        return alerts

    def check_telemetry_timeouts(
        self,
        drone_ids: List[str],
        current_time: Optional[float] = None,
    ) -> List[SafetyAlert]:
        """Detect communication loss watchdog timeouts with incident lifecycle tracking."""
        now = current_time if current_time is not None else time.time()
        alerts: List[SafetyAlert] = []

        for drone_id in drone_ids:
            last = self._last_heartbeat.get(drone_id)
            if last is not None and (now - last) > self.heartbeat_timeout_s:
                alerts.append(self._record_condition(
                    drone_id=drone_id,
                    condition_type="telemetry_timeout",
                    level=SafetyLevel.CRITICAL,
                    action=SafetyAction.RTL,
                    message=f"Telemetry lost for {drone_id} (silence: {now - last:.1f}s > {self.heartbeat_timeout_s:.1f}s).",
                    timestamp=now,
                ))

        return alerts

    def get_all_incidents(self) -> List[SafetyIncident]:
        """Return all logged safety incidents across the mission."""
        return list(self.incidents)

    def get_active_incidents(self) -> List[SafetyIncident]:
        """Return incidents currently unresolved."""
        return list(self.active_incidents.values())
