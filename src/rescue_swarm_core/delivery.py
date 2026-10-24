"""Delivery mission execution and autonomous payload release verification for NIDAR RescueSwarm.

Models the end-to-end flight profile and payload drop sequence for delivery drones (D1, D2, D3):
1. Takeoff and transit cruise at designated altitude (e.g., 25m AGL).
2. Approach deceleration and descent to safe payload drop altitude (e.g., 8m AGL).
3. Payload release actuator trigger.
4. Optical / load-sensor drop verification.
5. Post-delivery ascent and return-to-base (RTL).
"""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple

from .localization import haversine_distance_m
from .models import GeoPoint, RescueTask, TaskStatus


class DeliveryState(str, Enum):
    STANDBY = "standby"
    TAKEOFF = "takeoff"
    EN_ROUTE = "en_route"
    APPROACH = "approach"
    DESCENT_TO_DROP = "descent_to_drop"
    PAYLOAD_RELEASE = "payload_release"
    VERIFY_DROP = "verify_drop"
    ASCENT = "ascent"
    RETURNING_HOME = "returning_home"
    COMPLETED = "completed"
    ABORTED = "aborted"


class DeliveryVerificationMethod(str, Enum):
    """Method used to confirm payload delivery."""
    SIMULATED_TIMER = "simulated_timer"                 # Demonstration only; elapsed timer without physical sensor confirmation
    ACTUATOR_TELEMETRY = "actuator_telemetry"           # Confirmed servo / gripper pin open event
    GROUND_CONTACT_SENSOR = "ground_contact_sensor"     # Payload lanyard / microswitch ground impact signal
    OPTICAL_CONFIRMATION = "optical_confirmation"       # Downward camera confirms payload stationary on ground
    SIMULATION_EVENT = "simulation_event"               # Gazebo contact / PX4 gripper event topic


@dataclass
class DeliveryProfile:
    """Operational flight and drop parameters for survival kit deliveries."""
    transit_altitude_m: float = 25.0
    release_altitude_m: float = 8.0
    horizontal_speed_mps: float = 10.0
    vertical_speed_mps: float = 2.0
    drop_tolerance_radius_m: float = 4.0
    release_hover_time_s: float = 2.0
    verification_time_s: float = 3.0
    verification_timeout_s: float = 15.0  # Max wait time for sensor/simulation verification event

    def __post_init__(self) -> None:
        if self.release_altitude_m >= self.transit_altitude_m:
            raise ValueError("release_altitude_m must be strictly lower than transit_altitude_m")
        if self.release_altitude_m <= 0.0:
            raise ValueError("release_altitude_m must be positive")


class DeliveryMissionController:
    """Controls the discrete flight profile and autonomous drop verification for a single delivery task."""

    def __init__(
        self,
        drone_id: str,
        task: RescueTask,
        home_location: GeoPoint,
        profile: Optional[DeliveryProfile] = None,
        verification_method: DeliveryVerificationMethod = DeliveryVerificationMethod.SIMULATED_TIMER,
    ) -> None:
        if not task.location:
            raise ValueError(f"Task {task.task_id} must have a valid target location for delivery")
        self.drone_id = drone_id
        self.task = task
        self.home_location = home_location
        self.target_location = task.location
        self.profile = profile or DeliveryProfile()
        self.verification_method = verification_method

        self.state = DeliveryState.STANDBY
        self.current_position = GeoPoint(
            latitude=home_location.latitude,
            longitude=home_location.longitude,
            altitude_m=0.0,
        )
        self.altitude_agl_m = 0.0
        self.payload_released = False
        self.delivery_verified = False
        self.verification_source: Optional[str] = None
        self.verification_details: Optional[str] = None
        self.drop_accuracy_offset_m = 0.0
        self._hover_timer_s = 0.0

    def confirm_payload_release(self, source: str = "actuator_telemetry") -> None:
        """Signal that the physical actuator has confirmed payload release."""
        self.payload_released = True
        self.task.status = TaskStatus.RELEASED

    def confirm_ground_delivery(self, source: str, details: str = "") -> None:
        """Signal verified payload delivery via sensor, camera, or simulation event."""
        self.delivery_verified = True
        self.verification_source = source
        self.verification_details = details
        self.task.status = TaskStatus.VERIFIED

    def start_mission(self) -> None:
        """Begin delivery flight from standby."""
        if self.state != DeliveryState.STANDBY:
            raise ValueError(f"Cannot start mission from state {self.state}")
        self.state = DeliveryState.TAKEOFF
        self.task.status = TaskStatus.EN_ROUTE

    def update(self, dt_s: float) -> DeliveryState:
        """Step simulation by dt seconds and advance delivery state machine."""
        if dt_s <= 0.0:
            return self.state

        dist_to_target = haversine_distance_m(self.current_position, self.target_location)
        dist_to_home = haversine_distance_m(self.current_position, self.home_location)

        if self.state == DeliveryState.TAKEOFF:
            # Climb to transit altitude
            self.altitude_agl_m += self.profile.vertical_speed_mps * dt_s
            if self.altitude_agl_m >= self.profile.transit_altitude_m:
                self.altitude_agl_m = self.profile.transit_altitude_m
                self.state = DeliveryState.EN_ROUTE

        elif self.state == DeliveryState.EN_ROUTE:
            # Move horizontally towards target
            step_m = self.profile.horizontal_speed_mps * dt_s
            if dist_to_target <= max(15.0, step_m):
                self._move_towards(self.target_location, min(step_m, dist_to_target))
                self.state = DeliveryState.APPROACH
            else:
                self._move_towards(self.target_location, step_m)

        elif self.state == DeliveryState.APPROACH:
            # Final approach into delivery tolerance radius
            step_m = (self.profile.horizontal_speed_mps * 0.5) * dt_s
            if dist_to_target <= self.profile.drop_tolerance_radius_m:
                self.state = DeliveryState.DESCENT_TO_DROP
            else:
                self._move_towards(self.target_location, min(step_m, dist_to_target))

        elif self.state == DeliveryState.DESCENT_TO_DROP:
            # Descend from transit altitude to safe drop altitude
            self.altitude_agl_m -= self.profile.vertical_speed_mps * dt_s
            if self.altitude_agl_m <= self.profile.release_altitude_m:
                self.altitude_agl_m = self.profile.release_altitude_m
                self._hover_timer_s = 0.0
                self.state = DeliveryState.PAYLOAD_RELEASE

        elif self.state == DeliveryState.PAYLOAD_RELEASE:
            # Stabilize hover and trigger release mechanism
            self._hover_timer_s += dt_s
            if self._hover_timer_s >= self.profile.release_hover_time_s:
                self.payload_released = True
                self.task.status = TaskStatus.RELEASED
                self.drop_accuracy_offset_m = dist_to_target
                self._hover_timer_s = 0.0
                self.state = DeliveryState.VERIFY_DROP

        elif self.state == DeliveryState.VERIFY_DROP:
            self._hover_timer_s += dt_s
            if self.verification_method == DeliveryVerificationMethod.SIMULATED_TIMER:
                # Timed demonstration behavior: recorded as simulated demo timer, not physical sensor contact
                if self._hover_timer_s >= self.profile.verification_time_s:
                    self.delivery_verified = True
                    self.verification_source = "simulated_timer_demo"
                    self.verification_details = "Demonstration timer elapsed (simulated payload drop verification)"
                    self.task.status = TaskStatus.VERIFIED
                    self.state = DeliveryState.ASCENT
            else:
                # Sensor / actuator / simulation event required
                if self.delivery_verified:
                    self.state = DeliveryState.ASCENT
                elif self._hover_timer_s >= self.profile.verification_timeout_s:
                    self.verification_details = (
                        f"Drop verification timed out after {self._hover_timer_s:.1f}s waiting for {self.verification_method.value}"
                    )
                    self.state = DeliveryState.ASCENT

        elif self.state == DeliveryState.ASCENT:
            # Climb back to transit altitude before returning
            self.altitude_agl_m += self.profile.vertical_speed_mps * dt_s
            if self.altitude_agl_m >= self.profile.transit_altitude_m:
                self.altitude_agl_m = self.profile.transit_altitude_m
                self.state = DeliveryState.RETURNING_HOME

        elif self.state == DeliveryState.RETURNING_HOME:
            # Transit back to home base
            step_m = self.profile.horizontal_speed_mps * dt_s
            if dist_to_home <= max(2.0, step_m):
                self._move_towards(self.home_location, min(step_m, dist_to_home))
                self.altitude_agl_m = 0.0
                self.state = DeliveryState.COMPLETED
            else:
                self._move_towards(self.home_location, step_m)

        return self.state

    def abort(self) -> None:
        """Abort delivery mission and trigger RTL immediately."""
        self.state = DeliveryState.RETURNING_HOME

    def _move_towards(self, destination: GeoPoint, distance_m: float) -> None:
        """Displace current simulated position towards target coordinates by distance_m."""
        total_dist = haversine_distance_m(self.current_position, destination)
        if total_dist <= 1e-4:
            self.current_position = GeoPoint(
                latitude=destination.latitude,
                longitude=destination.longitude,
                altitude_m=self.altitude_agl_m,
            )
            return

        fraction = min(1.0, distance_m / total_dist)
        new_lat = self.current_position.latitude + fraction * (destination.latitude - self.current_position.latitude)
        new_lon = self.current_position.longitude + fraction * (destination.longitude - self.current_position.longitude)

        self.current_position = GeoPoint(
            latitude=new_lat,
            longitude=new_lon,
            altitude_m=self.altitude_agl_m,
        )
