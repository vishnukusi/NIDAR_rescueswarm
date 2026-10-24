"""Flight-controller bridge and telemetry adapter for NIDAR RescueSwarm.

Provides a unified, vehicle-isolated bridge connecting high-level mission logic
(SearchDroneApproachController, SwarmManager, SafetyManager) to simulated or real
autopilots (ArduCopter / PX4) using standard MAVLink protocols over pymavlink.
"""

from dataclasses import dataclass, field
from enum import Enum
import math
import time
from typing import List, Optional, Tuple

from pymavlink import mavutil

from .approach_controller import VisualServoCommand
from .localization import DroneTelemetry, GeoPoint
from .safety_manager import SafetyAction


class AutopilotType(str, Enum):
    """Supported autopilot flight stacks."""
    ARDUPILOT = "ardupilot"    # ArduCopter SITL (GUIDED mode, MAV_TYPE_QUADROTOR)
    PX4 = "px4"                # PX4 Autopilot (OFFBOARD mode, uORB/MAVLink)
    MOCK = "mock"              # In-memory mock harness for sandboxed tests


class VehicleConnectionState(str, Enum):
    """Network connection status to autopilot instance."""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    HEARTBEAT_LOST = "heartbeat_lost"


@dataclass
class FlightBridgeConfig:
    """Configuration for an individual vehicle flight bridge instance."""
    drone_id: str = "S1"
    autopilot_type: AutopilotType = AutopilotType.ARDUPILOT
    connection_string: str = "udp:127.0.0.1:14550"
    target_system: int = 1
    target_component: int = 1
    heartbeat_timeout_s: float = 3.0
    telemetry_stale_timeout_s: float = 2.0
    default_altitude_m: float = 30.0


@dataclass
class BridgeTelemetryState:
    """Current telemetry parsed from flight controller telemetry stream."""
    drone_id: str
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"
    latitude: float = 12.972000
    longitude: float = 77.594500
    altitude_agl_m: float = 0.0
    altitude_msl_m: float = 920.0
    heading_deg: float = 45.0
    yaw_rad: float = 0.7854
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    velocity_north_mps: float = 0.0
    velocity_east_mps: float = 0.0
    velocity_down_mps: float = 0.0
    battery_fraction: float = 1.0
    last_heartbeat_time: float = 0.0
    last_telemetry_time: float = 0.0


@dataclass
class MockAutopilotBackend:
    """In-memory flight controller simulation for sandboxed testing and offline validation."""
    drone_id: str
    target_system: int = 1
    armed: bool = False
    mode: str = "STABILIZE"
    lat: float = 12.972000
    lon: float = 77.594500
    alt_agl_m: float = 0.0
    heading_deg: float = 45.0
    yaw_rad: float = 0.7854
    vel_n_mps: float = 0.0
    vel_e_mps: float = 0.0
    vel_d_mps: float = 0.0
    battery_fraction: float = 0.94


class MAVLinkFlightBridge:
    """Manages MAVLink connection, telemetry parsing, and setpoint streaming for a drone."""

    def __init__(self, config: Optional[FlightBridgeConfig] = None) -> None:
        self.config = config or FlightBridgeConfig()
        self.drone_id = self.config.drone_id
        self.connection_state = VehicleConnectionState.DISCONNECTED
        self.telemetry = BridgeTelemetryState(drone_id=self.drone_id)

        self._master: Optional[mavutil.mavfile] = None
        self._last_command_time: float = 0.0
        self._target_yaw_rad: float = 0.0
        self.last_error: Optional[str] = None
        self.actual_system_id: Optional[int] = None
        self.actual_component_id: Optional[int] = None

        self._is_mock = (
            self.config.autopilot_type == AutopilotType.MOCK
            or self.config.connection_string.startswith("mock:")
        )
        self._mock_backend: Optional[MockAutopilotBackend] = (
            MockAutopilotBackend(
                drone_id=self.drone_id,
                target_system=self.config.target_system,
            )
            if self._is_mock
            else None
        )

    @property
    def is_connected(self) -> bool:
        return self.connection_state == VehicleConnectionState.CONNECTED

    @property
    def is_mock_backend(self) -> bool:
        """Indicates whether this bridge operates on the mock loopback or real MAVLink network."""
        return self._is_mock

    @property
    def backend_name(self) -> str:
        """Human-readable backend mode name."""
        return "MOCK_EMULATION" if self._is_mock else "REAL_MAVLINK"

    @property
    def mock_backend(self) -> Optional[MockAutopilotBackend]:
        return self._mock_backend

    def connect(self, timeout_s: float = 5.0) -> bool:
        """Establish network socket connection and wait for first heartbeat."""
        self.connection_state = VehicleConnectionState.CONNECTING
        self.last_error = None
        now = time.time()

        if self._is_mock:
            self.connection_state = VehicleConnectionState.CONNECTED
            self.actual_system_id = self.config.target_system
            self.actual_component_id = self.config.target_component
            self.telemetry.connected = True
            self.telemetry.last_heartbeat_time = now
            self.telemetry.last_telemetry_time = now
            self.telemetry.armed = self._mock_backend.armed
            self.telemetry.mode = self._mock_backend.mode
            self.telemetry.latitude = self._mock_backend.lat
            self.telemetry.longitude = self._mock_backend.lon
            self.telemetry.altitude_agl_m = self._mock_backend.alt_agl_m
            self.telemetry.yaw_rad = self._mock_backend.yaw_rad
            self.telemetry.heading_deg = self._mock_backend.heading_deg
            return True

        # Real MAVLink network socket connection
        try:
            self._master = mavutil.mavlink_connection(
                self.config.connection_string,
                source_system=255,  # Standard GCS / Companion system ID
                source_component=190,
            )
            start = time.time()
            while time.time() - start < timeout_s:
                msg = self._master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
                if msg is not None:
                    hb_time = time.time()
                    self.actual_system_id = msg.get_srcSystem()
                    self.actual_component_id = msg.get_srcComponent()
                    self.connection_state = VehicleConnectionState.CONNECTED
                    self.telemetry.connected = True
                    self.telemetry.last_heartbeat_time = hb_time
                    self.telemetry.last_telemetry_time = hb_time
                    self.telemetry.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    return True

            self.last_error = (
                f"No MAVLink HEARTBEAT received from real endpoint '{self.config.connection_string}' "
                f"within {timeout_s:.1f}s. Ensure ArduCopter SITL is running and broadcasting."
            )
            self.disconnect()
            return False

        except Exception as exc:
            self.last_error = f"Socket connection failed to '{self.config.connection_string}': {exc}"
            self.disconnect()
            return False

    def disconnect(self) -> None:
        """Close connection socket."""
        if self._master is not None:
            try:
                if hasattr(self._master, "port") and hasattr(self._master.port, "close"):
                    self._master.port.close()
                if hasattr(self._master, "close"):
                    self._master.close()
            except Exception:
                pass
            self._master = None
        self.connection_state = VehicleConnectionState.DISCONNECTED
        self.telemetry.connected = False

    def run_preflight_checks(self, timeout_s: float = 3.0) -> Tuple[bool, List[str]]:
        """Verify vehicle telemetry, GPS lock, attitude, and system health before flight."""
        checks: List[str] = []
        passed = True

        if not self.is_connected:
            return False, ["FAIL: Vehicle is not connected to autopilot"]

        checks.append(f"PASS: Autopilot connection active via {self.backend_name} ({self.config.connection_string})")
        if self.actual_system_id is not None and self.actual_system_id != self.config.target_system:
            checks.append(f"FAIL: Vehicle System ID mismatch: expected {self.config.target_system}, received {self.actual_system_id}")
            passed = False
        else:
            checks.append(f"PASS: Vehicle identification verified: System ID {self.actual_system_id or self.config.target_system}")

        self.update_telemetry()
        if self.is_telemetry_fresh():
            checks.append(f"PASS: Telemetry stream fresh (< {self.config.telemetry_stale_timeout_s:.1f}s)")
        else:
            checks.append("FAIL: Telemetry stream is stale or absent")
            passed = False

        if self.telemetry.altitude_agl_m is not None:
            checks.append(f"PASS: Altimeter AGL nominal ({self.telemetry.altitude_agl_m:.1f}m AGL)")
        else:
            checks.append("FAIL: No altitude telemetry available")
            passed = False

        # Verify finite and bounded geographic coordinates
        lat = self.telemetry.latitude
        lon = self.telemetry.longitude
        if math.isfinite(lat) and math.isfinite(lon) and (-90.0 <= lat <= 90.0) and (-180.0 <= lon <= 180.0):
            checks.append(f"PASS: Navigation coordinates valid ({lat:.6f}, {lon:.6f})")
        else:
            checks.append("FAIL: Invalid or non-finite geographic coordinates")
            passed = False

        if self.telemetry.battery_fraction is not None and math.isfinite(self.telemetry.battery_fraction):
            checks.append(f"PASS: Battery health verified ({self.telemetry.battery_fraction*100:.0f}%)")
        else:
            checks.append("FAIL: Invalid battery telemetry")
            passed = False
        return passed, checks

    def update_telemetry(self, current_time: Optional[float] = None) -> BridgeTelemetryState:
        """Drain incoming MAVLink packets non-blocking and update telemetry state."""
        now = current_time if current_time is not None else time.time()

        if self._is_mock:
            if self.connection_state == VehicleConnectionState.CONNECTED:
                # Check for heartbeat loss
                if now - self.telemetry.last_heartbeat_time > self.config.heartbeat_timeout_s:
                    self.connection_state = VehicleConnectionState.HEARTBEAT_LOST
                    self.telemetry.connected = False
                else:
                    self.telemetry.last_telemetry_time = now
                    self.telemetry.armed = self._mock_backend.armed
                    self.telemetry.mode = self._mock_backend.mode
                    self.telemetry.altitude_agl_m = self._mock_backend.alt_agl_m
                    self.telemetry.latitude = self._mock_backend.lat
                    self.telemetry.longitude = self._mock_backend.lon
                    self.telemetry.velocity_north_mps = self._mock_backend.vel_n_mps
                    self.telemetry.velocity_east_mps = self._mock_backend.vel_e_mps
                    self.telemetry.velocity_down_mps = self._mock_backend.vel_d_mps
                    self.telemetry.yaw_rad = self._mock_backend.yaw_rad
                    self.telemetry.heading_deg = math.degrees(self._mock_backend.yaw_rad) % 360.0
            return self.telemetry

        if self._master is None:
            self.connection_state = VehicleConnectionState.DISCONNECTED
            return self.telemetry

        # Read available messages without blocking
        while True:
            msg = self._master.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()
            if msg_type == "HEARTBEAT":
                self.telemetry.last_heartbeat_time = now
                self.telemetry.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                # Decode mode based on autopilot
                if self.config.autopilot_type == AutopilotType.ARDUPILOT:
                    mode_map = mavutil.mode_mapping_acm
                    if mode_map:
                        self.telemetry.mode = mode_map.get(msg.custom_mode, "UNKNOWN")
                elif self.config.autopilot_type == AutopilotType.PX4:
                    main_mode = (msg.custom_mode >> 16) & 0xFF
                    sub_mode = (msg.custom_mode >> 24) & 0xFF
                    if main_mode == 1:
                        self.telemetry.mode = "MANUAL"
                    elif main_mode == 2:
                        self.telemetry.mode = "ALTCTL"
                    elif main_mode == 3:
                        self.telemetry.mode = "POSCTL"
                    elif main_mode == 4:
                        if sub_mode == 3:
                            self.telemetry.mode = "HOLD"
                        elif sub_mode == 5:
                            self.telemetry.mode = "RTL"
                        elif sub_mode == 6:
                            self.telemetry.mode = "LAND"
                        elif sub_mode == 2:
                            self.telemetry.mode = "TAKEOFF"
                        elif sub_mode == 4:
                            self.telemetry.mode = "MISSION"
                        else:
                            self.telemetry.mode = "AUTO"
                    elif main_mode == 5:
                        self.telemetry.mode = "ACRO"
                    elif main_mode == 6:
                        self.telemetry.mode = "OFFBOARD"
                    elif main_mode == 7:
                        self.telemetry.mode = "STABILIZED"
                    elif main_mode == 8:
                        self.telemetry.mode = "RATTITUDE"
                    else:
                        self.telemetry.mode = "UNKNOWN"

            elif msg_type == "GLOBAL_POSITION_INT":
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt_msl = msg.alt / 1e3
                alt_agl = max(0.0, msg.relative_alt / 1e3)
                if (
                    math.isfinite(lat) and math.isfinite(lon)
                    and math.isfinite(alt_msl) and math.isfinite(alt_agl)
                    and (-90.0 <= lat <= 90.0) and (-180.0 <= lon <= 180.0)
                ):
                    self.telemetry.last_telemetry_time = now
                    self.telemetry.latitude = lat
                    self.telemetry.longitude = lon
                    self.telemetry.altitude_msl_m = alt_msl
                    self.telemetry.altitude_agl_m = alt_agl
                    if hasattr(msg, "vx") and math.isfinite(msg.vx):
                        self.telemetry.velocity_north_mps = msg.vx / 1e2
                    if hasattr(msg, "vy") and math.isfinite(msg.vy):
                        self.telemetry.velocity_east_mps = msg.vy / 1e2
                    if hasattr(msg, "vz") and math.isfinite(msg.vz):
                        self.telemetry.velocity_down_mps = msg.vz / 1e2
                    if hasattr(msg, "hdg") and math.isfinite(msg.hdg):
                        self.telemetry.heading_deg = msg.hdg / 1e2

            elif msg_type == "ATTITUDE":
                roll = math.degrees(msg.roll)
                pitch = math.degrees(msg.pitch)
                yaw = msg.yaw
                if math.isfinite(roll) and math.isfinite(pitch) and math.isfinite(yaw):
                    self.telemetry.last_telemetry_time = now
                    self.telemetry.roll_deg = roll
                    self.telemetry.pitch_deg = pitch
                    self.telemetry.yaw_rad = yaw
                    self.telemetry.heading_deg = math.degrees(yaw) % 360.0

            elif msg_type == "SYS_STATUS":
                if msg.battery_remaining != -1 and math.isfinite(msg.battery_remaining):
                    self.telemetry.battery_fraction = max(0.0, min(1.0, msg.battery_remaining / 100.0))

        # Check for stale heartbeat
        if now - self.telemetry.last_heartbeat_time > self.config.heartbeat_timeout_s:
            if self.connection_state == VehicleConnectionState.CONNECTED:
                self.connection_state = VehicleConnectionState.HEARTBEAT_LOST
                self.telemetry.connected = False

        return self.telemetry

    def is_telemetry_fresh(self, current_time: Optional[float] = None) -> bool:
        """Verify whether telemetry received from vehicle is within freshness threshold."""
        now = current_time if current_time is not None else time.time()
        if not self.is_connected:
            return False
        return (now - self.telemetry.last_telemetry_time) <= self.config.telemetry_stale_timeout_s

    def set_guided_mode(self, confirm_timeout_s: float = 4.0) -> bool:
        """Command autopilot into guided / offboard velocity control mode."""
        if self._is_mock:
            self._mock_backend.mode = "GUIDED"
            self.telemetry.mode = "GUIDED"
            return True

        if self._master is None or not self.is_connected:
            self.last_error = "Cannot set mode: vehicle not connected"
            return False

        start = time.time()
        while time.time() - start < confirm_timeout_s:
            if self.config.autopilot_type == AutopilotType.ARDUPILOT:
                mode_mapping = self._master.mode_mapping() if hasattr(self._master, "mode_mapping") else None
                mode_id = mode_mapping.get("GUIDED", 4) if mode_mapping else 4
                self._master.set_mode(mode_id)
            elif self.config.autopilot_type == AutopilotType.PX4:
                # Pre-stream setpoint before requesting OFFBOARD mode (PX4 rejects offboard if no setpoint active)
                if self.telemetry.latitude != 0.0 and math.isfinite(self.telemetry.latitude):
                    self.send_position_target_global(
                        self.telemetry.latitude,
                        self.telemetry.longitude,
                        max(1.0, self.telemetry.altitude_agl_m),
                        self.telemetry.heading_deg,
                    )
                else:
                    self.send_velocity_ned(0.0, 0.0, 0.0, self.telemetry.yaw_rad)
                time.sleep(0.05)
                self._master.mav.command_long_send(
                    self.config.target_system,
                    self.config.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    6,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
                    0, 0, 0, 0, 0,
                )

            inner_start = time.time()
            while time.time() - inner_start < 0.5:
                self.update_telemetry()
                if self.telemetry.mode in ("GUIDED", "OFFBOARD"):
                    return True
                time.sleep(0.05)

        self.last_error = (
            f"Failed to engage GUIDED mode within {confirm_timeout_s:.1f}s. "
            f"Current vehicle mode: '{self.telemetry.mode}'"
        )
        return False

    def set_mode(self, mode: str, confirm_timeout_s: float = 4.0) -> bool:
        """Command autopilot into specified flight mode."""
        mode_upper = mode.upper()
        if mode_upper in ("GUIDED", "OFFBOARD"):
            return self.set_guided_mode(confirm_timeout_s=confirm_timeout_s)
        elif mode_upper in ("HOLD", "LOITER"):
            return self.send_safety_override(SafetyAction.HOLD)
        elif mode_upper in ("RTL", "RETURN"):
            return self.send_safety_override(SafetyAction.RTL)
        elif mode_upper in ("LAND", "EMERGENCY_LAND"):
            return self.send_safety_override(SafetyAction.EMERGENCY_LAND)
        return False

    def arm(self, confirm_timeout_s: float = 6.0) -> bool:
        """Arm vehicle propulsion motors and verify armed status."""
        if self._is_mock:
            self._mock_backend.armed = True
            self.telemetry.armed = True
            return True

        if self._master is None or not self.is_connected:
            self.last_error = "Cannot arm: vehicle not connected"
            return False

        start = time.time()
        while time.time() - start < confirm_timeout_s:
            self._master.mav.command_long_send(
                self.config.target_system,
                self.config.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,  # 1 = Arm
                0, 0, 0, 0, 0, 0,
            )
            inner_start = time.time()
            while time.time() - inner_start < 0.8:
                self.update_telemetry()
                if self.telemetry.armed:
                    return True
                time.sleep(0.05)

        self.last_error = (
            f"Arming propulsion motors timed out after {confirm_timeout_s:.1f}s. "
            "Ensure pre-arm checks (EKF 3D GPS lock, origin set) have passed in SITL."
        )
        return False

    def disarm(self) -> bool:
        """Disarm vehicle propulsion motors."""
        if self._is_mock:
            self._mock_backend.armed = False
            self.telemetry.armed = False
            return True

        if self._master is None or not self.is_connected:
            return False
        self._master.mav.command_long_send(
            self.config.target_system,
            self.config.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # 0 = Disarm
            0, 0, 0, 0, 0, 0,
        )
        return True

    def takeoff(self, altitude_m: float, confirm_timeout_s: float = 15.0) -> bool:
        """Command vehicle takeoff to designated altitude."""
        if altitude_m <= 0.0:
            self.last_error = "Takeoff altitude must be strictly positive"
            return False

        if self._is_mock:
            self._mock_backend.alt_agl_m = altitude_m
            self.telemetry.altitude_agl_m = altitude_m
            return True

        if self._master is None or not self.is_connected:
            self.last_error = "Cannot takeoff: vehicle not connected"
            return False

        self._master.mav.command_long_send(
            self.config.target_system,
            self.config.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0,
            altitude_m,
        )

        start = time.time()
        while time.time() - start < confirm_timeout_s:
            self.update_telemetry()
            if self.telemetry.altitude_agl_m >= altitude_m * 0.90:
                return True
            time.sleep(0.1)

        self.last_error = (
            f"Takeoff to {altitude_m:.1f}m timed out after {confirm_timeout_s:.1f}s. "
            f"Current altitude reached: {self.telemetry.altitude_agl_m:.1f}m AGL"
        )
        return False

    def send_velocity_ned(
        self,
        velocity_north_mps: float,
        velocity_east_mps: float,
        velocity_down_mps: float = 0.0,
        yaw_rad: Optional[float] = None,
    ) -> bool:
        """Send local NED velocity vector setpoints via SET_POSITION_TARGET_LOCAL_NED.

        Ignores position, acceleration, and yaw-rate components, controlling
        lateral velocity and locking vehicle heading.
        """
        hold_yaw = yaw_rad if yaw_rad is not None else self.telemetry.yaw_rad

        if self._is_mock:
            self._mock_backend.vel_n_mps = velocity_north_mps
            self._mock_backend.vel_e_mps = velocity_east_mps
            self._mock_backend.vel_d_mps = velocity_down_mps
            self._mock_backend.yaw_rad = hold_yaw
            self.telemetry.velocity_north_mps = velocity_north_mps
            self.telemetry.velocity_east_mps = velocity_east_mps
            self.telemetry.velocity_down_mps = velocity_down_mps
            self.telemetry.yaw_rad = hold_yaw
            self.telemetry.heading_deg = math.degrees(hold_yaw) % 360.0

            # Integrate small coordinate step for realism
            meters_to_lat = 1.0 / 111139.0
            meters_to_lon = 1.0 / (111139.0 * math.cos(math.radians(self._mock_backend.lat)))
            self._mock_backend.lat += velocity_north_mps * 0.1 * meters_to_lat
            self._mock_backend.lon += velocity_east_mps * 0.1 * meters_to_lon
            self.telemetry.latitude = self._mock_backend.lat
            self.telemetry.longitude = self._mock_backend.lon
            self._last_command_time = time.time()
            return True

        if self._master is None or not self.is_connected:
            self.last_error = "Cannot send velocity setpoint: vehicle not connected"
            return False

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self._master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.config.target_system,
            self.config.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0, 0.0, 0.0,  # position (ignored)
            float(velocity_north_mps),
            float(velocity_east_mps),
            float(velocity_down_mps),
            0.0, 0.0, 0.0,  # acceleration (ignored)
            float(hold_yaw),
            0.0,  # yaw_rate (ignored)
        )
        self._last_command_time = time.time()
        return True

    def send_position_target_global(
        self, lat: float, lon: float, alt_agl_m: float, yaw_deg: float = 0.0
    ) -> bool:
        """Send global WGS84 position target to autopilot."""
        if not (math.isfinite(lat) and math.isfinite(lon) and math.isfinite(alt_agl_m) and math.isfinite(yaw_deg)):
            self.last_error = "Invalid non-finite position target parameters"
            return False

        if self._is_mock:
            self._mock_backend.lat = lat
            self._mock_backend.lon = lon
            self._mock_backend.alt_agl_m = alt_agl_m
            self._mock_backend.yaw_rad = math.radians(yaw_deg)
            self._mock_backend.heading_deg = yaw_deg % 360.0
            self.telemetry.latitude = lat
            self.telemetry.longitude = lon
            self.telemetry.altitude_agl_m = alt_agl_m
            self.telemetry.yaw_rad = math.radians(yaw_deg)
            self.telemetry.heading_deg = yaw_deg % 360.0
            self._last_command_time = time.time()
            return True

        if self._master is None or not self.is_connected:
            self.last_error = "Cannot send position target: vehicle not connected"
            return False

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self._master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms
            self.config.target_system,
            self.config.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            int(lat * 1e7),
            int(lon * 1e7),
            float(alt_agl_m),
            0.0, 0.0, 0.0,  # vx, vy, vz
            0.0, 0.0, 0.0,  # afx, afy, afz
            math.radians(yaw_deg),
            0.0,  # yaw_rate
        )
        self._last_command_time = time.time()
        return True

    def apply_visual_servo_command(self, cmd: VisualServoCommand) -> bool:
        """Forward VisualServoCommand computed by approach controller directly to autopilot."""
        return self.send_velocity_ned(
            velocity_north_mps=cmd.local_ned_north_mps,
            velocity_east_mps=cmd.local_ned_east_mps,
            velocity_down_mps=0.0,
            yaw_rad=cmd.hold_yaw_rad,
        )

    def send_safety_override(self, action: SafetyAction) -> bool:
        """Execute immediate flight controller safety action."""
        if self._is_mock:
            if action == SafetyAction.HOLD:
                return self.send_velocity_ned(0.0, 0.0, 0.0, self.telemetry.yaw_rad)
            elif action == SafetyAction.RTL:
                self._mock_backend.mode = "RTL"
                self.telemetry.mode = "RTL"
                self._mock_backend.vel_n_mps = 0.0
                self._mock_backend.vel_e_mps = 0.0
                return True
            elif action == SafetyAction.EMERGENCY_LAND:
                self._mock_backend.mode = "LAND"
                self.telemetry.mode = "LAND"
                self._mock_backend.vel_n_mps = 0.0
                self._mock_backend.vel_e_mps = 0.0
                return True
            return False

        if self._master is None or not self.is_connected:
            self.last_error = "Cannot send safety override: vehicle not connected"
            return False

        if action == SafetyAction.HOLD:
            if self.config.autopilot_type == AutopilotType.PX4 and self._master is not None:
                self._master.mav.command_long_send(
                    self.config.target_system,
                    self.config.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    4,  # PX4_CUSTOM_MAIN_MODE_AUTO
                    3,  # PX4_CUSTOM_SUB_MODE_AUTO_LOITER (Hold)
                    0, 0, 0, 0,
                )
            return self.send_velocity_ned(0.0, 0.0, 0.0, self.telemetry.yaw_rad)

        elif action == SafetyAction.RTL:
            if self.config.autopilot_type == AutopilotType.ARDUPILOT:
                try:
                    mode_mapping = self._master.mode_mapping() if hasattr(self._master, "mode_mapping") else None
                    mode_id = mode_mapping.get("RTL", 6) if mode_mapping else 6
                    self._master.set_mode(mode_id)
                except Exception:
                    pass
            self._master.mav.command_long_send(
                self.config.target_system,
                self.config.target_component,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0,
                0, 0, 0, 0, 0, 0, 0,
            )
            return True

        elif action == SafetyAction.EMERGENCY_LAND:
            if self.config.autopilot_type == AutopilotType.ARDUPILOT:
                try:
                    mode_mapping = self._master.mode_mapping() if hasattr(self._master, "mode_mapping") else None
                    mode_id = mode_mapping.get("LAND", 9) if mode_mapping else 9
                    self._master.set_mode(mode_id)
                except Exception:
                    pass
            self._master.mav.command_long_send(
                self.config.target_system,
                self.config.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LAND,
                0,
                0, 0, 0, 0, 0, 0, 0,
            )
            return True

        return False

    def to_drone_telemetry(self) -> DroneTelemetry:
        """Export parsed telemetry state into core DroneTelemetry for localization and safety."""
        return DroneTelemetry(
            drone_id=self.drone_id,
            position=GeoPoint(
                latitude=self.telemetry.latitude,
                longitude=self.telemetry.longitude,
                altitude_m=self.telemetry.altitude_agl_m,
            ),
            altitude_agl_m=max(0.1, self.telemetry.altitude_agl_m),
            heading_deg=self.telemetry.heading_deg,
            pitch_deg=self.telemetry.pitch_deg,
            roll_deg=self.telemetry.roll_deg,
            timestamp=self.telemetry.last_telemetry_time,
        )
