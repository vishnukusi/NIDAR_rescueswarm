"""Multi-drone fleet flight bridge coordinator for NIDAR RescueSwarm.

Manages isolated MAVLinkFlightBridge instances for the complete 6-drone fleet:
- Search Fleet: S1, S2, S3
- Delivery Fleet: D1, D2, D3

Guarantees distinct telemetry/command channels, prevents cross-talk, and ensures
SafetyManager actions immediately preempt mission setpoints.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

from .flight_bridge import AutopilotType, FlightBridgeConfig, MAVLinkFlightBridge
from .safety_manager import SafetyAction, SafetyAlert, SafetyManager


DEFAULT_ARDUPILOT_FLEET_PORT_MAP = {
    "S1": {"sysid": 1, "port": 14550},
    "S2": {"sysid": 2, "port": 14551},
    "S3": {"sysid": 3, "port": 14552},
    "D1": {"sysid": 4, "port": 14553},
    "D2": {"sysid": 5, "port": 14554},
    "D3": {"sysid": 6, "port": 14555},
}

DEFAULT_PX4_FLEET_PORT_MAP = {
    "S1": {"sysid": 1, "port": 14540, "gcs_port": 14550},
    "S2": {"sysid": 2, "port": 14541, "gcs_port": 14551},
    "S3": {"sysid": 3, "port": 14542, "gcs_port": 14552},
    "D1": {"sysid": 4, "port": 14543, "gcs_port": 14553},
    "D2": {"sysid": 5, "port": 14544, "gcs_port": 14554},
    "D3": {"sysid": 6, "port": 14545, "gcs_port": 14555},
}

DEFAULT_FLEET_PORT_MAP = DEFAULT_PX4_FLEET_PORT_MAP


class FleetFlightBridgeManager:
    """Coordinates independent flight bridges across all 6 vehicles in the swarm."""

    def __init__(
        self,
        autopilot_type: AutopilotType = AutopilotType.PX4,
        connection_scheme: str = "mock",  # "mock" or "udp"
        port_map: Optional[Dict[str, dict]] = None,
    ) -> None:
        self.autopilot_type = autopilot_type
        self.connection_scheme = connection_scheme
        if port_map is not None:
            self.port_map = port_map
        elif self.autopilot_type == AutopilotType.PX4:
            self.port_map = DEFAULT_PX4_FLEET_PORT_MAP
        else:
            self.port_map = DEFAULT_ARDUPILOT_FLEET_PORT_MAP

        self.bridges: Dict[str, MAVLinkFlightBridge] = {}
        for drone_id, info in self.port_map.items():
            sysid = info["sysid"]
            port = info["port"]
            uri = f"{self.connection_scheme}:127.0.0.1:{port}"
            cfg = FlightBridgeConfig(
                drone_id=drone_id,
                autopilot_type=autopilot_type,
                connection_string=uri,
                target_system=sysid,
                target_component=1,
            )
            self.bridges[drone_id] = MAVLinkFlightBridge(cfg)

    @property
    def is_mock(self) -> bool:
        """Indicates whether fleet operates on mock loopback or real MAVLink network."""
        return self.connection_scheme == "mock" or self.autopilot_type == AutopilotType.MOCK

    @property
    def backend_name(self) -> str:
        """Human-readable backend mode name."""
        return "MOCK_EMULATION" if self.is_mock else "REAL_MAVLINK"

    def get_bridge(self, drone_id: str) -> MAVLinkFlightBridge:
        """Retrieve the isolated bridge for a specific drone."""
        if drone_id not in self.bridges:
            raise KeyError(f"Drone ID {drone_id} not registered in fleet bridge")
        return self.bridges[drone_id]

    def get_connection_summary(self) -> Dict[str, dict]:
        """Summary of backend state and connection status for all 6 fleet vehicles."""
        return {
            drone_id: {
                "backend": bridge.backend_name,
                "is_mock": bridge.is_mock_backend,
                "connection_state": bridge.connection_state.value,
                "connected": bridge.is_connected,
                "uri": bridge.config.connection_string,
                "system_id": bridge.config.target_system,
                "mode": bridge.telemetry.mode,
                "armed": bridge.telemetry.armed,
            }
            for drone_id, bridge in self.bridges.items()
        }

    def connect_all(self, timeout_s: float = 2.0) -> Dict[str, bool]:
        """Attempt connection on all configured drone bridges."""
        results = {}
        for drone_id, bridge in self.bridges.items():
            results[drone_id] = bridge.connect(timeout_s=timeout_s)
        return results

    def disconnect_all(self) -> None:
        """Close sockets on all configured drone bridges."""
        for bridge in self.bridges.values():
            bridge.disconnect()

    def update_all_telemetry(self, current_time: Optional[float] = None) -> None:
        """Drain telemetry on all active vehicle bridges."""
        for bridge in self.bridges.values():
            bridge.update_telemetry(current_time=current_time)

    def enforce_safety_alerts(self, alerts: List[SafetyAlert]) -> Dict[str, SafetyAction]:
        """Ensure SafetyManager actions immediately preempt active mission setpoints.

        If a failsafe is triggered (e.g. Geofence breach or critical battery),
        the vehicle bridge executes the safety override command with highest priority.
        """
        enforced = {}
        for alert in alerts:
            if alert.drone_id in self.bridges:
                bridge = self.bridges[alert.drone_id]
                if alert.action in (SafetyAction.HOLD, SafetyAction.RTL, SafetyAction.EMERGENCY_LAND):
                    bridge.send_safety_override(alert.action)
                    enforced[alert.drone_id] = alert.action
        return enforced
