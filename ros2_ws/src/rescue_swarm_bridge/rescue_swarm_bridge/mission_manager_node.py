"""ROS 2 Mission Manager Node for NIDAR RescueSwarm.

Orchestrates the 6-drone heterogeneous fleet:
- Search Drones: S1, S2, S3
- Delivery Drones: D1, D2, D3

Core Responsibilities:
1. Fleet configuration loading and preflight readiness validation.
2. Independent vehicle adapter lifecycle management with vehicle isolation.
3. Telemetry polling and SafetyManager watchdog execution.
4. Autonomous search coverage planning and waypoint dispatch for S1-S3.
5. Real-time survivor task allocation for D1-D3.
6. Monitored delivery progression and simulated drop verification.
7. Mission status publication and post-mission deterministic reporting.
"""

from dataclasses import dataclass
from enum import Enum
import json
import logging
import math
import os
import signal
import sys
import time
from typing import Any, Dict, List, Optional

from .simulated_delivery_node import SimulatedDeliveryController, SimulatedDeliveryStage, SimulatedDeliveryVerificationMode
from .vehicle_adapter import (
    AdapterType,
    VehicleAdapter,
    VehicleConnectionState,
    VehicleTelemetry,
    create_vehicle_adapter,
)

logger = logging.getLogger(__name__)


class MissionPhase(str, Enum):
    """Global mission lifecycle phases."""
    INIT = "INIT"
    PREFLIGHT_CHECK = "PREFLIGHT_CHECK"
    READY = "READY"
    SEARCH_AND_RESCUE = "SEARCH_AND_RESCUE"
    ALL_DELIVERIES_COMPLETE = "ALL_DELIVERIES_COMPLETE"
    RETURN_TO_LAUNCH = "RETURN_TO_LAUNCH"
    COMPLETED = "COMPLETED"
    ABORTED_SAFETY = "ABORTED_SAFETY"
    TIMEOUT = "TIMEOUT"


class FleetMissionManager:
    """Core autonomous mission coordinator operating independently or inside ROS 2 node."""

    def __init__(
        self,
        fleet_config_path: str = "config/fleet.json",
        adapter_type: AdapterType = AdapterType.MOCK,
        mission_timeout_s: float = 300.0,
        ros2_node: Optional[Any] = None,
    ) -> None:
        self.fleet_config_path = fleet_config_path
        self.adapter_type = adapter_type
        self.mission_timeout_s = mission_timeout_s
        self.ros2_node = ros2_node

        self.phase = MissionPhase.INIT
        self.start_time: Optional[float] = None
        self.fleet_config: Dict[str, Any] = {}
        self.adapters: Dict[str, VehicleAdapter] = {}
        self.delivery_controllers: Dict[str, SimulatedDeliveryController] = {}

        # Autonomy subsystems (from rescue_swarm_core)
        self.safety_manager: Optional[Any] = None
        self.swarm_manager: Optional[Any] = None
        self.coverage_planner: Optional[Any] = None

        self.search_waypoints: Dict[str, List[Any]] = {}
        self.search_wp_indices: Dict[str, int] = {}

        self._load_config()
        self._init_subsystems()
        self._init_adapters()
        self._init_search_plans()

    @staticmethod
    def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        r = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2.0) ** 2
        return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    def _init_search_plans(self) -> None:
        """Generate coverage waypoints for search drones S1, S2, S3."""
        search_drones = sorted([d_id for d_id, cfg in self.drones_dict.items() if cfg.get("role") == "search"])
        if not search_drones:
            return
        for d_id in search_drones:
            self.search_wp_indices[d_id] = 0
            self.search_waypoints[d_id] = []

        if self.coverage_planner:
            try:
                from rescue_swarm_core.coverage_planner import SearchArea
                from rescue_swarm_core.localization import CameraCalibration
                area = SearchArea(min_lat=12.9710, min_lon=77.5920, max_lat=12.9740, max_lon=77.5970)
                cam = CameraCalibration(image_width=1920, image_height=1080, hfov_deg=84.0)
                plan = self.coverage_planner(
                    search_area=area,
                    search_drone_ids=search_drones,
                    camera=cam,
                    altitude_agl_m=30.0,
                )
                for d_id, sector in plan.sectors.items():
                    self.search_waypoints[d_id] = sector.waypoints
            except Exception as e:
                logger.warning("Could not generate coverage plan for fleet: %s", e)

    def _load_config(self) -> None:
        """Load and validate fleet configuration."""
        if not os.path.exists(self.fleet_config_path):
            raise FileNotFoundError(f"Fleet configuration not found at {self.fleet_config_path}")
        with open(self.fleet_config_path, "r", encoding="utf-8") as f:
            self.fleet_config = json.load(f)

        raw_fleet = self.fleet_config.get("fleet")
        if isinstance(raw_fleet, list):
            self.drones_dict = {d["id"]: d for d in raw_fleet if "id" in d}
        elif isinstance(self.fleet_config.get("drones"), dict):
            self.drones_dict = self.fleet_config["drones"]
        else:
            self.drones_dict = {}

        expected = {"S1", "S2", "S3", "D1", "D2", "D3"}
        missing = expected - set(self.drones_dict.keys())
        if missing:
            raise ValueError(f"Fleet configuration is missing expected vehicles: {missing}")

    def _init_subsystems(self) -> None:
        """Initialize core safety, swarm, and coverage planning modules."""
        try:
            from rescue_swarm_core.safety_manager import SafetyManager
            self.safety_manager = SafetyManager()
        except ImportError as e:
            logger.warning("Could not import SafetyManager from rescue_swarm_core: %s", e)

        try:
            from rescue_swarm_core.swarm_manager import SwarmManager
            self.swarm_manager = SwarmManager()
        except ImportError as e:
            logger.warning("Could not import SwarmManager: %s", e)

        try:
            from rescue_swarm_core.coverage_planner import SearchArea, plan_coverage
            self.coverage_planner = plan_coverage
        except ImportError as e:
            logger.warning("Could not import coverage_planner: %s", e)

    def _init_adapters(self) -> None:
        """Initialize isolated vehicle adapters and delivery controllers for all 6 drones."""
        for drone_id, cfg in self.drones_dict.items():
            adapter = create_vehicle_adapter(
                drone_id=drone_id,
                config=cfg,
                adapter_type=self.adapter_type,
                ros2_node=self.ros2_node,
            )
            self.adapters[drone_id] = adapter

            if cfg.get("role") == "delivery":
                self.delivery_controllers[drone_id] = SimulatedDeliveryController(
                    drone_id=drone_id,
                    verification_mode=SimulatedDeliveryVerificationMode.PROXIMITY_AND_HOVER,
                )

    def run_preflight_checks(self, timeout_s: float = 5.0) -> Dict[str, Any]:
        """Perform comprehensive preflight checks across all 6 vehicles."""
        self.phase = MissionPhase.PREFLIGHT_CHECK
        results = {
            "all_passed": True,
            "timestamp": time.time(),
            "vehicles": {},
        }

        for drone_id, adapter in self.adapters.items():
            connected = adapter.connect(timeout_s=timeout_s)
            telem = adapter.update()
            fresh = adapter.is_telemetry_fresh(max_age_s=2.0)
            battery = telem.battery_fraction if (telem and math.isfinite(telem.battery_fraction)) else 0.0

            # Guard: If fleet manager is configured for live/SITL adapter, mock telemetry is strictly rejected
            mock_rejected = (self.adapter_type != AdapterType.MOCK and adapter.is_mock)
            v_ok = connected and fresh and (battery >= 0.20) and not mock_rejected

            results["vehicles"][drone_id] = {
                "connected": connected,
                "telemetry_fresh": fresh,
                "battery": battery,
                "passed": v_ok,
                "adapter": adapter.adapter_type.value,
                "is_mock": adapter.is_mock,
                "error": "Mock telemetry rejected for live adapter" if mock_rejected else None,
            }
            if not v_ok:
                results["all_passed"] = False

        if results["all_passed"]:
            self.phase = MissionPhase.READY
            logger.info("Preflight checks PASSED for all 6 vehicles.")
        else:
            self.phase = MissionPhase.INIT
            logger.error("Preflight checks FAILED: %s", results["vehicles"])
        return results

    def step(self, current_time: Optional[float] = None) -> Dict[str, Any]:
        """Single tick of the mission coordination loop."""
        now = current_time if current_time is not None else time.time()
        if self.start_time is None:
            self.start_time = now

        elapsed = now - self.start_time

        # 1. Check mission timeout
        if elapsed > self.mission_timeout_s and self.phase not in (MissionPhase.COMPLETED, MissionPhase.ABORTED_SAFETY):
            self.phase = MissionPhase.TIMEOUT
            logger.warning("Mission timed out after %.1fs", elapsed)
            self._trigger_fleet_rtl()

        # 2. Poll telemetry across all 6 vehicles
        telemetries: Dict[str, Optional[VehicleTelemetry]] = {}
        for drone_id, adapter in self.adapters.items():
            telemetries[drone_id] = adapter.update(current_time=now)

        # 3. Evaluate fleet safety if SafetyManager is available
        active_incidents = []
        if self.safety_manager is not None:
            for drone_id, telem in telemetries.items():
                if telem:
                    from rescue_swarm_core.localization import DroneTelemetry as CoreTelem
                    from rescue_swarm_core.models import GeoPoint
                    alt_agl = max(0.1, telem.altitude_agl_m)
                    ct = CoreTelem(
                        drone_id=drone_id,
                        position=GeoPoint(latitude=telem.latitude, longitude=telem.longitude),
                        altitude_agl_m=alt_agl,
                        heading_deg=telem.heading_deg,
                        timestamp=telem.timestamp,
                    )
                    alerts = self.safety_manager.check_telemetry(ct, battery_fraction=telem.battery_fraction, current_time=now)
                    for alert in alerts:
                        adapter = self.adapters.get(drone_id)
                        if adapter:
                            adapter.send_safety_action(alert.action.value)
                        active_incidents.append(alert)

        # 4. Mission coordination state progression
        if self.phase == MissionPhase.READY:
            self.phase = MissionPhase.SEARCH_AND_RESCUE
            logger.info("Entering SEARCH_AND_RESCUE phase.")

        if self.phase == MissionPhase.SEARCH_AND_RESCUE:
            # A. Search Fleet (S1, S2, S3) Waypoint Dispatch
            for s_id in ("S1", "S2", "S3"):
                adapter = self.adapters.get(s_id)
                telem = telemetries.get(s_id)
                if adapter and telem and telem.connected:
                    if not telem.armed:
                        adapter.arm()
                    if telem.mode not in ("OFFBOARD", "GUIDED"):
                        adapter.set_mode("OFFBOARD")
                    wps = self.search_waypoints.get(s_id, [])
                    wp_idx = self.search_wp_indices.get(s_id, 0)
                    if wps and wp_idx < len(wps):
                        target_wp = wps[wp_idx]
                        adapter.send_position_setpoint(target_wp.latitude, target_wp.longitude, 30.0)
                        dist = self._haversine(telem.latitude, telem.longitude, target_wp.latitude, target_wp.longitude)
                        if dist <= 5.0 and wp_idx < len(wps) - 1:
                            self.search_wp_indices[s_id] += 1

            # B. Delivery Fleet (D1, D2, D3) Flight Profile Dispatch
            all_deliveries_done = True
            for drone_id, ctrl in self.delivery_controllers.items():
                adapter = self.adapters.get(drone_id)
                telem = telemetries.get(drone_id)
                if telem and ctrl.current_task:
                    all_deliveries_done = False
                    task = ctrl.current_task
                    if adapter and telem.connected:
                        if not telem.armed:
                            adapter.arm()
                        if telem.mode not in ("OFFBOARD", "GUIDED"):
                            adapter.set_mode("OFFBOARD")
                        if task.stage == SimulatedDeliveryStage.INGRESS:
                            adapter.send_position_setpoint(task.target_lat, task.target_lon, 25.0)
                        elif task.stage == SimulatedDeliveryStage.HOVER_DESCENT:
                            adapter.send_position_setpoint(task.target_lat, task.target_lon, task.target_alt_m)
                        elif task.stage in (SimulatedDeliveryStage.VERIFIED, SimulatedDeliveryStage.FAILED):
                            adapter.send_safety_action("RTL")
                    ctrl.update(telem.latitude, telem.longitude, telem.altitude_agl_m, current_time=now)

        status_summary = {
            "phase": self.phase.value,
            "elapsed_s": elapsed,
            "active_incidents": len(active_incidents),
            "vehicles": {
                d_id: {
                    "mode": t.mode if t else "OFFLINE",
                    "lat": t.latitude if t else 0.0,
                    "lon": t.longitude if t else 0.0,
                    "alt_m": t.altitude_agl_m if t else 0.0,
                    "battery": t.battery_fraction if t else 0.0,
                    "is_mock": t.is_mock if t else True,
                }
                for d_id, t in telemetries.items()
            },
        }
        return status_summary

    def assign_rescue_task(
        self, task_id: str, target_lat: float, target_lon: float, target_alt_m: float = 5.0
    ) -> Optional[str]:
        """Assign a rescue delivery task to the highest-battery ready delivery drone."""
        best_drone = None
        best_battery = -1.0
        for drone_id, ctrl in self.delivery_controllers.items():
            if ctrl.current_task is None:
                adapter = self.adapters.get(drone_id)
                telem = adapter.get_telemetry() if adapter else None
                batt = telem.battery_fraction if telem else 1.0
                if batt > best_battery and batt >= 0.20:
                    best_battery = batt
                    best_drone = drone_id
        if best_drone is not None:
            self.delivery_controllers[best_drone].assign_task(
                task_id=task_id, target_lat=target_lat, target_lon=target_lon, target_alt_m=target_alt_m
            )
            logger.info("Assigned rescue task %s to delivery drone %s", task_id, best_drone)
            return best_drone
        logger.warning("No ready delivery drone available for task %s", task_id)
        return None

    def _trigger_fleet_rtl(self) -> None:
        """Command all 6 vehicles to execute safe Return-to-Launch."""
        logger.warning("Triggering FLEET RETURN-TO-LAUNCH failsafe.")
        for drone_id, adapter in self.adapters.items():
            adapter.send_safety_action("RTL")

    def shutdown(self) -> None:
        """Safely disconnect and release resources for all vehicles."""
        logger.info("Shutting down FleetMissionManager...")
        for adapter in self.adapters.values():
            adapter.disconnect()


def main(args: Optional[List[str]] = None) -> None:
    """ROS 2 Node Entry Point."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        rclpy.init(args=args)

        class MissionManagerROS2Node(Node):
            def __init__(self) -> None:
                super().__init__("mission_manager_node")
                self.declare_parameter("config_path", "config/fleet.json")
                self.declare_parameter("adapter_type", "mock")
                self.declare_parameter("timeout_s", 300.0)

                cfg_path = self.get_parameter("config_path").get_parameter_value().string_value
                ad_type_str = self.get_parameter("adapter_type").get_parameter_value().string_value
                timeout = self.get_parameter("timeout_s").get_parameter_value().double_value

                ad_type = AdapterType(ad_type_str)
                self.manager = FleetMissionManager(
                    fleet_config_path=cfg_path,
                    adapter_type=ad_type,
                    mission_timeout_s=timeout,
                    ros2_node=self,
                )

                self.pub_status = self.create_publisher(String, "/rescue_swarm/mission_status", 10)
                self.sub_assign = self.create_subscription(
                    String, "/rescue_swarm/assign_task", self.assign_task_callback, 10
                )
                self.timer = self.create_timer(0.2, self.timer_callback)  # 5 Hz loop
                self.get_logger().info("MissionManagerROS2Node initialized and running at 5Hz.")

            def assign_task_callback(self, msg: Any) -> None:
                try:
                    data = json.loads(msg.data)
                    self.manager.assign_rescue_task(
                        task_id=str(data.get("task_id", "task_001")),
                        target_lat=float(data["lat"]),
                        target_lon=float(data["lon"]),
                        target_alt_m=float(data.get("alt", 5.0)),
                    )
                except Exception as e:
                    self.get_logger().error(f"Error processing task assignment: {e}")

            def timer_callback(self) -> None:
                status = self.manager.step()
                msg = String()
                msg.data = json.dumps(status)
                self.pub_status.publish(msg)

        node = MissionManagerROS2Node()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except ImportError:
        logger.warning("rclpy not installed; MissionManager operating in standalone Python mode.")


if __name__ == "__main__":
    main()
