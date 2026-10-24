"""Simulated delivery adapter and verification node for NIDAR RescueSwarm.

CRITICAL SAFETY & INTEGRATION SPECIFICATION:
This module is STRICTLY A SIMULATION ADAPTER.
Physical payload release mechanisms and physical verification sensors are NOT
implemented or physically actuated.
All verification states, drop acknowledgements, and outcome classifications
are explicitly tagged with 'SIMULATION_ONLY'.
"""

from dataclasses import dataclass, field
from enum import Enum
import json
import logging
import math
import time
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


class SimulatedDeliveryVerificationMode(str, Enum):
    """Supported simulation verification modes."""
    PROXIMITY_AND_HOVER = "proximity_and_hover"
    TIMER_VERIFIED = "timer_verified"
    FAIL_FIRST_THEN_RETRY = "fail_first_then_retry"
    PERMANENT_FAILURE = "permanent_failure"


class SimulatedDeliveryStage(str, Enum):
    """Lifecycle stages for simulated delivery mission."""
    IDLE = "IDLE"
    ASSIGNED = "ASSIGNED"
    INGRESS = "INGRESS"
    HOVER_DESCENT = "HOVER_DESCENT"
    SIMULATED_PAYLOAD_DROP = "SIMULATED_PAYLOAD_DROP"
    SIMULATED_VERIFYING = "SIMULATED_VERIFYING"
    VERIFIED = "VERIFIED"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


@dataclass
class DeliveryTaskState:
    """State record for an active delivery assignment."""
    task_id: str
    drone_id: str
    target_lat: float
    target_lon: float
    target_alt_m: float = 5.0
    stage: SimulatedDeliveryStage = SimulatedDeliveryStage.ASSIGNED
    assigned_time: float = 0.0
    drop_attempt_time: float = 0.0
    verified_time: float = 0.0
    attempt_count: int = 1
    failure_reason: str = ""
    is_simulation: bool = True
    verification_type: str = "SIMULATION_VERIFIED"


class SimulatedDeliveryController:
    """Core logic managing delivery state progression in simulation."""

    def __init__(
        self,
        drone_id: str,
        verification_mode: SimulatedDeliveryVerificationMode = SimulatedDeliveryVerificationMode.PROXIMITY_AND_HOVER,
        drop_hover_duration_s: float = 2.0,
        arrival_radius_m: float = 2.5,
        target_delivery_alt_m: float = 5.0,
    ) -> None:
        self.drone_id = drone_id
        self.verification_mode = verification_mode
        self.drop_hover_duration_s = drop_hover_duration_s
        self.arrival_radius_m = arrival_radius_m
        self.target_delivery_alt_m = target_delivery_alt_m

        self.current_task: Optional[DeliveryTaskState] = None
        self.completed_tasks: List[DeliveryTaskState] = []
        self._hover_start_time: Optional[float] = None
        self._has_failed_once: bool = False

    def assign_task(
        self,
        task_id: str,
        target_lat: float,
        target_lon: float,
        target_alt_m: float = 5.0,
        current_time: Optional[float] = None,
        attempt: int = 1,
    ) -> DeliveryTaskState:
        """Assign a new or retried rescue delivery task to this vehicle."""
        now = current_time if current_time is not None else time.time()
        task = DeliveryTaskState(
            task_id=task_id,
            drone_id=self.drone_id,
            target_lat=target_lat,
            target_lon=target_lon,
            target_alt_m=target_alt_m,
            stage=SimulatedDeliveryStage.INGRESS,
            assigned_time=now,
            attempt_count=attempt,
            is_simulation=True,
            verification_type=f"SIMULATION_{self.verification_mode.value.upper()}",
        )
        self.current_task = task
        self._hover_start_time = None
        logger.info(
            "[%s] [SIMULATED_DELIVERY] Assigned task %s (attempt %d) at (%.6f, %.6f, %.1fm)",
            self.drone_id, task_id, attempt, target_lat, target_lon, target_alt_m
        )
        return task

    def update(
        self,
        current_lat: float,
        current_lon: float,
        current_alt_m: float,
        current_time: Optional[float] = None,
    ) -> Optional[DeliveryTaskState]:
        """Progress delivery task based on simulated vehicle position."""
        if not self.current_task:
            return None

        now = current_time if current_time is not None else time.time()
        task = self.current_task

        # Ingress / Approach check
        dist_m = self._haversine_distance_m(
            current_lat, current_lon, task.target_lat, task.target_lon
        )

        if task.stage == SimulatedDeliveryStage.INGRESS:
            if dist_m <= self.arrival_radius_m:
                task.stage = SimulatedDeliveryStage.HOVER_DESCENT
                self._hover_start_time = now
                logger.info("[%s] [SIMULATED_DELIVERY] Arrived at target zone, commencing descent/hover", self.drone_id)

        elif task.stage == SimulatedDeliveryStage.HOVER_DESCENT:
            # Check altitude and hover duration
            alt_ok = current_alt_m <= (task.target_alt_m + 1.5)
            if self._hover_start_time and (now - self._hover_start_time >= self.drop_hover_duration_s) and alt_ok:
                task.stage = SimulatedDeliveryStage.SIMULATED_PAYLOAD_DROP
                task.drop_attempt_time = now
                logger.info("[%s] [SIMULATED_DELIVERY] Triggering simulated payload drop", self.drone_id)

        elif task.stage in (SimulatedDeliveryStage.SIMULATED_PAYLOAD_DROP, SimulatedDeliveryStage.SIMULATED_VERIFYING):
            task.stage = SimulatedDeliveryStage.SIMULATED_VERIFYING
            logger.info("[%s] [SIMULATED_DELIVERY] Verifying simulated drop outcome...", self.drone_id)

            # Evaluate verification according to mode
            if self.verification_mode == SimulatedDeliveryVerificationMode.PERMANENT_FAILURE:
                task.stage = SimulatedDeliveryStage.FAILED
                task.failure_reason = "SIMULATED_PAYLOAD_RELEASE_JAM"
                logger.warning("[%s] [SIMULATED_DELIVERY] Verification FAILED: %s", self.drone_id, task.failure_reason)
                self.completed_tasks.append(task)
                self.current_task = None
            elif self.verification_mode == SimulatedDeliveryVerificationMode.FAIL_FIRST_THEN_RETRY and not self._has_failed_once:
                self._has_failed_once = True
                task.stage = SimulatedDeliveryStage.FAILED
                task.failure_reason = "SIMULATED_DELIVERY_DISPERSION_EXCEEDED"
                logger.warning("[%s] [SIMULATED_DELIVERY] First attempt FAILED for retry test: %s", self.drone_id, task.failure_reason)
                self.completed_tasks.append(task)
                self.current_task = None
            else:
                # Success
                task.stage = SimulatedDeliveryStage.VERIFIED
                task.verified_time = now
                logger.info("[%s] [SIMULATED_DELIVERY] Task %s VERIFIED (SIMULATED)", self.drone_id, task.task_id)
                self.completed_tasks.append(task)
                self.current_task = None

        return task

    def abort_task(self, reason: str = "MISSION_ABORT") -> Optional[DeliveryTaskState]:
        """Abort currently active task."""
        if not self.current_task:
            return None
        self.current_task.stage = SimulatedDeliveryStage.ABORTED
        self.current_task.failure_reason = reason
        self.completed_tasks.append(self.current_task)
        aborted = self.current_task
        self.current_task = None
        return aborted

    @staticmethod
    def _haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        r = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2.0) ** 2
        return r * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def main(args: Optional[List[str]] = None) -> None:
    """ROS 2 Node Entry Point."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        rclpy.init(args=args)

        class SimulatedDeliveryNode(Node):
            def __init__(self) -> None:
                super().__init__("simulated_delivery_node")
                self.declare_parameter("drone_id", "D1")
                self.declare_parameter("verification_mode", "proximity_and_hover")
                drone_id = self.get_parameter("drone_id").get_parameter_value().string_value
                mode_str = self.get_parameter("verification_mode").get_parameter_value().string_value

                mode = SimulatedDeliveryVerificationMode(mode_str)
                self.controller = SimulatedDeliveryController(drone_id, verification_mode=mode)
                self.publisher_ = self.create_publisher(String, f"/{drone_id.lower()}/delivery_status", 10)
                self.get_logger().info(f"Started SimulatedDeliveryNode for {drone_id} (mode={mode_str})")

        node = SimulatedDeliveryNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except ImportError:
        logger.warning("rclpy not installed; running in standalone mode.")


if __name__ == "__main__":
    main()
