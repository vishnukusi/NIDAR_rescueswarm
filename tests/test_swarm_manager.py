import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.localization import GeolocationEstimate
from rescue_swarm_core.mission import MissionManager
from rescue_swarm_core.models import (
    Drone,
    DroneRole,
    DroneStatus,
    GeoPoint,
    MissionState,
    TaskStatus,
)
from rescue_swarm_core.perception import BoundingBox, Detection
from rescue_swarm_core.swarm_manager import SwarmManager


class SwarmManagerTests(unittest.TestCase):
    def setUp(self):
        self.drones = [
            Drone("S1", DroneRole.SEARCH, DroneStatus.STANDBY),
            Drone("S2", DroneRole.SEARCH, DroneStatus.STANDBY),
            Drone("S3", DroneRole.SEARCH, DroneStatus.STANDBY),
            Drone("D1", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.75),
            Drone("D2", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.92),
            Drone("D3", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.80),
        ]
        self.mission = MissionManager()
        self.mission.transition_to(MissionState.INITIALIZING)
        self.mission.transition_to(MissionState.SEARCHING)

        self.manager = SwarmManager(
            drones=self.drones,
            mission_manager=self.mission,
            duplicate_distance_threshold_m=10.0,
        )

    def _make_detection(self, conf: float = 0.90) -> Detection:
        return Detection(
            class_id=0,
            class_name="person",
            confidence=conf,
            bbox=BoundingBox(xmin=0.45, ymin=0.45, xmax=0.55, ymax=0.55),
        )

    def _make_geo_estimate(self, lat: float = 12.9716, lon: float = 77.5946, uncertainty: float = 2.0) -> GeolocationEstimate:
        return GeolocationEstimate(
            location=GeoPoint(latitude=lat, longitude=lon, altitude_m=0.0),
            offset_north_m=0.0,
            offset_east_m=0.0,
            ground_distance_m=0.0,
            slant_range_m=30.0,
            uncertainty_radius_m=uncertainty,
        )

    def test_new_detection_allocates_highest_battery_delivery_drone(self):
        det = self._make_detection(0.88)
        geo = self._make_geo_estimate(12.9716, 77.5946)

        task, is_new = self.manager.process_detection(det, geo, detected_by_drone_id="S1")
        self.assertTrue(is_new)
        self.assertEqual(task.status, TaskStatus.ASSIGNED)
        # D2 has the highest battery (0.92)
        self.assertEqual(task.assigned_drone_id, "D2")
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.BUSY)
        self.assertEqual(self.manager.get_drone("D2").active_task_id, task.task_id)
        # Mission transitioned to rescue in progress
        self.assertEqual(self.mission.state, MissionState.RESCUE_IN_PROGRESS)

    def test_spatial_deduplication_prevents_duplicate_tasks(self):
        det1 = self._make_detection(0.85)
        geo1 = self._make_geo_estimate(12.971600, 77.594600)
        task1, is_new1 = self.manager.process_detection(det1, geo1, "S1")
        self.assertTrue(is_new1)
        self.assertEqual(task1.assigned_drone_id, "D2")

        # Second detection slightly shifted (~2 meters away) from S2
        det2 = self._make_detection(0.95)
        geo2 = self._make_geo_estimate(12.971615, 77.594610)
        task2, is_new2 = self.manager.process_detection(det2, geo2, "S2")

        self.assertFalse(is_new2)
        self.assertEqual(task2.task_id, task1.task_id)
        self.assertEqual(task2.confidence, 0.95)  # Updated to higher confidence
        self.assertEqual(task2.assigned_drone_id, "D2")  # No second drone assigned

        # Only one task should exist
        self.assertEqual(len(self.manager.get_all_tasks()), 1)

    def test_distinct_detection_allocates_next_drone(self):
        # First survivor
        task1, _ = self.manager.process_detection(
            self._make_detection(0.90),
            self._make_geo_estimate(12.9716, 77.5946),
            "S1",
        )
        self.assertEqual(task1.assigned_drone_id, "D2")

        # Second survivor ~200 meters away
        task2, is_new = self.manager.process_detection(
            self._make_detection(0.87),
            self._make_geo_estimate(12.9734, 77.5946),
            "S2",
        )
        self.assertTrue(is_new)
        # Next highest battery is D3 (0.80)
        self.assertEqual(task2.assigned_drone_id, "D3")
        self.assertEqual(len(self.manager.get_all_tasks()), 2)

    def test_task_verification_releases_drone_and_dispatches_queue(self):
        # Assign all 3 delivery drones
        self.manager.process_detection(self._make_detection(), self._make_geo_estimate(12.971, 77.591), "S1") # D2
        self.manager.process_detection(self._make_detection(), self._make_geo_estimate(12.972, 77.592), "S2") # D3
        self.manager.process_detection(self._make_detection(), self._make_geo_estimate(12.973, 77.593), "S3") # D1

        # Fourth detection has no delivery drones left available
        task4, _ = self.manager.process_detection(
            self._make_detection(), self._make_geo_estimate(12.974, 77.594), "S1"
        )
        self.assertEqual(task4.status, TaskStatus.GEOLOCATED)
        self.assertIsNone(task4.assigned_drone_id)

        # Mark first task as VERIFIED
        self.manager.update_task_status("task_001", TaskStatus.VERIFIED)

        # D2 should have been released and immediately allocated to task4!
        self.assertEqual(task4.status, TaskStatus.ASSIGNED)
        self.assertEqual(task4.assigned_drone_id, "D2")
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.BUSY)

    def test_low_confidence_rejected(self):
        low_det = self._make_detection(0.30)
        with self.assertRaises(ValueError):
            self.manager.process_detection(low_det, self._make_geo_estimate(), "S1", minimum_confidence=0.50)


if __name__ == "__main__":
    unittest.main()
