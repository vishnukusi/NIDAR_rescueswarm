"""Comprehensive deterministic tests for Survivor Association and Deduplication.

Covers all 8 required scenarios:
1. Repeated observation of stationary survivor from same drone (no duplicates).
2. Cross-drone consistent observation of same survivor (confirming drones merged).
3. Distinct nearby survivors (< 10m) not merged (same-frame & distinct flags).
4. Uncertain localization gating (ambiguity policy, safe handling, invalid rejection).
5. Multiple detections in one frame (distinct tasks without duplicate allocations).
6. Task lifecycle (assigned / en-route repeated observation safety).
7. Resolved tasks (verified tasks not reopened; failed tasks retried).
8. Invalid input rejection (NaN, Inf, None coordinates, fleet untouched).
"""

import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core import (
    AssociationDecision,
    AssociationStatus,
    BoundingBox,
    Detection,
    Drone,
    DroneRole,
    DroneStatus,
    GeolocationEstimate,
    GeoPoint,
    MissionManager,
    MissionState,
    RescueTask,
    SurvivorAssociator,
    SurvivorRecord,
    SwarmManager,
    TaskStatus,
)


class SurvivorAssociationTests(unittest.TestCase):
    """Deterministic validation of survivor association and duplicate task prevention."""

    def setUp(self):
        self.drones = [
            Drone("S1", DroneRole.SEARCH, DroneStatus.STANDBY),
            Drone("S2", DroneRole.SEARCH, DroneStatus.STANDBY),
            Drone("S3", DroneRole.SEARCH, DroneStatus.STANDBY),
            Drone("D1", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.75),
            Drone("D2", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.95),
            Drone("D3", DroneRole.DELIVERY, DroneStatus.READY, battery_fraction=0.85),
        ]
        self.mission = MissionManager()
        self.mission.transition_to(MissionState.INITIALIZING)
        self.mission.transition_to(MissionState.SEARCHING)

        self.manager = SwarmManager(
            drones=self.drones,
            mission_manager=self.mission,
            duplicate_distance_threshold_m=10.0,
        )

    def _make_det(self, conf: float = 0.90, cx: float = 0.5, cy: float = 0.5, w: float = 0.1, h: float = 0.1) -> Detection:
        return Detection(
            class_id=0,
            class_name="person",
            confidence=conf,
            bbox=BoundingBox(
                xmin=max(0.0, cx - w / 2.0),
                ymin=max(0.0, cy - h / 2.0),
                xmax=min(1.0, cx + w / 2.0),
                ymax=min(1.0, cy + h / 2.0),
            ),
        )

    def _make_geo(self, lat: float = 12.971600, lon: float = 77.594600, uncertainty: float = 2.0) -> GeolocationEstimate:
        return GeolocationEstimate(
            location=GeoPoint(latitude=lat, longitude=lon, altitude_m=0.0),
            offset_north_m=0.0,
            offset_east_m=0.0,
            ground_distance_m=0.0,
            slant_range_m=30.0,
            uncertainty_radius_m=uncertainty,
        )

    # -------------------------------------------------------------------------
    # Scenario 1: Repeated observation from same drone
    # -------------------------------------------------------------------------
    def test_scenario_1_repeated_observation_same_drone(self):
        """Repeated observations of one stationary survivor must not create duplicate tasks."""
        # Observation 1 at T=0
        task1, is_new1 = self.manager.process_detection(
            self._make_det(0.85),
            self._make_geo(12.971600, 77.594600, uncertainty=3.0),
            detected_by_drone_id="S1",
            timestamp=10.0,
        )
        self.assertTrue(is_new1)
        self.assertEqual(task1.task_id, "task_001")
        self.assertEqual(task1.assigned_drone_id, "D2")  # Highest battery (0.95)

        # Observation 2 at T=1s (slight sensor jitter, ~0.7m away, better uncertainty)
        task2, is_new2 = self.manager.process_detection(
            self._make_det(0.92),
            self._make_geo(12.971605, 77.594605, uncertainty=1.5),
            detected_by_drone_id="S1",
            timestamp=11.0,
        )
        self.assertFalse(is_new2)
        self.assertEqual(task2.task_id, "task_001")
        self.assertEqual(task2.confidence, 0.92)  # Updated to higher confidence

        # Observation 3 at T=2s (another sighting)
        task3, is_new3 = self.manager.process_detection(
            self._make_det(0.88),
            self._make_geo(12.971602, 77.594601, uncertainty=2.5),
            detected_by_drone_id="S1",
            timestamp=12.0,
        )
        self.assertFalse(is_new3)
        self.assertEqual(task3.task_id, "task_001")

        # Verify only 1 active task exists across the swarm
        self.assertEqual(len(self.manager.get_all_tasks()), 1)
        self.assertEqual(len(self.manager.get_active_tasks()), 1)
        record = self.manager.survivor_records["task_001"]
        self.assertEqual(record.detection_count, 3)
        # Location was refined to the lowest uncertainty observation (1.5m at lat 12.971605)
        self.assertEqual(record.uncertainty_radius_m, 1.5)
        self.assertAlmostEqual(record.task.location.latitude, 12.971605, places=6)

        # Delivery allocation: exactly 1 drone remains allocated
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.BUSY)
        self.assertEqual(self.manager.get_drone("D3").status, DroneStatus.READY)

    # -------------------------------------------------------------------------
    # Scenario 2: Cross-drone observation consistency
    # -------------------------------------------------------------------------
    def test_scenario_2_cross_drone_observation_consistency(self):
        """Two search drones observing the same survivor must merge without duplicate tasks."""
        # Drone S1 detects survivor at Pos A
        task1, is_new1 = self.manager.process_detection(
            self._make_det(0.80),
            self._make_geo(12.971600, 77.594600, uncertainty=2.5),
            detected_by_drone_id="S1",
        )
        self.assertTrue(is_new1)
        self.assertEqual(task1.task_id, "task_001")

        # Drone S2 observes same survivor from another angle (~2.7m away, conf 0.94)
        task2, is_new2 = self.manager.process_detection(
            self._make_det(0.94),
            self._make_geo(12.971620, 77.594615, uncertainty=2.0),
            detected_by_drone_id="S2",
        )
        self.assertFalse(is_new2)
        self.assertEqual(task2.task_id, "task_001")
        self.assertEqual(task2.confidence, 0.94)

        # Confirming drones list updated to contain both S1 and S2
        record = self.manager.survivor_records["task_001"]
        self.assertIn("S1", record.confirming_drones)
        self.assertIn("S2", record.confirming_drones)
        self.assertEqual(len(record.confirming_drones), 2)
        self.assertEqual(len(self.manager.get_all_tasks()), 1)

    # -------------------------------------------------------------------------
    # Scenario 3: Distinct nearby survivors (< 10m) not merged
    # -------------------------------------------------------------------------
    def test_scenario_3_distinct_nearby_survivors_same_frame(self):
        """Two distinct survivors < 10m apart in the SAME frame must not be merged."""
        # Survivor A at (12.971600, 77.594600)
        # Survivor B at (12.971640, 77.594620) -> separation ~ 4.9 meters!
        geo_A = self._make_geo(12.971600, 77.594600)
        geo_B = self._make_geo(12.971640, 77.594620)
        det_A = self._make_det(0.90, cx=0.3, cy=0.5)
        det_B = self._make_det(0.88, cx=0.7, cy=0.5)

        # Process as batch from same frame
        results = self.manager.process_detections_batch(
            detections_with_geo=[(det_A, geo_A), (det_B, geo_B)],
            detected_by_drone_id="S1",
            frame_id="frame_0042",
        )

        task_A, is_new_A = results[0]
        task_B, is_new_B = results[1]

        # Both must be created as new distinct tasks!
        self.assertTrue(is_new_A, "Survivor A must create a task")
        self.assertTrue(is_new_B, "Survivor B must create a distinct task despite being 4.9m away")
        self.assertNotEqual(task_A.task_id, task_B.task_id)

        # Both must have distinct delivery drones allocated
        self.assertEqual(len(self.manager.get_all_tasks()), 2)
        self.assertEqual(task_A.assigned_drone_id, "D2")  # 0.95 battery
        self.assertEqual(task_B.assigned_drone_id, "D3")  # 0.85 battery
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.BUSY)
        self.assertEqual(self.manager.get_drone("D3").status, DroneStatus.BUSY)

    def test_scenario_3_distinct_nearby_survivors_explicit_flag(self):
        """Two distinct survivors < 10m apart marked as known distinct must not be merged."""
        geo_A = self._make_geo(12.971600, 77.594600)
        task_A, _ = self.manager.process_detection(self._make_det(0.90), geo_A, "S1")

        # Survivor B detected 4.5m away, explicitly declared distinct from task_A
        geo_B = self._make_geo(12.971635, 77.594625)
        task_B, is_new_B = self.manager.process_detection(
            self._make_det(0.85),
            geo_B,
            "S2",
            known_distinct_from=[task_A.task_id],
        )

        self.assertTrue(is_new_B)
        self.assertNotEqual(task_A.task_id, task_B.task_id)
        self.assertEqual(len(self.manager.get_all_tasks()), 2)

    # -------------------------------------------------------------------------
    # Scenario 4: Uncertain localization policy
    # -------------------------------------------------------------------------
    def test_scenario_4_excessive_uncertainty_flagged_as_ambiguous(self):
        """An observation with excessive uncertainty must not overwrite a refined location."""
        # Initial confident detection (uncertainty = 2.0m)
        task1, is_new1 = self.manager.process_detection(
            self._make_det(0.90),
            self._make_geo(12.971600, 77.594600, uncertainty=2.0),
            "S1",
        )
        self.assertTrue(is_new1)

        # High-uncertainty observation (uncertainty = 35.0m, > 20.0m threshold)
        # Position is 6m away from task1
        noisy_geo = self._make_geo(12.971650, 77.594610, uncertainty=35.0)
        task2, is_new2 = self.manager.process_detection(
            self._make_det(0.85),
            noisy_geo,
            "S2",
        )
        self.assertFalse(is_new2)
        self.assertEqual(task2.task_id, "task_001")

        # Refined location must NOT have been overwritten with the noisy coordinates
        record = self.manager.survivor_records["task_001"]
        self.assertAlmostEqual(record.task.location.latitude, 12.971600, places=6)
        self.assertEqual(record.uncertainty_radius_m, 2.0)
        # Ambiguous sighting recorded
        self.assertEqual(record.ambiguous_sightings, 1)
        self.assertGreater(len(record.ambiguity_notes), 0)

    def test_scenario_4_invalid_uncertainty_raises_error(self):
        """Negative, NaN, or infinite uncertainty must be rejected immediately."""
        with self.assertRaises(ValueError):
            self._make_geo(uncertainty=-5.0)
        with self.assertRaises(ValueError):
            self._make_geo(uncertainty=float("nan"))
        with self.assertRaises(ValueError):
            self._make_geo(uncertainty=float("inf"))

    # -------------------------------------------------------------------------
    # Scenario 5: Multiple detections in one frame
    # -------------------------------------------------------------------------
    def test_scenario_5_multiple_detections_in_one_frame_no_duplicate_allocation(self):
        """Frame with two real people and one duplicate box assigns exactly 2 delivery drones."""
        # Person 1 at (12.97160, 77.59460)
        geo_p1 = self._make_geo(12.971600, 77.594600)
        det_p1 = self._make_det(0.92, cx=0.2, cy=0.5)

        # Person 2 at (12.97175, 77.59460) -> 16.6m away (distinct)
        geo_p2 = self._make_geo(12.971750, 77.594600)
        det_p2 = self._make_det(0.89, cx=0.8, cy=0.5)

        # Duplicate box on Person 1 (jitter box, 0.5m away)
        geo_p1_dup = self._make_geo(12.971604, 77.594602)
        det_p1_dup = self._make_det(0.85, cx=0.21, cy=0.51)

        # Process all 3 in the same frame
        results = self.manager.process_detections_batch(
            detections_with_geo=[
                (det_p1, geo_p1),
                (det_p2, geo_p2),
                (det_p1_dup, geo_p1_dup),
            ],
            detected_by_drone_id="S1",
            frame_id="frame_0088",
        )

        # First two are new tasks
        self.assertTrue(results[0][1], "Person 1 is a new task")
        self.assertTrue(results[1][1], "Person 2 is a new task")
        # Third is a duplicate of Person 1
        self.assertFalse(results[2][1], "Duplicate box must match Person 1")
        self.assertEqual(results[2][0].task_id, results[0][0].task_id)

        # Total tasks must be exactly 2
        self.assertEqual(len(self.manager.get_all_tasks()), 2)
        # Exactly 2 delivery drones allocated (D2, D3); D1 remains READY
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.BUSY)
        self.assertEqual(self.manager.get_drone("D3").status, DroneStatus.BUSY)
        self.assertEqual(self.manager.get_drone("D1").status, DroneStatus.READY)

    # -------------------------------------------------------------------------
    # Scenario 6: Task lifecycle: Assigned / En-Route safe
    # -------------------------------------------------------------------------
    def test_scenario_6_task_lifecycle_en_route_safe(self):
        """Repeated observation of en-route survivor must not disrupt delivery or create duplicate."""
        task1, _ = self.manager.process_detection(self._make_det(0.85), self._make_geo(), "S1")
        self.assertEqual(task1.status, TaskStatus.ASSIGNED)
        self.assertEqual(task1.assigned_drone_id, "D2")

        # Simulate drone launching and entering EN_ROUTE
        self.manager.update_task_status(task1.task_id, TaskStatus.EN_ROUTE)
        self.assertEqual(task1.status, TaskStatus.EN_ROUTE)

        # Search drone S2 spots survivor again while delivery is in flight
        task_repeat, is_new = self.manager.process_detection(self._make_det(0.95), self._make_geo(), "S2")

        self.assertFalse(is_new)
        self.assertEqual(task_repeat.task_id, task1.task_id)
        # Task remains in EN_ROUTE
        self.assertEqual(task_repeat.status, TaskStatus.EN_ROUTE)
        self.assertEqual(task_repeat.assigned_drone_id, "D2")
        self.assertEqual(len(self.manager.get_all_tasks()), 1)

    # -------------------------------------------------------------------------
    # Scenario 7: Resolved tasks (Verified and Failed)
    # -------------------------------------------------------------------------
    def test_scenario_7a_verified_task_not_reopened(self):
        """Observing a verified survivor records the sighting without reopening task or dropping again."""
        task1, _ = self.manager.process_detection(self._make_det(0.90), self._make_geo(), "S1")
        # Mark as VERIFIED (rescued)
        self.manager.update_task_status(task1.task_id, TaskStatus.VERIFIED)
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.READY)

        # Search drone detects survivor at same location again
        task_repeat, is_new = self.manager.process_detection(self._make_det(0.92), self._make_geo(), "S3")

        self.assertFalse(is_new)
        self.assertEqual(task_repeat.task_id, task1.task_id)
        # Status MUST remain VERIFIED (not reset to GEOLOCATED or ASSIGNED)
        self.assertEqual(task_repeat.status, TaskStatus.VERIFIED)
        # No delivery drone is dispatched
        self.assertEqual(len(self.manager.get_active_tasks()), 0)
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.READY)
        record = self.manager.survivor_records[task1.task_id]
        self.assertEqual(record.verified_re_sightings, 1)

    def test_scenario_7b_failed_task_triggers_retry(self):
        """Observing a previously failed survivor triggers a retry task rather than abandoning."""
        task1, _ = self.manager.process_detection(self._make_det(0.85), self._make_geo(), "S1")
        # Delivery failed (e.g. timeout) — drone is confirmed recovered at base
        self.manager.update_task_status(task1.task_id, TaskStatus.FAILED, drone_available=True)
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.READY)

        # Search drone spots the survivor again
        task_retry, is_new = self.manager.process_detection(self._make_det(0.91), self._make_geo(), "S2")

        self.assertTrue(is_new, "A new rescue task must be created to retry the failed delivery")
        self.assertNotEqual(task_retry.task_id, task1.task_id)
        self.assertEqual(task_retry.retry_of, task1.task_id)
        self.assertEqual(task_retry.status, TaskStatus.ASSIGNED)
        # Assigned to highest available battery delivery drone (D2)
        self.assertEqual(task_retry.assigned_drone_id, "D2")
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.BUSY)
        # Both task_001 (FAILED) and task_002 (ASSIGNED) exist in record
        self.assertEqual(len(self.manager.get_all_tasks()), 2)

    # -------------------------------------------------------------------------
    # Scenario 8: Invalid input rejection
    # -------------------------------------------------------------------------
    def test_scenario_8_invalid_input_rejection(self):
        """Invalid, NaN, infinite, or missing coordinates must be rejected before association."""
        valid_det = self._make_det()

        # None geo_estimate
        with self.assertRaises(ValueError):
            self.manager.process_detection(valid_det, None, "S1")

        # None location
        corrupt_geo = type("CorruptGeo", (), {"location": None, "uncertainty_radius_m": 2.0})()
        with self.assertRaises(ValueError):
            self.manager.process_detection(valid_det, corrupt_geo, "S1")

        # NaN coordinates
        with self.assertRaises(ValueError):
            self._make_geo(lat=float("nan"))
        with self.assertRaises(ValueError):
            self._make_geo(lon=float("inf"))

        # None detection
        with self.assertRaises(ValueError):
            self.manager.process_detection(None, self._make_geo(), "S1")

        # Verify swarm state remained 100% clean
        self.assertEqual(len(self.manager.get_all_tasks()), 0)
        self.assertEqual(len(self.manager.get_active_tasks()), 0)
        for d in self.drones:
            if d.role == DroneRole.DELIVERY:
                self.assertEqual(d.status, DroneStatus.READY)
                self.assertIsNone(d.active_task_id)


if __name__ == "__main__":
    unittest.main()
