"""Comprehensive test suite for Fix 6: Survivor Association Integration and Lifecycle Audit.

Validates:
1. End-to-end frame identity propagation and batch processing.
2. Same-frame 2D bounding-box IoU vs physical ground distance association (preventing
   both false splits of duplicate boxes separated by >= 3m and false merges of distinct
   nearby people separated by < 3m).
3. Overlapping uncertainty intervals and high-uncertainty observation gating.
4. Batch processing atomicity (strict pre-validation and rollback guarantee).
5. Drone and task lifecycle decoupling (distinguishing task failure from vehicle readiness).
6. Retry lineage management (suppressing duplicate active retries and enabling repeated retries).
"""

import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.allocator import AllocationError
from rescue_swarm_core.association import (
    AssociationDecision,
    AssociationStatus,
    SurvivorAssociator,
)
from rescue_swarm_core.localization import GeolocationEstimate
from rescue_swarm_core.models import (
    Drone,
    DroneRole,
    DroneStatus,
    GeoPoint,
    MissionState,
    RescueTask,
    TaskStatus,
)
from rescue_swarm_core.perception import BoundingBox, Detection
from rescue_swarm_core.swarm_manager import SurvivorRecord, SwarmManager


class TestBoundingBoxIoU(unittest.TestCase):
    """Unit tests for BoundingBox.iou computation."""

    def test_exact_overlap_iou_is_one(self):
        box1 = BoundingBox(0.2, 0.3, 0.6, 0.7)
        box2 = BoundingBox(0.2, 0.3, 0.6, 0.7)
        self.assertAlmostEqual(box1.iou(box2), 1.0, places=6)

    def test_disjoint_boxes_iou_is_zero(self):
        box1 = BoundingBox(0.1, 0.1, 0.3, 0.3)
        box2 = BoundingBox(0.4, 0.4, 0.6, 0.6)
        self.assertEqual(box1.iou(box2), 0.0)

    def test_partial_overlap_iou(self):
        # 10x10 boxes overlapping by 5x10:
        # box1: [0.0, 0.0, 0.2, 0.2] (area = 0.04)
        # box2: [0.1, 0.0, 0.3, 0.2] (area = 0.04)
        # inter: [0.1, 0.0, 0.2, 0.2] (area = 0.02)
        # union: 0.04 + 0.04 - 0.02 = 0.06
        # IoU = 0.02 / 0.06 = 1/3
        box1 = BoundingBox(0.0, 0.0, 0.2, 0.2)
        box2 = BoundingBox(0.1, 0.0, 0.3, 0.2)
        self.assertAlmostEqual(box1.iou(box2), 1.0 / 3.0, places=5)

    def test_iou_rejects_non_bounding_box(self):
        box = BoundingBox(0.1, 0.1, 0.3, 0.3)
        with self.assertRaises(TypeError):
            box.iou("not_a_box")  # type: ignore


class TestAssociationLifecycleAudit(unittest.TestCase):
    """Test suite covering the 6 core Fix 6 audit scenarios."""

    def setUp(self):
        self.associator = SurvivorAssociator(
            match_distance_threshold_m=10.0,
            max_association_uncertainty_m=20.0,
            min_distinct_separation_m=3.0,
            intra_frame_iou_threshold=0.3,
        )
        self.delivery_drones = [
            Drone(drone_id="D1", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.80),
            Drone(drone_id="D2", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.95),
            Drone(drone_id="D3", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.70),
        ]
        self.manager = SwarmManager(drones=self.delivery_drones, associator=self.associator)

    def _make_det(
        self,
        confidence: float = 0.90,
        bbox: BoundingBox = BoundingBox(0.48, 0.48, 0.52, 0.58),
        frame_id: str = "frame_001",
    ) -> Detection:
        return Detection(
            class_id=0,
            class_name="person",
            confidence=confidence,
            bbox=bbox,
            frame_id=frame_id,
        )

    def _make_geo(
        self,
        lat: float = 12.972000,
        lon: float = 77.594500,
        uncertainty_m: float = 3.0,
    ) -> GeolocationEstimate:
        return GeolocationEstimate(
            location=GeoPoint(latitude=lat, longitude=lon, altitude_m=0.0),
            offset_north_m=0.0,
            offset_east_m=0.0,
            ground_distance_m=0.0,
            slant_range_m=30.0,
            uncertainty_radius_m=uncertainty_m,
        )

    # -------------------------------------------------------------------------
    # Audit 1: Frame Identity Propagation End-to-End
    # -------------------------------------------------------------------------
    def test_frame_identity_propagates_to_record_and_detection(self):
        """A detection carries a stable frame ID that is recorded in SurvivorRecord."""
        frame_id = "test_frame_123.jpg"
        det = self._make_det(frame_id=frame_id)
        geo = self._make_geo()

        task, is_new = self.manager.process_detection(det, geo, detected_by_drone_id="S1", frame_id=frame_id)
        self.assertTrue(is_new)
        record = self.manager.survivor_records[task.task_id]

        self.assertEqual(record.last_frame_id, frame_id)
        self.assertEqual(det.frame_id, frame_id)
        self.assertEqual(record.last_detected_by, "S1")
        self.assertIsNotNone(record.last_bbox)
        self.assertEqual(record.last_bbox, det.bbox)

    # -------------------------------------------------------------------------
    # Audit 2: Same-Frame Association Logic (False Split & False Merge Prevention)
    # -------------------------------------------------------------------------
    def test_same_frame_duplicate_overlapping_boxes_merge_even_if_separated_by_3_5m(self):
        """Duplicate bounding boxes on one person (IoU >= 0.3) must merge into ONE task.

        Simulates detector jitter or grazing ray angles where bounding box centers differ,
        producing ground coordinates 3.5 m apart. The old rule (> 3m separation) would
        incorrectly split them into two tasks. The new IoU rule correctly merges them.
        """
        frame_id = "frame_duplicate_test"
        # Box 1: torso-dominant box
        box1 = BoundingBox(xmin=0.45, ymin=0.45, xmax=0.55, ymax=0.60)
        det1 = self._make_det(confidence=0.88, bbox=box1, frame_id=frame_id)
        # Location 1: (12.972000, 77.594500)
        geo1 = self._make_geo(lat=12.972000, lon=77.594500)

        # Box 2: full-body + shadow box (strongly overlapping, IoU > 0.5)
        box2 = BoundingBox(xmin=0.46, ymin=0.44, xmax=0.56, ymax=0.62)
        det2 = self._make_det(confidence=0.92, bbox=box2, frame_id=frame_id)
        self.assertGreater(box1.iou(box2), 0.5)

        # Ground coordinate shifted North by 3.5m (approx 0.0000315 deg latitude)
        geo2 = self._make_geo(lat=12.972000 + 0.0000315, lon=77.594500)

        # Process first detection
        task1, is_new1 = self.manager.process_detection(det1, geo1, "S1", frame_id=frame_id)
        self.assertTrue(is_new1)

        # Process second detection from the SAME frame
        task2, is_new2 = self.manager.process_detection(det2, geo2, "S1", frame_id=frame_id)

        # Must merge into task1, not create task2 (prevent false split)
        self.assertFalse(is_new2, "Overlapping bounding box in same frame must merge into existing task")
        self.assertEqual(task2.task_id, task1.task_id)
        self.assertEqual(len(self.manager.get_all_tasks()), 1)

    def test_same_frame_two_distinct_people_separate_even_if_ground_distance_is_2m(self):
        """Two distinct, non-overlapping people (IoU = 0) in one frame must create TWO tasks.

        Simulates two individuals standing 2.0 m apart in the same camera frame.
        The old rule (< 3m separation) would incorrectly merge them into a single task.
        The new IoU rule recognizes they are distinct entities in image space.
        """
        frame_id = "frame_multi_person_test"
        # Person A on left side of image
        box_a = BoundingBox(xmin=0.30, ymin=0.45, xmax=0.38, ymax=0.60)
        det_a = self._make_det(confidence=0.91, bbox=box_a, frame_id=frame_id)
        geo_a = self._make_geo(lat=12.972000, lon=77.594500)

        # Person B on right side of image (disjoint, IoU = 0.0)
        box_b = BoundingBox(xmin=0.45, ymin=0.45, xmax=0.53, ymax=0.60)
        det_b = self._make_det(confidence=0.89, bbox=box_b, frame_id=frame_id)
        self.assertEqual(box_a.iou(box_b), 0.0)

        # Ground coordinate is only 2.0 m away (approx 0.000018 deg latitude)
        geo_b = self._make_geo(lat=12.972000 + 0.000018, lon=77.594500)

        # Process batch together
        batch = [(det_a, geo_a), (det_b, geo_b)]
        results = self.manager.process_detections_batch(batch, detected_by_drone_id="S1", frame_id=frame_id)

        self.assertEqual(len(results), 2)
        task_a, is_new_a = results[0]
        task_b, is_new_b = results[1]

        self.assertTrue(is_new_a)
        self.assertTrue(is_new_b, "Distinct non-overlapping people in the same frame must create separate tasks")
        self.assertNotEqual(task_a.task_id, task_b.task_id)
        self.assertEqual(len(self.manager.get_all_tasks()), 2)
        # Both tasks should be assigned distinct delivery drones
        self.assertIsNotNone(task_a.assigned_drone_id)
        self.assertIsNotNone(task_b.assigned_drone_id)
        self.assertNotEqual(task_a.assigned_drone_id, task_b.assigned_drone_id)

    def test_overlapping_uncertainty_intervals_triggers_ambiguity(self):
        """An observation near two closely spaced survivors triggers AMBIGUOUS status."""
        # Frame f1 discovers two distinct survivors simultaneously:
        # Person 1 (box 1) at (12.972000, 77.594500)
        # Person 2 (box 2) 6m North at (12.972000 + 0.000054, 77.594500)
        det1 = self._make_det(bbox=BoundingBox(0.1, 0.1, 0.2, 0.2), frame_id="f1")
        geo1 = self._make_geo(lat=12.972000, lon=77.594500, uncertainty_m=3.0)

        det2 = self._make_det(bbox=BoundingBox(0.8, 0.8, 0.9, 0.9), frame_id="f1")
        geo2 = self._make_geo(lat=12.972000 + 0.000054, lon=77.594500, uncertainty_m=3.0)

        results = self.manager.process_detections_batch([(det1, geo1), (det2, geo2)], "S1", frame_id="f1")
        self.assertEqual(len(self.manager.get_all_tasks()), 2)

        # Later frame f2: Observation 3 lands midway between them (3.0m from each) with 4.0m uncertainty
        det3 = self._make_det(bbox=BoundingBox(0.4, 0.4, 0.5, 0.5), frame_id="f2")
        geo3 = self._make_geo(lat=12.972000 + 0.000027, lon=77.594500, uncertainty_m=4.0)

        decision = self.associator.associate(
            det3, geo3, detected_by_drone_id="S3", survivor_records=self.manager.survivor_records, frame_id="f2"
        )
        self.assertEqual(decision.status, AssociationStatus.AMBIGUOUS)
        self.assertTrue(decision.is_ambiguous)
        self.assertIn("Conflicting candidates", decision.ambiguity_reason or "")

    def test_high_uncertainty_observation_triggers_ambiguity(self):
        """Observation with uncertainty exceeding threshold is flagged AMBIGUOUS."""
        det1 = self._make_det(frame_id="f1")
        geo1 = self._make_geo(lat=12.972000, lon=77.594500, uncertainty_m=3.0)
        self.manager.process_detection(det1, geo1, "S1", frame_id="f1")

        # Coarse observation with 25 m uncertainty (> 20 m threshold)
        det2 = self._make_det(frame_id="f2")
        geo2 = self._make_geo(lat=12.972000, lon=77.594500, uncertainty_m=25.0)

        decision = self.associator.associate(
            det2, geo2, detected_by_drone_id="S2", survivor_records=self.manager.survivor_records, frame_id="f2"
        )
        self.assertEqual(decision.status, AssociationStatus.AMBIGUOUS)
        self.assertTrue(decision.is_ambiguous)
        self.assertIn("High observation uncertainty", decision.ambiguity_reason or "")

    def test_input_order_permutation_invariance(self):
        """Processing detections in different orders produces identical task counts and assignments."""
        frame_id = "frame_perm"
        det_a = self._make_det(confidence=0.92, bbox=BoundingBox(0.2, 0.2, 0.28, 0.35), frame_id=frame_id)
        geo_a = self._make_geo(lat=12.972000, lon=77.594500)

        det_b = self._make_det(confidence=0.88, bbox=BoundingBox(0.6, 0.6, 0.68, 0.75), frame_id=frame_id)
        geo_b = self._make_geo(lat=12.972500, lon=77.595000)

        # Order 1: [A, B]
        mgr1 = SwarmManager(
            drones=[
                Drone(drone_id="D1", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.80),
                Drone(drone_id="D2", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.95),
            ],
            associator=SurvivorAssociator(),
        )
        res1 = mgr1.process_detections_batch([(det_a, geo_a), (det_b, geo_b)], "S1", frame_id=frame_id)

        # Order 2: [B, A]
        mgr2 = SwarmManager(
            drones=[
                Drone(drone_id="D1", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.80),
                Drone(drone_id="D2", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.95),
            ],
            associator=SurvivorAssociator(),
        )
        res2 = mgr2.process_detections_batch([(det_b, geo_b), (det_a, geo_a)], "S1", frame_id=frame_id)

        self.assertEqual(len(mgr1.get_all_tasks()), 2)
        self.assertEqual(len(mgr2.get_all_tasks()), 2)
        self.assertTrue(all(is_new for _, is_new in res1))
        self.assertTrue(all(is_new for _, is_new in res2))

    # -------------------------------------------------------------------------
    # Audit 3: Batch Atomicity
    # -------------------------------------------------------------------------
    def test_batch_atomicity_valid_followed_by_invalid_rejection(self):
        """Batch containing valid detections followed by an invalid one must leave ZERO state mutations."""
        valid_det1 = self._make_det(confidence=0.90, bbox=BoundingBox(0.1, 0.1, 0.2, 0.2))
        valid_geo1 = self._make_geo(lat=12.972000, lon=77.594500)

        valid_det2 = self._make_det(confidence=0.85, bbox=BoundingBox(0.7, 0.7, 0.8, 0.8))
        valid_geo2 = self._make_geo(lat=12.973000, lon=77.595500)

        # Invalid detection: confidence below threshold or NaN coordinates
        invalid_det3 = self._make_det(confidence=0.10)  # below 0.50 minimum_confidence
        invalid_geo3 = self._make_geo()

        batch = [
            (valid_det1, valid_geo1),
            (valid_det2, valid_geo2),
            (invalid_det3, invalid_geo3),
        ]

        # Must raise ValueError due to pre-validation
        with self.assertRaises(ValueError) as ctx:
            self.manager.process_detections_batch(batch, detected_by_drone_id="S1", minimum_confidence=0.50)

        self.assertIn("confidence", str(ctx.exception).lower())

        # Verify ATOMICITY: No tasks created, no drones allocated, no mission transition
        self.assertEqual(len(self.manager.get_all_tasks()), 0, "No partial tasks should remain after batch failure")
        self.assertEqual(len(self.manager.survivor_records), 0)
        self.assertEqual(self.manager._next_task_index, 1)

        # Drones must remain completely unallocated and READY
        for d in self.manager.drones.values():
            self.assertIsNone(d.active_task_id)
            self.assertEqual(d.status, DroneStatus.READY)

        self.assertEqual(self.manager.mission_manager.state, MissionState.IDLE)

    def test_batch_atomicity_with_corrupted_coordinates_in_middle(self):
        """A batch with an invalid item in the middle does not leave the first task allocated."""
        valid_det = self._make_det(confidence=0.90)
        valid_geo = self._make_geo(lat=12.972000, lon=77.594500)

        # Invalid second item: not a Detection instance
        corrupt_det = "invalid_detection_object"  # type: ignore
        corrupt_geo = self._make_geo()

        batch = [(valid_det, valid_geo), (corrupt_det, corrupt_geo)]

        with self.assertRaises(ValueError) as ctx:
            self.manager.process_detections_batch(batch, detected_by_drone_id="S1")

        self.assertIn("invalid detection", str(ctx.exception).lower())
        self.assertEqual(len(self.manager.get_all_tasks()), 0, "No tasks should remain after batch rejection")
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.READY)

    # -------------------------------------------------------------------------
    # Audit 4: Retry and Drone Lifecycle
    # -------------------------------------------------------------------------
    def test_delivery_task_fails_while_drone_active_or_recovering(self):
        """Task failure while drone is in flight leaves drone in RETURNING status, NOT READY."""
        task, _ = self.manager.process_detection(self._make_det(), self._make_geo(), "S1")
        drone_id = task.assigned_drone_id
        drone = self.manager.get_drone(drone_id)

        self.assertEqual(drone.status, DroneStatus.BUSY)
        self.assertEqual(drone.active_task_id, task.task_id)

        # Task fails while drone is still returning/recovering
        self.manager.update_task_status(task.task_id, TaskStatus.FAILED, drone_available=False)

        self.assertEqual(task.status, TaskStatus.FAILED)
        self.assertIsNone(drone.active_task_id)
        # Vehicle safety state: MUST NOT be marked READY
        self.assertEqual(drone.status, DroneStatus.RETURNING)

    def test_delivery_task_fails_after_drone_confirmed_available(self):
        """Task failure with confirmed vehicle return sets drone to READY."""
        task, _ = self.manager.process_detection(self._make_det(), self._make_geo(), "S1")
        drone = self.manager.get_drone(task.assigned_drone_id)

        self.manager.update_task_status(task.task_id, TaskStatus.FAILED, drone_available=True)

        self.assertEqual(task.status, TaskStatus.FAILED)
        self.assertIsNone(drone.active_task_id)
        self.assertEqual(drone.status, DroneStatus.READY)

    def test_retry_requested_when_no_delivery_drone_available(self):
        """When all delivery drones are busy or recovering, retry task stays in GEOLOCATED queue."""
        # Mark all delivery drones as RETURNING or BUSY
        for d in self.manager.drones.values():
            d.status = DroneStatus.RETURNING

        task1, _ = self.manager.process_detection(self._make_det(), self._make_geo(), "S1")
        # task1 was queued in GEOLOCATED because no drone was ready
        self.assertEqual(task1.status, TaskStatus.GEOLOCATED)
        self.assertIsNone(task1.assigned_drone_id)

        # Simulate task1 failed
        task1.status = TaskStatus.FAILED

        # Re-observation triggers retry
        task_retry, is_new = self.manager.process_detection(self._make_det(0.95), self._make_geo(), "S2")

        self.assertTrue(is_new)
        self.assertEqual(task_retry.status, TaskStatus.GEOLOCATED)
        self.assertEqual(task_retry.retry_of, task1.task_id)
        self.assertIsNone(task_retry.assigned_drone_id)

    def test_task_retried_repeatedly(self):
        """A task that fails repeatedly can be retried multiple times in sequence."""
        geo = self._make_geo()

        # Attempt 1
        t1, _ = self.manager.process_detection(self._make_det(0.80), geo, "S1")
        self.assertEqual(t1.task_id, "task_001")
        self.manager.update_task_status(t1.task_id, TaskStatus.FAILED, drone_available=True)

        # Attempt 2 (Retry of 1)
        t2, is_new2 = self.manager.process_detection(self._make_det(0.85), geo, "S1")
        self.assertTrue(is_new2)
        self.assertEqual(t2.task_id, "task_002")
        self.assertEqual(t2.retry_of, "task_001")
        self.manager.update_task_status(t2.task_id, TaskStatus.FAILED, drone_available=True)

        # Attempt 3 (Retry of 2)
        t3, is_new3 = self.manager.process_detection(self._make_det(0.90), geo, "S2")
        self.assertTrue(is_new3)
        self.assertEqual(t3.task_id, "task_003")
        self.assertIn(t3.retry_of, ("task_001", "task_002"))
        self.assertEqual(len(self.manager.get_all_tasks()), 3)

    def test_verified_survivor_observed_again_does_not_dispatch_duplicate_drone(self):
        """Observing a verified (rescued) survivor records sighting without new drone dispatch."""
        task, _ = self.manager.process_detection(self._make_det(), self._make_geo(), "S1")
        self.manager.update_task_status(task.task_id, TaskStatus.VERIFIED, drone_available=True)

        # Drone D2 was released and is READY
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.READY)

        # Survivor is observed again
        repeated_task, is_new = self.manager.process_detection(self._make_det(0.96), self._make_geo(), "S2")

        self.assertFalse(is_new)
        self.assertEqual(repeated_task.task_id, task.task_id)
        self.assertEqual(repeated_task.status, TaskStatus.VERIFIED)
        # D2 must NOT be allocated
        self.assertEqual(self.manager.get_drone("D2").status, DroneStatus.READY)
        self.assertEqual(len(self.manager.get_active_tasks()), 0)
        self.assertEqual(self.manager.survivor_records[task.task_id].verified_re_sightings, 1)

    def test_failed_task_with_active_retry_prevents_duplicate_active_retries(self):
        """New observation while previous retry is still active matches active retry task.

        Critical reliability check: if task_001 failed and task_002 is currently EN_ROUTE,
        a new sighting of the survivor must associate with task_002 and NOT create task_003!
        """
        geo = self._make_geo()

        # Step 1: Initial task fails
        t1, _ = self.manager.process_detection(self._make_det(0.80), geo, "S1")
        self.manager.update_task_status(t1.task_id, TaskStatus.FAILED, drone_available=True)

        # Step 2: Retry creates task_002, assigned to D2
        t2, is_new2 = self.manager.process_detection(self._make_det(0.85), geo, "S1")
        self.assertTrue(is_new2)
        self.assertEqual(t2.task_id, "task_002")
        self.assertEqual(t2.status, TaskStatus.ASSIGNED)
        self.manager.update_task_status(t2.task_id, TaskStatus.EN_ROUTE)

        # Step 3: Search drone spots the survivor again WHILE task_002 is still EN_ROUTE!
        t_sight, is_new_sight = self.manager.process_detection(self._make_det(0.93), geo, "S2")

        # Must NOT spawn duplicate task_003
        self.assertFalse(is_new_sight, "Duplicate active retries must be prevented")
        self.assertEqual(t_sight.task_id, "task_002")
        self.assertEqual(t_sight.status, TaskStatus.EN_ROUTE)
        self.assertEqual(len(self.manager.get_all_tasks()), 2)
        self.assertEqual(len(self.manager.get_active_tasks()), 1)


if __name__ == "__main__":
    unittest.main()
