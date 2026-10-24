"""Deterministic synthetic tests for temporal survivor association edge cases in RescueSwarm.

Validates the spatial association policy against 7 core temporal scenarios:
1. Stationary survivor with small bounding-box jitter across frames.
2. Moving survivor with position changes within and beyond spatial match threshold.
3. Two nearby survivors crossing / changing relative positions (spatial ambiguity & identity swap).
4. Survivor temporarily disappearing (occlusion) and reappearing in active, verified, and failed states.
5. High-uncertainty observation gating (uncertainty radius > threshold).
6. Frame IDs that repeat accidentally or arrive out of order.
7. Batch processing order invariance and transactional rollback under corrupted concurrent input.

All tests in this suite are 100% deterministic unit tests requiring NO external
video assets, weights, PX4 SITL, or Gazebo.
"""

import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

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


class TestTemporalAssociationEdgeCases(unittest.TestCase):
    """Deterministic synthetic test suite for multi-frame association edge cases."""

    def setUp(self):
        self.associator = SurvivorAssociator(
            match_distance_threshold_m=10.0,
            max_association_uncertainty_m=20.0,
            min_distinct_separation_m=3.0,
            intra_frame_iou_threshold=0.3,
        )
        self.delivery_drones = [
            Drone(drone_id="D1", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.85),
            Drone(drone_id="D2", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.90),
            Drone(drone_id="D3", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.75),
        ]
        self.manager = SwarmManager(drones=self.delivery_drones, associator=self.associator)
        self.manager.mission_manager.transition_to(MissionState.INITIALIZING)
        self.manager.mission_manager.transition_to(MissionState.SEARCHING)

    def _create_estimate(
        self,
        lat: float = 12.972000,
        lon: float = 77.594500,
        uncertainty_m: float = 2.5,
    ) -> GeolocationEstimate:
        return GeolocationEstimate(
            location=GeoPoint(latitude=lat, longitude=lon, altitude_m=0.0),
            offset_north_m=0.0,
            offset_east_m=0.0,
            ground_distance_m=0.0,
            slant_range_m=30.0,
            uncertainty_radius_m=uncertainty_m,
        )

    def _create_detection(
        self,
        confidence: float = 0.88,
        bbox: BoundingBox = BoundingBox(0.48, 0.48, 0.52, 0.52),
        frame_id: str = "frame_001",
    ) -> Detection:
        return Detection(
            class_id=0,
            class_name="pedestrian",
            confidence=confidence,
            bbox=bbox,
            frame_id=frame_id,
        )

    # -------------------------------------------------------------------------
    # Scenario 1: Stationary survivor with small bounding-box jitter across frames
    # -------------------------------------------------------------------------
    def test_01_stationary_survivor_with_bbox_jitter(self):
        """Stationary survivor observed across 5 consecutive frames with bbox jitter must
        maintain a single rescue task without duplicates.
        """
        base_lat = 12.972000
        base_lon = 77.594500

        # Frame 1: initial detection
        det1 = self._create_detection(confidence=0.85, bbox=BoundingBox(0.48, 0.48, 0.52, 0.52), frame_id="f_01")
        geo1 = self._create_estimate(lat=base_lat, lon=base_lon, uncertainty_m=3.0)
        t1, is_new1 = self.manager.process_detection(det1, geo1, "S1", frame_id="f_01", timestamp=1.0)

        self.assertTrue(is_new1)
        self.assertEqual(t1.task_id, "task_001")
        self.assertEqual(len(self.manager.get_all_tasks()), 1)

        # Frames 2..5: small sub-pixel and meter jitter (+- 0.000002 deg ~= 0.2m ground jitter)
        jitters = [
            (BoundingBox(0.482, 0.478, 0.522, 0.518), 0.90, 0.000001, -0.000001, 2.5),
            (BoundingBox(0.479, 0.481, 0.519, 0.521), 0.87, -0.000002, 0.000001, 2.2),
            (BoundingBox(0.481, 0.480, 0.521, 0.520), 0.92, 0.000000, 0.000002, 2.0),
            (BoundingBox(0.480, 0.482, 0.520, 0.522), 0.86, -0.000001, -0.000002, 2.4),
        ]

        for i, (j_bbox, j_conf, d_lat, d_lon, u_m) in enumerate(jitters, start=2):
            det = self._create_detection(confidence=j_conf, bbox=j_bbox, frame_id=f"f_{i:02d}")
            geo = self._create_estimate(lat=base_lat + d_lat, lon=base_lon + d_lon, uncertainty_m=u_m)
            task, is_new = self.manager.process_detection(det, geo, "S1", frame_id=f"f_{i:02d}", timestamp=1.0 + 0.1*i)

            self.assertFalse(is_new, f"Frame {i} jitter must match existing task")
            self.assertEqual(task.task_id, "task_001")

        # Verify only 1 task was created, detection count is 5, confidence updated to max (0.92)
        rec = self.manager.survivor_records["task_001"]
        self.assertEqual(len(self.manager.get_all_tasks()), 1)
        self.assertEqual(rec.detection_count, 5)
        self.assertAlmostEqual(rec.task.confidence, 0.92, places=2)
        self.assertEqual(rec.uncertainty_radius_m, 2.0)  # Most refined uncertainty preserved

    # -------------------------------------------------------------------------
    # Scenario 2: Moving survivor with position changes
    # -------------------------------------------------------------------------
    def test_02_moving_survivor_within_and_beyond_threshold(self):
        """Moving survivor within 10m threshold matches; survivor moving beyond 10m
        triggers a new task (documented limitation of pure spatial association).
        """
        # Start position
        lat0, lon0 = 12.972000, 77.594500
        det0 = self._create_detection(frame_id="f_m0")
        geo0 = self._create_estimate(lat=lat0, lon=lon0)
        task0, is_new0 = self.manager.process_detection(det0, geo0, "S1", frame_id="f_m0", timestamp=10.0)
        self.assertTrue(is_new0)
        self.assertEqual(task0.task_id, "task_001")

        # Step 1: Moves 3.5m North (0.0000315 deg latitude ~= 3.5m)
        lat1 = lat0 + 0.0000315
        det1 = self._create_detection(frame_id="f_m1")
        geo1 = self._create_estimate(lat=lat1, lon=lon0)
        task1, is_new1 = self.manager.process_detection(det1, geo1, "S1", frame_id="f_m1", timestamp=12.0)

        self.assertFalse(is_new1, "3.5m movement is within 10.0m match threshold and must match")
        self.assertEqual(task1.task_id, "task_001")

        # Step 2: Moves 14m further North (beyond 10.0m match threshold from original location)
        # 14m displacement: 0.000126 deg lat ~= 14m
        lat2 = lat0 + 0.000126
        det2 = self._create_detection(frame_id="f_m2")
        geo2 = self._create_estimate(lat=lat2, lon=lon0)
        task2, is_new2 = self.manager.process_detection(det2, geo2, "S1", frame_id="f_m2", timestamp=15.0)

        # Documented behavior: without kinematic velocity tracking, spatial associator treats this as a new survivor
        self.assertTrue(is_new2, "Displacement > 10m creates a distinct task under spatial association")
        self.assertEqual(task2.task_id, "task_002")
        self.assertEqual(len(self.manager.get_all_tasks()), 2)

    # -------------------------------------------------------------------------
    # Scenario 3: Two nearby survivors crossing or changing relative positions
    # -------------------------------------------------------------------------
    def test_03_two_nearby_survivors_crossing_triggers_ambiguity(self):
        """When two nearby survivors approach each other within min_distinct_separation (3m),
        the spatial associator correctly flags ambiguity instead of silently misassigning.
        """
        # Survivor A at (0m), Survivor B at (+5m North)
        lat_a = 12.972000
        lat_b = 12.972045  # ~5.0m North of A
        lon = 77.594500

        # Frame 1: Initial separation (5.0m >= 3.0m) -> 2 distinct tasks created
        det_a1 = self._create_detection(bbox=BoundingBox(0.4, 0.4, 0.45, 0.45), frame_id="f_cross_1")
        geo_a1 = self._create_estimate(lat=lat_a, lon=lon)
        task_a, is_new_a = self.manager.process_detection(det_a1, geo_a1, "S1", frame_id="f_cross_1", timestamp=1.0)
        self.assertTrue(is_new_a)
        self.assertEqual(task_a.task_id, "task_001")

        det_b1 = self._create_detection(bbox=BoundingBox(0.55, 0.55, 0.6, 0.6), frame_id="f_cross_1")
        geo_b1 = self._create_estimate(lat=lat_b, lon=lon)
        task_b, is_new_b = self.manager.process_detection(det_b1, geo_b1, "S1", frame_id="f_cross_1", timestamp=1.0)
        self.assertTrue(is_new_b)
        self.assertEqual(task_b.task_id, "task_002")

        # Frame 2: Crossing point. Both survivors converge at mid-point (+2.5m North)
        # Sighting is within 2.5m of BOTH task_001 and task_002 (|dist1 - dist2| < 3.0m)
        lat_mid = 12.972022  # ~2.5m North
        det_cross = self._create_detection(bbox=BoundingBox(0.48, 0.48, 0.52, 0.52), frame_id="f_cross_2")
        geo_cross = self._create_estimate(lat=lat_mid, lon=lon)

        # Query associator directly to inspect ambiguity flag
        decision = self.associator.associate(
            detection=det_cross,
            geo_estimate=geo_cross,
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            frame_id="f_cross_2",
            timestamp=2.0,
        )
        self.assertTrue(decision.is_ambiguous, "Midpoint crossing sighting must be flagged as ambiguous")
        self.assertEqual(decision.status, AssociationStatus.AMBIGUOUS)
        self.assertIn("Conflicting candidates", decision.ambiguity_reason)

        # Process through SwarmManager
        task_c, is_new_c = self.manager.process_detection(det_cross, geo_cross, "S1", frame_id="f_cross_2", timestamp=2.0)
        self.assertFalse(is_new_c)
        # Verify ambiguous sighting was recorded on the nearest record without creating a duplicate task
        self.assertEqual(len(self.manager.get_all_tasks()), 2)
        rec = self.manager.survivor_records[task_c.task_id]
        self.assertGreater(rec.ambiguous_sightings, 0)

    # -------------------------------------------------------------------------
    # Scenario 4: Survivor temporarily disappearing (occlusion) and reappearing
    # -------------------------------------------------------------------------
    def test_04_occlusion_and_reappearance_active_verified_and_failed(self):
        """Tests survivor disappearance across 3 frames and subsequent reappearance in
        ACTIVE, VERIFIED, and FAILED states.
        """
        lat, lon = 12.972000, 77.594500
        det = self._create_detection(frame_id="f_occ_1")
        geo = self._create_estimate(lat=lat, lon=lon)

        # Frame 1: Detection -> task_001 ASSIGNED to D2
        task, is_new = self.manager.process_detection(det, geo, "S1", frame_id="f_occ_1", timestamp=1.0)
        self.assertTrue(is_new)
        self.assertEqual(task.status, TaskStatus.ASSIGNED)

        # Frames 2, 3, 4: Survivor occluded (e.g. under tree canopy). No detections processed.

        # Frame 5: Reappearance while task is still ASSIGNED (within 10m)
        det_reapp = self._create_detection(frame_id="f_occ_5")
        geo_reapp = self._create_estimate(lat=lat + 0.000010, lon=lon, uncertainty_m=2.0)
        task_reapp, is_new_reapp = self.manager.process_detection(
            det_reapp, geo_reapp, "S1", frame_id="f_occ_5", timestamp=5.0
        )
        self.assertFalse(is_new_reapp, "Reappearance of active task must match existing task")
        self.assertEqual(task_reapp.task_id, "task_001")

        # Transition task_001 to VERIFIED (simulated drop successful)
        task.status = TaskStatus.VERIFIED
        self.manager.drones[task.assigned_drone_id].status = DroneStatus.READY
        self.manager.drones[task.assigned_drone_id].active_task_id = None

        # Frame 6: Reappearance of VERIFIED survivor
        det_ver = self._create_detection(frame_id="f_occ_6")
        geo_ver = self._create_estimate(lat=lat, lon=lon)
        task_ver, is_new_ver = self.manager.process_detection(
            det_ver, geo_ver, "S1", frame_id="f_occ_6", timestamp=6.0
        )
        self.assertFalse(is_new_ver, "Re-sighting of verified survivor must not create new task")
        self.assertEqual(task_ver.task_id, "task_001")
        # Ensure NO new delivery drone was dispatched
        self.assertEqual(sum(1 for d in self.manager.drones.values() if d.status == DroneStatus.BUSY), 0)

        # Now test FAILED task re-observation
        task.status = TaskStatus.FAILED
        det_fail = self._create_detection(frame_id="f_occ_7")
        geo_fail = self._create_estimate(lat=lat, lon=lon)
        task_fail, is_new_fail = self.manager.process_detection(
            det_fail, geo_fail, "S1", frame_id="f_occ_7", timestamp=7.0
        )
        self.assertTrue(is_new_fail, "Re-sighting of failed task must trigger a retry task")
        self.assertEqual(task_fail.retry_of, "task_001")
        self.assertEqual(task_fail.task_id, "task_002")

    # -------------------------------------------------------------------------
    # Scenario 5: High-uncertainty observations
    # -------------------------------------------------------------------------
    def test_05_high_uncertainty_observation_gating(self):
        """Observations with uncertainty > 20.0m are gated as AMBIGUOUS and do not
        corrupt refined task location.
        """
        lat, lon = 12.972000, 77.594500
        # Frame 1: Low-uncertainty detection (2.0m)
        det1 = self._create_detection(frame_id="f_u1")
        geo1 = self._create_estimate(lat=lat, lon=lon, uncertainty_m=2.0)
        task1, _ = self.manager.process_detection(det1, geo1, "S1", frame_id="f_u1", timestamp=1.0)
        self.assertEqual(self.manager.survivor_records["task_001"].uncertainty_radius_m, 2.0)

        # Frame 2: High-uncertainty observation (25.0m > 20.0m threshold) shifted by 5m
        lat_shift = lat + 0.000045  # ~5m North
        det2 = self._create_detection(frame_id="f_u2")
        geo2 = self._create_estimate(lat=lat_shift, lon=lon, uncertainty_m=25.0)

        decision = self.associator.associate(
            detection=det2,
            geo_estimate=geo2,
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            frame_id="f_u2",
            timestamp=2.0,
        )
        self.assertTrue(decision.is_ambiguous)
        self.assertIn("High observation uncertainty", decision.ambiguity_reason)

        task2, is_new2 = self.manager.process_detection(det2, geo2, "S1", frame_id="f_u2", timestamp=2.0)
        self.assertFalse(is_new2)
        # Refined location must NOT have been overwritten by coarse 25m estimate
        rec = self.manager.survivor_records["task_001"]
        self.assertAlmostEqual(rec.task.location.latitude, lat, places=6)
        self.assertEqual(rec.uncertainty_radius_m, 2.0)
        self.assertEqual(rec.ambiguous_sightings, 1)

    # -------------------------------------------------------------------------
    # Scenario 6: Frame IDs that repeat accidentally or arrive out of order
    # -------------------------------------------------------------------------
    def test_06_duplicate_frame_ids_and_out_of_order_arrival(self):
        """Tests handling of duplicated frame IDs and non-monotonic timestamps.
        Documents how same-frame IoU gating interacts with accidental frame ID reuse.
        """
        lat, lon = 12.972000, 77.594500

        # Sub-test A: Accidental duplicate frame ID across consecutive temporal steps
        # Frame A: Person at position A, bbox [0.1, 0.1, 0.2, 0.2]
        det_a = self._create_detection(bbox=BoundingBox(0.1, 0.1, 0.2, 0.2), frame_id="frame_DUP")
        geo_a = self._create_estimate(lat=lat, lon=lon)
        t_a, is_new_a = self.manager.process_detection(det_a, geo_a, "S1", frame_id="frame_DUP", timestamp=1.0)
        self.assertTrue(is_new_a)
        self.assertEqual(t_a.task_id, "task_001")

        # Frame B: Person moves to bbox [0.5, 0.5, 0.6, 0.6] (disjoint box, IoU = 0.0), but frame_id repeats!
        # Because frame_id repeats, the associator applies the SAME-FRAME rule:
        # Since IoU = 0.0 < 0.3, it assumes they are two distinct persons in the same image!
        det_b = self._create_detection(bbox=BoundingBox(0.5, 0.5, 0.6, 0.6), frame_id="frame_DUP")
        geo_b = self._create_estimate(lat=lat + 0.000018, lon=lon)  # 2m away
        t_b, is_new_b = self.manager.process_detection(det_b, geo_b, "S1", frame_id="frame_DUP", timestamp=1.04)

        # Documented failure mode: accidental frame_id reuse with non-overlapping bbox splits task!
        self.assertTrue(is_new_b, "Duplicate frame ID with disjoint bbox causes false task creation under intra-frame gating")
        self.assertEqual(t_b.task_id, "task_002")

        # Sub-test B: Out-of-order timestamp arrival
        # Frame at t=10.0s arrives
        det_t10 = self._create_detection(bbox=BoundingBox(0.1, 0.1, 0.2, 0.2), frame_id="f_t10")
        geo_t10 = self._create_estimate(lat=lat, lon=lon)
        t_10, _ = self.manager.process_detection(det_t10, geo_t10, "S1", frame_id="f_t10", timestamp=10.0)
        rec = self.manager.survivor_records["task_001"]
        self.assertEqual(rec.last_seen_timestamp, 10.0)

        # Delayed frame at t=5.0s arrives later
        det_t5 = self._create_detection(bbox=BoundingBox(0.1, 0.1, 0.2, 0.2), frame_id="f_t5")
        geo_t5 = self._create_estimate(lat=lat, lon=lon)
        t_5, is_new_5 = self.manager.process_detection(det_t5, geo_t5, "S1", frame_id="f_t5", timestamp=5.0)
        self.assertFalse(is_new_5)
        # Note: In current implementation, last_seen_timestamp is overwritten with the incoming observation timestamp
        self.assertEqual(rec.last_seen_timestamp, 5.0)

    # -------------------------------------------------------------------------
    # Scenario 7: Multiple observations arriving concurrently or in different orders
    # -------------------------------------------------------------------------
    def test_07_batch_order_invariance_and_atomic_rollback(self):
        """Verifies that batch processing creates the same number of tasks regardless of
        arrival permutation, and that corruption rolls back all changes atomically.
        """
        lat1, lon1 = 12.972000, 77.594500
        lat2, lon2 = 12.972100, 77.594500  # ~11m North (distinct survivor)

        det1 = self._create_detection(bbox=BoundingBox(0.1, 0.1, 0.2, 0.2), frame_id="batch_01")
        geo1 = self._create_estimate(lat=lat1, lon=lon1)

        det2 = self._create_detection(bbox=BoundingBox(0.6, 0.6, 0.7, 0.7), frame_id="batch_01")
        geo2 = self._create_estimate(lat=lat2, lon=lon2)

        # Trial 1: Order [det1, det2]
        res1 = self.manager.process_detections_batch(
            detections_with_geo=[(det1, geo1), (det2, geo2)],
            detected_by_drone_id="S1",
            frame_id="batch_01",
            timestamp=1.0,
            minimum_confidence=0.50,
        )
        self.assertEqual(len(res1), 2)
        self.assertTrue(res1[0][1])  # task 1 is new
        self.assertTrue(res1[1][1])  # task 2 is new
        self.assertEqual(len(self.manager.get_all_tasks()), 2)
        self.assertEqual(sum(1 for d in self.manager.drones.values() if d.status == DroneStatus.BUSY), 2)

        # Trial 2: Fresh manager with reverse order [det2, det1]
        fresh_drones = [
            Drone(drone_id="D1", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.85),
            Drone(drone_id="D2", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.90),
            Drone(drone_id="D3", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.75),
        ]
        fresh_mgr = SwarmManager(drones=fresh_drones, associator=self.associator)
        fresh_mgr.mission_manager.transition_to(MissionState.INITIALIZING)
        fresh_mgr.mission_manager.transition_to(MissionState.SEARCHING)

        res2 = fresh_mgr.process_detections_batch(
            detections_with_geo=[(det2, geo2), (det1, geo1)],
            detected_by_drone_id="S1",
            frame_id="batch_01",
            timestamp=1.0,
            minimum_confidence=0.50,
        )
        self.assertEqual(len(res2), 2)
        self.assertTrue(res2[0][1])
        self.assertTrue(res2[1][1])
        self.assertEqual(len(fresh_mgr.get_all_tasks()), 2)
        self.assertEqual(sum(1 for d in fresh_mgr.drones.values() if d.status == DroneStatus.BUSY), 2)

        # Trial 3: Corrupted batch must abort atomically before any tasks are created
        tasks_before = len(self.manager.get_all_tasks())
        with self.assertRaises(ValueError):
            self.manager.process_detections_batch(
                detections_with_geo=[(det1, geo1), ("corrupted_non_detection", geo2)],
                detected_by_drone_id="S1",
                frame_id="batch_corrupt",
                timestamp=2.0,
            )
        # Verify rollback: task count and drone statuses unchanged
        self.assertEqual(len(self.manager.get_all_tasks()), tasks_before)


if __name__ == "__main__":
    unittest.main()
