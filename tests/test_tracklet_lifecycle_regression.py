"""Deterministic synthetic regression test suite for Fix 8: Local Visual Tracklet Layer.

Validates:
1. Stationary image target with small bounding-box jitter maintaining local track continuity.
2. Moving target with continuous local track identity.
3. Two nearby targets crossing paths (tracklet-based spatial disambiguation & identity preservation).
4. Temporarily occluded target that reappears under same track ID.
5. Two drones producing the same numeric local track ID (cross-drone namespace scoping).
6. Tracker reset that reuses a previous numeric track ID (session generation scoping).
7. Track that drops out and returns under a new track ID (spatial fallback matching).
8. Conflicting track identity and geolocation evidence (contradictory physical distance gating).
9. Missing track IDs, invalid coordinates, and high uncertainty gating.
10. Verified-task and failed-retry lifecycle preservation with local visual tracklets.

NOTE: All tests in this file use deterministic synthetic tracks and telemetry fixtures;
they validate software interfaces, namespace scoping, and association lifecycle policies,
not physical camera hardware or real-world tracker precision.
"""

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


class TestTrackletLifecycleRegression(unittest.TestCase):
    """Deterministic synthetic test suite covering the 10 core tracklet lifecycle requirements."""

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

    def _geo(self, lat: float = 12.972000, lon: float = 77.594500, uncertainty_m: float = 2.0) -> GeolocationEstimate:
        return GeolocationEstimate(
            location=GeoPoint(latitude=lat, longitude=lon, altitude_m=0.0),
            offset_north_m=0.0,
            offset_east_m=0.0,
            ground_distance_m=0.0,
            slant_range_m=30.0,
            uncertainty_radius_m=uncertainty_m,
        )

    def _det(
        self,
        track_id: int | None = 1,
        source_id: str | None = "S1",
        tracker_session_id: str | None = "session_001",
        confidence: float = 0.90,
        bbox: BoundingBox = BoundingBox(0.48, 0.48, 0.52, 0.52),
        frame_id: str = "f_01",
    ) -> Detection:
        return Detection(
            class_id=0,
            class_name="pedestrian",
            confidence=confidence,
            bbox=bbox,
            frame_id=frame_id,
            track_id=track_id,
            tracker_session_id=tracker_session_id,
            source_id=source_id,
        )

    # -------------------------------------------------------------------------
    # Scenario 1: Stationary image target with small bounding-box jitter
    # -------------------------------------------------------------------------
    def test_01_stationary_target_with_bbox_jitter_and_track_continuity(self):
        """Stationary target with track_id=1 across 5 jittered frames maintains single task."""
        lat, lon = 12.972000, 77.594500

        # Frame 1: initial detection with track_id=1
        det1 = self._det(track_id=1, source_id="S1", frame_id="f_01", confidence=0.85)
        task1, is_new1 = self.manager.process_detection(det1, self._geo(lat, lon), "S1", frame_id="f_01", timestamp=1.0)
        self.assertTrue(is_new1)
        self.assertEqual(task1.task_id, "task_001")

        # Frames 2..5: small bbox jitter and small coordinate jitter
        for i in range(2, 6):
            jitter_bbox = BoundingBox(0.48 + 0.002*i, 0.48 - 0.002*i, 0.52 + 0.002*i, 0.52 - 0.002*i)
            det = self._det(track_id=1, source_id="S1", bbox=jitter_bbox, frame_id=f"f_{i:02d}", confidence=0.88 + 0.01*i)
            task, is_new = self.manager.process_detection(det, self._geo(lat + 0.000002*i, lon), "S1", frame_id=f"f_{i:02d}", timestamp=1.0 + 0.04*i)
            self.assertFalse(is_new)
            self.assertEqual(task.task_id, "task_001")

        rec = self.manager.survivor_records["task_001"]
        self.assertEqual(len(self.manager.get_all_tasks()), 1)
        self.assertEqual(rec.detection_count, 5)
        self.assertEqual(rec.active_scoped_tracks.get("S1"), "S1:session_001:1")

    # -------------------------------------------------------------------------
    # Scenario 2: Moving target with continuous local track identity
    # -------------------------------------------------------------------------
    def test_02_moving_target_with_continuous_track_identity(self):
        """Moving target with continuous local track identity associates smoothly across steps."""
        lat0, lon0 = 12.972000, 77.594500
        det0 = self._det(track_id=2, source_id="S1", frame_id="f_m0")
        task0, is_new0 = self.manager.process_detection(det0, self._geo(lat0, lon0), "S1", frame_id="f_m0", timestamp=1.0)
        self.assertTrue(is_new0)

        # Target moves 2 meters North in each of 3 subsequent frames (total 6m)
        for i in range(1, 4):
            lat_i = lat0 + i * 0.000018  # ~2.0m North per step
            det_i = self._det(track_id=2, source_id="S1", frame_id=f"f_m{i}")
            task_i, is_new_i = self.manager.process_detection(det_i, self._geo(lat_i, lon0), "S1", frame_id=f"f_m{i}", timestamp=1.0 + 0.1*i)
            self.assertFalse(is_new_i)
            self.assertEqual(task_i.task_id, "task_001")

        self.assertEqual(len(self.manager.get_all_tasks()), 1)

    # -------------------------------------------------------------------------
    # Scenario 3: Two nearby targets crossing paths (tracklet disambiguation)
    # -------------------------------------------------------------------------
    def test_03_two_nearby_targets_crossing_disambiguated_by_tracklets(self):
        """When two nearby targets cross (separation < 3.0m), local tracklet continuity
        resolves ambiguity and prevents identity swaps.
        """
        lat_a = 12.972000
        lat_b = 12.972045  # 5m North
        lon = 77.594500

        # Frame 1: Initial separation (5m). Target A has track_id=10, Target B has track_id=20
        det_a1 = self._det(track_id=10, source_id="S1", bbox=BoundingBox(0.4, 0.4, 0.45, 0.45), frame_id="f_c1")
        task_a, is_new_a = self.manager.process_detection(det_a1, self._geo(lat_a, lon), "S1", frame_id="f_c1", timestamp=1.0)
        self.assertTrue(is_new_a)
        self.assertEqual(task_a.task_id, "task_001")

        det_b1 = self._det(track_id=20, source_id="S1", bbox=BoundingBox(0.55, 0.55, 0.6, 0.6), frame_id="f_c1")
        task_b, is_new_b = self.manager.process_detection(det_b1, self._geo(lat_b, lon), "S1", frame_id="f_c1", timestamp=1.0)
        self.assertTrue(is_new_b)
        self.assertEqual(task_b.task_id, "task_002")

        # Frame 2: Crossing midpoint. Both targets are at separation 0.8m (< 3.0m).
        # Query associator for Target A (track_id=10) at midpoint (2.5m North)
        lat_mid_a = 12.972022  # near midpoint
        det_a2 = self._det(track_id=10, source_id="S1", bbox=BoundingBox(0.47, 0.47, 0.52, 0.52), frame_id="f_c2")
        decision_a = self.associator.associate(
            detection=det_a2,
            geo_estimate=self._geo(lat_mid_a, lon),
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            frame_id="f_c2",
            timestamp=2.0,
        )

        # Tracklet continuity disambiguates the crossing: NO ambiguity, matches task_001
        self.assertFalse(decision_a.is_ambiguous, "Track continuity must prevent proximity ambiguity during crossing")
        self.assertEqual(decision_a.matched_task_id, "task_001")
        self.assertTrue(decision_a.matched_by_track)

        # Query associator for Target B (track_id=20) at midpoint
        lat_mid_b = 12.972024  # near midpoint
        det_b2 = self._det(track_id=20, source_id="S1", bbox=BoundingBox(0.49, 0.49, 0.54, 0.54), frame_id="f_c2")
        decision_b = self.associator.associate(
            detection=det_b2,
            geo_estimate=self._geo(lat_mid_b, lon),
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            frame_id="f_c2",
            timestamp=2.0,
        )
        self.assertFalse(decision_b.is_ambiguous)
        self.assertEqual(decision_b.matched_task_id, "task_002")
        self.assertTrue(decision_b.matched_by_track)

    # -------------------------------------------------------------------------
    # Scenario 4: Temporarily occluded target that reappears
    # -------------------------------------------------------------------------
    def test_04_occluded_target_reappearing_with_same_track(self):
        """Target disappears for 3 frames and reappears under same track ID."""
        lat, lon = 12.972000, 77.594500
        det1 = self._det(track_id=5, source_id="S1", frame_id="f_o1")
        task1, is_new1 = self.manager.process_detection(det1, self._geo(lat, lon), "S1", frame_id="f_o1", timestamp=1.0)
        self.assertTrue(is_new1)

        # Frames 2, 3, 4: occlusion (no detections)
        # Frame 5: reappears with track_id=5
        det5 = self._det(track_id=5, source_id="S1", frame_id="f_o5")
        task5, is_new5 = self.manager.process_detection(det5, self._geo(lat + 0.000010, lon), "S1", frame_id="f_o5", timestamp=5.0)
        self.assertFalse(is_new5)
        self.assertEqual(task5.task_id, "task_001")

    # -------------------------------------------------------------------------
    # Scenario 5: Two drones producing the same numeric local track ID
    # -------------------------------------------------------------------------
    def test_05_cross_drone_numeric_track_id_isolation(self):
        """Two search drones both producing local track_id=1 must not be confused as same survivor."""
        # Drone S1 observes Survivor 1 at Base
        lat1, lon1 = 12.972000, 77.594500
        det_s1 = self._det(track_id=1, source_id="S1", frame_id="f_s1")
        t_s1, new_s1 = self.manager.process_detection(det_s1, self._geo(lat1, lon1), "S1", frame_id="f_s1", timestamp=1.0)
        self.assertTrue(new_s1)
        self.assertEqual(t_s1.task_id, "task_001")

        # Drone S2 observes Survivor 2 located 60m away, also assigning local track_id=1
        lat2, lon2 = 12.972540, 77.594500  # ~60m away
        det_s2 = self._det(track_id=1, source_id="S2", frame_id="f_s2")
        t_s2, new_s2 = self.manager.process_detection(det_s2, self._geo(lat2, lon2), "S2", frame_id="f_s2", timestamp=1.0)
        self.assertTrue(new_s2, "Drone S2 track_id=1 60m away must create a distinct task")
        self.assertEqual(t_s2.task_id, "task_002")

        # S1 subsequent sighting of track 1 matches task_001
        det_s1_sub = self._det(track_id=1, source_id="S1", frame_id="f_s1_2")
        t_s1_sub, is_new_s1_sub = self.manager.process_detection(det_s1_sub, self._geo(lat1, lon1), "S1", frame_id="f_s1_2", timestamp=2.0)
        self.assertFalse(is_new_s1_sub)
        self.assertEqual(t_s1_sub.task_id, "task_001")

        # S2 subsequent sighting of track 1 matches task_002
        det_s2_sub = self._det(track_id=1, source_id="S2", frame_id="f_s2_2")
        t_s2_sub, is_new_s2_sub = self.manager.process_detection(det_s2_sub, self._geo(lat2, lon2), "S2", frame_id="f_s2_2", timestamp=2.0)
        self.assertFalse(is_new_s2_sub)
        self.assertEqual(t_s2_sub.task_id, "task_002")

    # -------------------------------------------------------------------------
    # Scenario 6: Tracker reset that reuses a previous ID
    # -------------------------------------------------------------------------
    def test_06_tracker_reset_session_scoping(self):
        """Track ID reused after a tracker reset (new session ID) is not treated as continuing."""
        lat1, lon1 = 12.972000, 77.594500
        det_sess1 = self._det(track_id=1, source_id="S1", tracker_session_id="session_001")
        t1, _ = self.manager.process_detection(det_sess1, self._geo(lat1, lon1), "S1", timestamp=1.0)
        self.assertEqual(t1.task_id, "task_001")

        # Tracker reset occurs: session becomes session_002.
        # Tracker happens to reassign local track_id=1 to a DIFFERENT target 45m away
        lat2, lon2 = 12.972400, 77.594500  # ~45m away
        det_sess2 = self._det(track_id=1, source_id="S1", tracker_session_id="session_002")
        t2, is_new2 = self.manager.process_detection(det_sess2, self._geo(lat2, lon2), "S1", timestamp=5.0)

        self.assertTrue(is_new2, "Reused track_id=1 under new session at 45m must not associate with task_001")
        self.assertEqual(t2.task_id, "task_002")

    # -------------------------------------------------------------------------
    # Scenario 7: Track disappears and later returns under a different ID
    # -------------------------------------------------------------------------
    def test_07_track_loss_and_reacquisition_under_new_id(self):
        """Track drops out and returns under a new ID; spatial proximity matches existing task."""
        lat, lon = 12.972000, 77.594500
        det1 = self._det(track_id=1, source_id="S1", frame_id="f_01")
        t1, _ = self.manager.process_detection(det1, self._geo(lat, lon), "S1", frame_id="f_01", timestamp=1.0)
        self.assertEqual(t1.task_id, "task_001")

        # Track is lost and re-acquired at same location under new track_id=42
        det_new_id = self._det(track_id=42, source_id="S1", frame_id="f_02")
        t2, is_new2 = self.manager.process_detection(det_new_id, self._geo(lat + 0.000015, lon), "S1", frame_id="f_02", timestamp=4.0)

        self.assertFalse(is_new_i := is_new2, "New track ID within 10m spatial radius must match existing task")
        self.assertEqual(t2.task_id, "task_001")
        # Verify active track updated to 42
        self.assertEqual(self.manager.survivor_records["task_001"].active_scoped_tracks.get("S1"), "S1:session_001:42")

    # -------------------------------------------------------------------------
    # Scenario 8: Conflicting track identity and geolocation evidence
    # -------------------------------------------------------------------------
    def test_08_conflicting_track_and_geolocation_distance(self):
        """Track ID does not override contradictory geolocation evidence (> 10m away)."""
        lat1, lon1 = 12.972000, 77.594500
        det1 = self._det(track_id=7, source_id="S1")
        t1, _ = self.manager.process_detection(det1, self._geo(lat1, lon1), "S1", timestamp=1.0)
        self.assertEqual(t1.task_id, "task_001")

        # Sighting claims track_id=7, but geolocation is 80m away (teleportation/glitch)
        lat_far = lat1 + 0.000720  # ~80m away
        det_far = self._det(track_id=7, source_id="S1")
        decision = self.associator.associate(
            detection=det_far,
            geo_estimate=self._geo(lat_far, lon1),
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            timestamp=2.0,
        )
        # Contradictory physical distance prevents association with task_001
        self.assertEqual(decision.status, AssociationStatus.NEW_SURVIVOR)

    # -------------------------------------------------------------------------
    # Scenario 9: Missing track IDs, invalid coordinates, and high uncertainty
    # -------------------------------------------------------------------------
    def test_09_missing_track_id_invalid_coords_and_high_uncertainty(self):
        """Validates behavior when track_id is None, coordinates are invalid, or uncertainty > 20m."""
        lat, lon = 12.972000, 77.594500

        # Sub-test 9A: track_id=None operates under normal spatial matching
        det_none = self._det(track_id=None, source_id="S1")
        t_none, is_new_none = self.manager.process_detection(det_none, self._geo(lat, lon), "S1", timestamp=1.0)
        self.assertTrue(is_new_none)
        self.assertEqual(t_none.task_id, "task_001")

        # Sub-test 9B: Invalid coordinates fail defensively
        with self.assertRaises(ValueError):
            self.manager.process_detection(det_none, None, "S1")  # type: ignore

        # Sub-test 9C: Uncertainty > 20m triggers AMBIGUOUS even with matching track_id
        det_unc = self._det(track_id=1, source_id="S1")
        geo_unc = self._geo(lat, lon, uncertainty_m=28.0)
        decision_unc = self.associator.associate(
            detection=det_unc,
            geo_estimate=geo_unc,
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            timestamp=2.0,
        )
        self.assertTrue(decision_unc.is_ambiguous, "Uncertainty > 20m must be flagged ambiguous regardless of track ID")
        self.assertEqual(decision_unc.status, AssociationStatus.AMBIGUOUS)

    # -------------------------------------------------------------------------
    # Scenario 10: Verified-task and failed-retry lifecycle preservation
    # -------------------------------------------------------------------------
    def test_10_verified_and_failed_lifecycle_with_tracklets(self):
        """Validates that VERIFIED_MATCH and FAILED_RETRY rules are preserved with tracklets."""
        lat, lon = 12.972000, 77.594500
        det = self._det(track_id=9, source_id="S1")
        task, _ = self.manager.process_detection(det, self._geo(lat, lon), "S1", timestamp=1.0)
        self.assertEqual(task.status, TaskStatus.ASSIGNED)

        # Mark task as VERIFIED
        task.status = TaskStatus.VERIFIED
        self.manager.drones[task.assigned_drone_id].status = DroneStatus.READY
        self.manager.drones[task.assigned_drone_id].active_task_id = None

        # Re-sighting of verified target under track_id=9
        t_ver, is_new_ver = self.manager.process_detection(det, self._geo(lat, lon), "S1", timestamp=2.0)
        self.assertFalse(is_new_ver)
        self.assertEqual(t_ver.task_id, "task_001")
        # Ensure no drone was allocated
        self.assertEqual(sum(1 for d in self.manager.drones.values() if d.status == DroneStatus.BUSY), 0)

        # Mark task as FAILED
        task.status = TaskStatus.FAILED
        t_fail, is_new_fail = self.manager.process_detection(det, self._geo(lat, lon), "S1", timestamp=3.0)
        self.assertTrue(is_new_fail, "Failed task re-sighting with tracklet must trigger retry task")
        self.assertEqual(t_fail.task_id, "task_002")
        self.assertEqual(t_fail.retry_of, "task_001")

    # -------------------------------------------------------------------------
    # Scenario 11: Stale active_scoped_tracks evicted on track reassignment
    # -------------------------------------------------------------------------
    def test_11_stale_active_scoped_tracks_evicted_on_reassignment(self):
        """When a scoped track is newly associated with another record, prior record evicts it."""
        lat1, lon1 = 12.972000, 77.594500
        det1 = self._det(track_id=1, source_id="S1")
        t1, _ = self.manager.process_detection(det1, self._geo(lat1, lon1), "S1", timestamp=1.0)
        self.assertEqual(t1.task_id, "task_001")
        self.assertEqual(self.manager.survivor_records["task_001"].active_scoped_tracks.get("S1"), "S1:session_001:1")

        # Now, far away (40m), a new survivor is detected claiming track_id=1 (e.g. tracker reassignment)
        lat2, lon2 = 12.972360, 77.594500
        det2 = self._det(track_id=1, source_id="S1")
        t2, is_new2 = self.manager.process_detection(det2, self._geo(lat2, lon2), "S1", timestamp=2.0)
        self.assertTrue(is_new2)
        self.assertEqual(t2.task_id, "task_002")

        # Invariant: task_002 owns S1:session_001:1; task_001 must have evicted it
        self.assertEqual(self.manager.survivor_records["task_002"].active_scoped_tracks.get("S1"), "S1:session_001:1")
        self.assertNotIn("S1", self.manager.survivor_records["task_001"].active_scoped_tracks)

    # -------------------------------------------------------------------------
    # Scenario 12: Two distinct targets in same frame with tracks never merged
    # -------------------------------------------------------------------------
    def test_12_two_distinct_targets_same_frame_with_tracks_never_merged(self):
        """Two detections in same frame with distinct tracks are never merged even if proximate."""
        lat1, lon1 = 12.972000, 77.594500
        lat2 = lat1 + 0.000018  # 2.0m away (< 3.0m min_distinct_separation_m)

        # Batch in frame f_same with two separate detections
        det1 = self._det(track_id=1, source_id="S1", bbox=BoundingBox(0.40, 0.40, 0.45, 0.45), frame_id="f_same")
        det2 = self._det(track_id=2, source_id="S1", bbox=BoundingBox(0.55, 0.55, 0.60, 0.60), frame_id="f_same")

        results = self.manager.process_detections_batch(
            detections_with_geo=[
                (det1, self._geo(lat1, lon1)),
                (det2, self._geo(lat2, lon1)),
            ],
            detected_by_drone_id="S1",
            frame_id="f_same",
            timestamp=1.0,
            minimum_confidence=0.50,
        )

        self.assertEqual(len(results), 2)
        self.assertTrue(results[0][1], "First detection must create task")
        self.assertTrue(results[1][1], "Second distinct tracklet in same frame must create separate task")
        self.assertEqual(results[0][0].task_id, "task_001")
        self.assertEqual(results[1][0].task_id, "task_002")
        self.assertEqual(len(self.manager.survivor_records), 2)

    # -------------------------------------------------------------------------
    # Scenario 13: Batch rollback restores active_scoped_tracks exactly
    # -------------------------------------------------------------------------
    def test_13_batch_rollback_restores_active_scoped_tracks_exactly(self):
        """Batch failure reverts active_scoped_tracks to exact pre-batch state."""
        lat, lon = 12.972000, 77.594500
        det1 = self._det(track_id=1, source_id="S1")
        self.manager.process_detection(det1, self._geo(lat, lon), "S1", timestamp=1.0)
        self.assertEqual(self.manager.survivor_records["task_001"].active_scoped_tracks.get("S1"), "S1:session_001:1")

        # Attempt invalid batch that updates track 1 on item 1 but fails validation on item 2
        det_upd = self._det(track_id=2, source_id="S1")
        invalid_det = Detection(class_id=0, class_name="pedestrian", confidence=0.10, bbox=BoundingBox(0, 0, 1, 1))

        with self.assertRaises(ValueError):
            self.manager.process_detections_batch(
                detections_with_geo=[
                    (det_upd, self._geo(lat, lon)),
                    (invalid_det, self._geo(lat, lon)),
                ],
                detected_by_drone_id="S1",
                minimum_confidence=0.50,
            )

        # Pre-batch track binding must remain intact
        self.assertEqual(self.manager.survivor_records["task_001"].active_scoped_tracks.get("S1"), "S1:session_001:1")

    # -------------------------------------------------------------------------
    # Scenario 14: Track expiry after extended gap falls back to spatial matching
    # -------------------------------------------------------------------------
    def test_14_track_expiry_after_extended_gap_falls_back_to_spatial_matching(self):
        """After an extended gap (> 5s), visual tracklet expires; spatial proximity safely matches."""
        lat, lon = 12.972000, 77.594500
        det1 = self._det(track_id=1, source_id="S1", frame_id="f_01")
        t1, is_new1 = self.manager.process_detection(det1, self._geo(lat, lon), "S1", frame_id="f_01", timestamp=1.0)
        self.assertTrue(is_new1)
        self.assertEqual(t1.task_id, "task_001")

        # 59 seconds later (t=60.0s), target is observed at same location with track_id=1
        det_late = self._det(track_id=1, source_id="S1", frame_id="f_late")
        decision = self.associator.associate(
            detection=det_late,
            geo_estimate=self._geo(lat + 0.000005, lon),
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            frame_id="f_late",
            timestamp=60.0,
        )

        # Tracklet expired (59s > 5s max_tracklet_gap_s): matched_by_track is False
        self.assertFalse(decision.matched_by_track, "Tracklet must be marked expired after 59s gap")
        self.assertEqual(decision.status, AssociationStatus.MATCH, "Spatial evidence must still associate survivor")
        self.assertEqual(decision.matched_task_id, "task_001")

        # Process through manager: updates record and refreshes timestamp to 60.0s
        t_late, is_new_late = self.manager.process_detection(det_late, self._geo(lat + 0.000005, lon), "S1", frame_id="f_late", timestamp=60.0)
        self.assertFalse(is_new_late)
        self.assertEqual(t_late.task_id, "task_001")
        rec = self.manager.survivor_records["task_001"]
        self.assertEqual(rec.detection_count, 2)
        self.assertEqual(rec.last_seen_timestamp, 60.0)

    # -------------------------------------------------------------------------
    # Scenario 15: Stale track cannot bypass proximity ambiguity between nearby people
    # -------------------------------------------------------------------------
    def test_15_stale_track_cannot_bypass_proximity_ambiguity_between_nearby_people(self):
        """Stale tracklet cannot suppress proximity ambiguity between two nearby survivors."""
        lat_a = 12.972000
        lat_b = 12.972018  # 2.0m North (< 3.0m min_distinct_separation_m)
        lon = 77.594500

        # Create two distinct survivors at t=1.0s in same frame f_01
        det_a = self._det(track_id=10, source_id="S1", bbox=BoundingBox(0.40, 0.40, 0.45, 0.45), frame_id="f_01")
        det_b = self._det(track_id=20, source_id="S1", bbox=BoundingBox(0.55, 0.55, 0.60, 0.60), frame_id="f_01")
        self.manager.process_detection(det_a, self._geo(lat_a, lon), "S1", frame_id="f_01", timestamp=1.0)
        self.manager.process_detection(det_b, self._geo(lat_b, lon), "S1", frame_id="f_01", timestamp=1.0)
        self.assertEqual(len(self.manager.survivor_records), 2)

        # 60s later (t=61.0s), a detection at the midpoint (1.0m from each) arrives claiming track_id=10
        lat_mid = (lat_a + lat_b) / 2.0
        det_mid = self._det(track_id=10, source_id="S1")
        decision = self.associator.associate(
            detection=det_mid,
            geo_estimate=self._geo(lat_mid, lon),
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            timestamp=61.0,
        )

        # Because the tracklet expired (60s > 5s), it cannot suppress proximity ambiguity!
        self.assertTrue(decision.is_ambiguous, "Stale track must NOT bypass proximity ambiguity between nearby targets")
        self.assertEqual(decision.status, AssociationStatus.AMBIGUOUS)

        # Counter-test: Within active window (t=2.0s, gap=1.0s <= 5.0s), tracklet DOES disambiguate
        decision_fresh = self.associator.associate(
            detection=det_mid,
            geo_estimate=self._geo(lat_mid, lon),
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            timestamp=2.0,
        )
        self.assertFalse(decision_fresh.is_ambiguous, "Fresh tracklet must disambiguate proximity conflict")
        self.assertEqual(decision_fresh.matched_task_id, "task_001")
        self.assertTrue(decision_fresh.matched_by_track)

    # -------------------------------------------------------------------------
    # Scenario 16: Track expiry boundary conditions (exact dt threshold)
    # -------------------------------------------------------------------------
    def test_16_track_expiry_boundary_conditions(self):
        """Validates exact temporal gap boundary: dt <= 5.0s is fresh, dt > 5.0s is expired."""
        associator = SurvivorAssociator(max_tracklet_gap_s=5.0)
        lat, lon = 12.972000, 77.594500
        det = self._det(track_id=99, source_id="S1")

        # Initial sighting at t=10.0s
        self.manager.process_detection(det, self._geo(lat, lon), "S1", timestamp=10.0)

        # Sub-test 16A: dt = 4.9s (t=14.9s) -> Fresh
        d_fresh = associator.associate(
            detection=det,
            geo_estimate=self._geo(lat, lon),
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            timestamp=14.9,
        )
        self.assertTrue(d_fresh.matched_by_track, "dt=4.9s must be evaluated as fresh tracklet")

        # Sub-test 16B: dt = 5.0s (t=15.0s) -> Fresh (exact boundary)
        d_bound = associator.associate(
            detection=det,
            geo_estimate=self._geo(lat, lon),
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            timestamp=15.0,
        )
        self.assertTrue(d_bound.matched_by_track, "dt=5.0s (exact boundary) must be fresh")

        # Sub-test 16C: dt = 5.1s (t=15.1s) -> Expired
        d_expired = associator.associate(
            detection=det,
            geo_estimate=self._geo(lat, lon),
            detected_by_drone_id="S1",
            survivor_records=self.manager.survivor_records,
            timestamp=15.1,
        )
        self.assertFalse(d_expired.matched_by_track, "dt=5.1s (>5.0s) must be evaluated as expired")
        self.assertEqual(d_expired.status, AssociationStatus.MATCH, "Spatial association must remain intact")


if __name__ == "__main__":
    unittest.main()
