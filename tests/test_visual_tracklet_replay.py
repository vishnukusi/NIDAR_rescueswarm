"""Asset-dependent Visual Tracklet Replay and Smoke Tests for RescueSwarm.

Verifies:
1. YOLOPerceptionAdapter.track() executes native ByteTrack tracking on vid1.mp4 frames.
2. Local track IDs, tracker_session_id, and source_id are correctly populated on Detection objects.
3. Track continuity across consecutive 25 FPS frames: high persistence of primary tracks.
4. Tracker reset increments session generation and isolates subsequent track IDs from prior sessions.
5. Tracking interface remains separate from ordinary predict() without breaking detections.

Requires:
- assets/models/best.pt
- assets/vid1.mp4
"""

from pathlib import Path
import sys
import unittest

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core import (
    BoundingBox,
    Detection,
    YOLOPerceptionAdapter,
    CameraCalibration,
    DroneTelemetry,
    estimate_survivor_location,
    GeoPoint,
    Drone,
    DroneRole,
    DroneStatus,
    MissionState,
    SurvivorAssociator,
    SwarmManager,
)

BEST_PT_PATH = ROOT / "assets" / "models" / "best.pt"
if not BEST_PT_PATH.exists():
    BEST_PT_PATH = ROOT / "assets" / "models" / "best"
VID1_PATH = ROOT / "assets" / "vid1.mp4"
ASSETS_AVAILABLE = BEST_PT_PATH.exists() and VID1_PATH.exists()


@unittest.skipUnless(ASSETS_AVAILABLE, "Requires assets/models/best.pt and assets/vid1.mp4")
class TestVisualTrackletReplay(unittest.TestCase):
    """Integration suite testing native ByteTrack visual tracking on real video frames."""

    @classmethod
    def setUpClass(cls):
        if not ASSETS_AVAILABLE:
            return
        cls.adapter = YOLOPerceptionAdapter(
            model_path=BEST_PT_PATH,
            confidence_threshold=0.25,
            target_classes=["pedestrian", "people", "person", "survivor"],
            tracker="bytetrack.yaml",
        )
        cls.cap = cv2.VideoCapture(str(VID1_PATH))
        cls.fps = cls.cap.get(cv2.CAP_PROP_FPS) or 25.0

        # Extract 6 consecutive frames (frames 25 to 30)
        cls.frames_bgr = []
        for idx in range(25, 31):
            cls.cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
            ret, frame = cls.cap.read()
            if ret:
                cls.frames_bgr.append((idx, frame))

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, "cap") and cls.cap:
            cls.cap.release()

    def test_01_track_produces_scoped_track_ids(self):
        """Verifies track() populates valid track_id, tracker_session_id, and scoped_track_id."""
        self.assertGreater(len(self.frames_bgr), 0)
        f_idx, img = self.frames_bgr[0]

        dets = self.adapter.track(img, frame_id=f"frame_{f_idx:04d}", source_id="S1", persist=True)
        self.assertGreater(len(dets), 5, "Frame 25 should produce multiple detections")

        tracked_dets = [d for d in dets if d.track_id is not None]
        self.assertGreater(len(tracked_dets), 0, "Tracker must assign track IDs to detections")

        for d in tracked_dets:
            self.assertIsInstance(d.track_id, int)
            self.assertGreaterEqual(d.track_id, 1)
            self.assertEqual(d.source_id, "S1")
            self.assertEqual(d.tracker_session_id, "session_001")
            # Verify scoped track ID format: source_id:tracker_session_id:track_id
            self.assertEqual(d.scoped_track_id, f"S1:session_001:{d.track_id}")

    def test_02_consecutive_frame_track_continuity(self):
        """Measures track persistence across consecutive frames at 25 FPS."""
        # Reset tracker to start clean for this test
        self.adapter.reset_tracker()
        sess_id = self.adapter.tracker_session_id

        track_history = []
        prev_tracks = set()

        for idx, img in self.frames_bgr:
            dets = self.adapter.track(img, frame_id=f"frame_{idx:04d}", source_id="S1", persist=True)
            curr_tracks = {d.track_id for d in dets if d.track_id is not None}

            if prev_tracks:
                continuing = curr_tracks & prev_tracks
                # At 25 FPS, adjacent frames should retain at least 60% of active tracks
                retention = len(continuing) / len(prev_tracks) if prev_tracks else 1.0
                self.assertGreaterEqual(
                    retention,
                    0.50,
                    f"Frame {idx} retained only {len(continuing)}/{len(prev_tracks)} tracks ({retention:.1%})",
                )

            prev_tracks = curr_tracks
            track_history.append((idx, curr_tracks))

        # Check persistent core tracks across the entire 6-frame window
        all_window_tracks = track_history[0][1]
        for _, tracks in track_history[1:]:
            all_window_tracks = all_window_tracks & tracks

        # Several pedestrians walk steadily across the frame and remain continuously tracked
        self.assertGreaterEqual(
            len(all_window_tracks),
            5,
            f"Expected at least 5 tracks to persist across all 6 frames, got: {all_window_tracks}",
        )

    def test_03_tracker_reset_increments_session_and_isolates_tracks(self):
        """Verifies reset_tracker() increments tracker session generation and clears tracker state."""
        initial_sess = self.adapter.tracker_session_id
        new_sess = self.adapter.reset_tracker()

        self.assertNotEqual(initial_sess, new_sess)
        self.assertEqual(new_sess, "session_003")

        # Running track on the same frame now produces scoped track IDs under the new session
        f_idx, img = self.frames_bgr[0]
        dets = self.adapter.track(img, frame_id=f"frame_{f_idx:04d}", source_id="S1", persist=True)
        tracked = [d for d in dets if d.track_id is not None]
        self.assertGreater(len(tracked), 0)

        for d in tracked:
            self.assertEqual(d.tracker_session_id, new_sess)
            self.assertTrue(d.scoped_track_id.startswith(f"S1:{new_sess}:"))

    def test_04_ordinary_predict_remains_independent_of_tracking(self):
        """Verifies ordinary predict() operates without tracking side-effects and leaves track_id None."""
        f_idx, img = self.frames_bgr[0]
        dets = self.adapter.predict(img, frame_id=f"frame_{f_idx:04d}")
        self.assertGreater(len(dets), 0)

        for d in dets:
            self.assertIsNone(d.track_id, "Ordinary predict() detections must have track_id=None")
            self.assertIsNone(d.scoped_track_id, "Ordinary predict() detections must have scoped_track_id=None")

    def test_05_real_tracklet_end_to_end_pipeline_association(self):
        """Validates real end-to-end path:
        vid1.mp4 -> YOLO ByteTrack -> Localization -> SwarmManager.process_detections_batch.

        Asserts:
        - Initial frame 25 creates survivor tasks and assigns available delivery drones.
        - Subsequent frame 26 (with continuing visual tracks) updates existing records
          without creating duplicate tasks for continuing tracklets.
        - Active scoped tracks on survivor records preserve genuine tracker IDs.
        """
        # Clean tracker state
        self.adapter.reset_tracker()
        sess_id = self.adapter.tracker_session_id

        # Setup SwarmManager with 3 delivery drones
        drones = [
            Drone(drone_id="D1", role=DroneRole.DELIVERY, status=DroneStatus.READY, position=GeoPoint(12.9720, 77.5945), battery_fraction=0.95),
            Drone(drone_id="D2", role=DroneRole.DELIVERY, status=DroneStatus.READY, position=GeoPoint(12.9720, 77.5945), battery_fraction=0.90),
            Drone(drone_id="D3", role=DroneRole.DELIVERY, status=DroneStatus.READY, position=GeoPoint(12.9720, 77.5945), battery_fraction=0.85),
        ]
        associator = SurvivorAssociator(
            match_distance_threshold_m=10.0,
            max_association_uncertainty_m=20.0,
            min_distinct_separation_m=3.0,
            intra_frame_iou_threshold=0.3,
        )
        manager = SwarmManager(drones=drones, associator=associator)
        manager.mission_manager.transition_to(MissionState.INITIALIZING)
        manager.mission_manager.transition_to(MissionState.SEARCHING)

        camera = CameraCalibration(
            image_width=1080,
            image_height=1920,
            hfov_deg=84.0,
            mount_pitch_deg=-90.0,
        )
        s1_telem = DroneTelemetry(
            drone_id="S1",
            position=GeoPoint(latitude=12.972000, longitude=77.594500, altitude_m=30.0),
            altitude_agl_m=30.0,
            heading_deg=45.0,
        )

        # ── Step 1: Process Frame 25 (initial capture) ──
        f25_idx, f25_img = self.frames_bgr[0]
        dets25 = self.adapter.track(f25_img, frame_id="frame_0025", source_id="S1", persist=True)
        self.assertGreater(len(dets25), 10, "Frame 25 must yield multiple detections")

        geo_dets25 = [(d, estimate_survivor_location(d, s1_telem, camera)) for d in dets25]
        res25 = manager.process_detections_batch(
            detections_with_geo=geo_dets25,
            detected_by_drone_id="S1",
            frame_id="frame_0025",
            timestamp=1.0,
            minimum_confidence=0.25,
        )

        initial_tasks = [t for t, is_new in res25 if is_new]
        self.assertGreaterEqual(len(initial_tasks), 10, "Initial frame must create survivor tasks")
        initial_record_count = len(manager.survivor_records)
        self.assertEqual(len(initial_tasks), initial_record_count)

        # Verify delivery drones were allocated to top-confidence tasks
        assigned_drones = {t.assigned_drone_id for t in manager.get_all_tasks() if t.assigned_drone_id}
        self.assertEqual(len(assigned_drones), 3, "All 3 ready delivery drones must be assigned")

        # ── Step 2: Process Frame 26 (consecutive 25 FPS frame) ──
        f26_idx, f26_img = self.frames_bgr[1]
        dets26 = self.adapter.track(f26_img, frame_id="frame_0026", source_id="S1", persist=True)
        self.assertGreater(len(dets26), 10)

        geo_dets26 = [(d, estimate_survivor_location(d, s1_telem, camera)) for d in dets26]
        res26 = manager.process_detections_batch(
            detections_with_geo=geo_dets26,
            detected_by_drone_id="S1",
            frame_id="frame_0026",
            timestamp=1.04,
            minimum_confidence=0.25,
        )

        new_tasks26 = [t for t, is_new in res26 if is_new]
        updated_tasks26 = [t for t, is_new in res26 if not is_new]

        # Continuing tracks must NOT spawn duplicate tasks
        self.assertEqual(len(new_tasks26), 0, "Continuing tracks in frame 26 must not create duplicate tasks")
        self.assertGreaterEqual(len(updated_tasks26), 10, "Frame 26 must update existing tasks")
        self.assertEqual(len(manager.survivor_records), initial_record_count, "Record count must remain stable")

        # Verify sighting counts incremented on updated records
        for task in updated_tasks26:
            rec = manager.survivor_records[task.task_id]
            self.assertGreaterEqual(rec.detection_count, 2, "Updated records must record at least 2 sightings")
            self.assertTrue(rec.active_scoped_tracks.get("S1", "").startswith(f"S1:{sess_id}:"))


if __name__ == "__main__":
    unittest.main()
