"""Asset-dependent Multi-Frame Survivor Association Validation Tests for RescueSwarm.

Evaluates genuine Ultralytics YOLO model inference on real video asset (vid1.mp4):
1. Separates image-level 2D tracking tests (bounding-box IoU) from geolocation tests.
2. Measures per-frame detection count, new vs matched survivor records, ambiguous sightings,
   rescue tasks created, and delivery drone allocations across consecutive and sampled frames.
3. Tests both consecutive high-rate frames (25 FPS, frames 25..35) and sampled frames (1 Hz,
   frames 0, 25, 50, 75, 100, 125, 150).
4. Strictly does NOT assume the video represents aerial drone footage or valid flight geometry.
   Labels all vehicle pose as synthetic test fixtures.
5. Does NOT fabricate tracking identities or claim ground-truth accuracy.

Requires:
- assets/models/best.pt
- assets/vid1.mp4
"""

import math
from pathlib import Path
import sys
import unittest

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core import (
    BoundingBox,
    CameraCalibration,
    Detection,
    Drone,
    DroneRole,
    DroneStatus,
    DroneTelemetry,
    GeoPoint,
    MissionState,
    RescueTask,
    SwarmManager,
    TaskStatus,
    YOLOPerceptionAdapter,
    estimate_survivor_location,
    extract_frame_from_video,
)
from rescue_swarm_core.association import AssociationDecision, AssociationStatus, SurvivorAssociator

BEST_PT_PATH = ROOT / "assets" / "models" / "best.pt"
if not BEST_PT_PATH.exists():
    BEST_PT_PATH = ROOT / "assets" / "models" / "best"
VID1_PATH = ROOT / "assets" / "vid1.mp4"
ASSETS_AVAILABLE = BEST_PT_PATH.exists() and VID1_PATH.exists()

CACHE_DIR = ROOT / "assets" / "test_cache_frames"


@unittest.skipUnless(ASSETS_AVAILABLE, "Requires assets/models/best.pt and assets/vid1.mp4")
class TestMultiFrameAssociationReplay(unittest.TestCase):
    """Integration suite measuring multi-frame perception and association on vid1.mp4."""

    @classmethod
    def setUpClass(cls):
        if not ASSETS_AVAILABLE:
            return
        CACHE_DIR.mkdir(parents=True, exist_ok=True)
        cls.adapter = YOLOPerceptionAdapter(
            model_path=BEST_PT_PATH,
            confidence_threshold=0.25,
            target_classes=["pedestrian", "people", "person", "survivor"],
        )
        cls.calibration = CameraCalibration(image_width=1080, image_height=1920, hfov_deg=84.0)
        # Explicit synthetic drone pose fixture: nadir at 30m AGL, heading 45 deg
        cls.synthetic_telemetry = DroneTelemetry(
            drone_id="S1",
            position=GeoPoint(latitude=12.972000, longitude=77.594500, altitude_m=30.0),
            altitude_agl_m=30.0,
            heading_deg=45.0,
        )

        # Open video and cache test frames
        cls.cap = cv2.VideoCapture(str(VID1_PATH))
        cls.fps = cls.cap.get(cv2.CAP_PROP_FPS) or 25.0
        cls.total_frames = int(cls.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        # Sequences under test:
        # Sequence A: 11 consecutive frames at 25 FPS (frames 25..35, dt = 0.04s)
        cls.consec_indices = list(range(25, 36))
        # Sequence B: 7 sampled frames spanning 6 seconds (frames 0, 25, 50, 75, 100, 125, 150)
        cls.sampled_indices = [0, 25, 50, 75, 100, 125, 150]

        all_needed = sorted(set(cls.consec_indices + cls.sampled_indices))
        cls.cached_frames = {}
        for f_idx in all_needed:
            f_path = CACHE_DIR / f"frame_{f_idx:04d}.jpg"
            if not f_path.exists():
                cls.cap.set(cv2.CAP_PROP_POS_FRAMES, f_idx)
                ret, frame_bgr = cls.cap.read()
                if ret:
                    cv2.imwrite(str(f_path), frame_bgr)
            cls.cached_frames[f_idx] = f_path

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls, "cap") and cls.cap:
            cls.cap.release()

    def _extract_and_localize_frame(self, frame_idx: int):
        f_path = self.cached_frames[frame_idx]
        frame_id = f"frame_{frame_idx:04d}"
        dets = self.adapter.predict(f_path, frame_id=frame_id)

        pairs = []
        for det in dets:
            geo = estimate_survivor_location(
                detection=det,
                telemetry=self.synthetic_telemetry,
                camera=self.calibration,
            )
            pairs.append((det, geo))
        return dets, pairs

    # -------------------------------------------------------------------------
    # Test 1: Image-level tracking evaluation (pure 2D IoU, separated from geometry)
    # -------------------------------------------------------------------------
    def test_01_image_level_temporal_iou_across_consecutive_frames(self):
        """Measures 2D bounding-box IoU across consecutive frames (25 FPS, dt = 0.04s)
        independent of geolocation or drone camera calibration.
        """
        dets_25, _ = self._extract_and_localize_frame(25)
        dets_26, _ = self._extract_and_localize_frame(26)

        self.assertGreater(len(dets_25), 10, "Frame 25 must have multiple genuine detections")
        self.assertGreater(len(dets_26), 10, "Frame 26 must have multiple genuine detections")

        # Compute maximal 2D bounding-box IoU for each detection in frame 26 against frame 25
        high_overlap_matches = 0
        total_eval = len(dets_26)

        for d26 in dets_26:
            best_iou = max((d26.bbox.iou(d25.bbox) for d25 in dets_25), default=0.0)
            if best_iou >= 0.50:
                high_overlap_matches += 1

        overlap_fraction = high_overlap_matches / total_eval
        # At 25 FPS, pedestrian displacement in pixel space is small. Majority of persistent
        # detections maintain >= 0.50 IoU with the preceding frame.
        self.assertGreaterEqual(
            overlap_fraction,
            0.60,
            f"At least 60% of detections at 25 FPS should have IoU >= 0.50 (got {overlap_fraction:.1%})",
        )

    # -------------------------------------------------------------------------
    # Test 2: Consecutive frame replay (frames 25..35) association measurements
    # -------------------------------------------------------------------------
    def test_02_consecutive_frame_association_measurements(self):
        """Replays 11 consecutive frames (frames 25 to 35) through SwarmManager and records
        per-frame detections, new tasks, matches, ambiguous sightings, and allocations.
        """
        # Create fleet of 6 delivery drones
        delivery_drones = [
            Drone(drone_id=f"D{i}", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.60 + 0.05*i)
            for i in range(1, 7)
        ]
        associator = SurvivorAssociator(
            match_distance_threshold_m=10.0,
            max_association_uncertainty_m=20.0,
            min_distinct_separation_m=3.0,
            intra_frame_iou_threshold=0.3,
        )
        manager = SwarmManager(drones=delivery_drones, associator=associator)
        manager.mission_manager.transition_to(MissionState.INITIALIZING)
        manager.mission_manager.transition_to(MissionState.SEARCHING)

        per_frame_records = []

        for f_idx in self.consec_indices:
            t_sec = f_idx / self.fps
            frame_id = f"frame_{f_idx:04d}"
            dets, pairs = self._extract_and_localize_frame(f_idx)

            tasks_before = len(manager.get_all_tasks())
            drones_before = sum(1 for d in manager.drones.values() if d.status == DroneStatus.BUSY)

            batch_results = manager.process_detections_batch(
                detections_with_geo=pairs,
                detected_by_drone_id="S1",
                frame_id=frame_id,
                timestamp=t_sec,
                minimum_confidence=0.25,
            )

            tasks_after = len(manager.get_all_tasks())
            drones_after = sum(1 for d in manager.drones.values() if d.status == DroneStatus.BUSY)

            num_new = sum(1 for task, is_new in batch_results if is_new)
            num_matched = sum(1 for task, is_new in batch_results if not is_new)
            total_ambiguous = sum(r.ambiguous_sightings for r in manager.survivor_records.values())

            record = {
                "frame_idx": f_idx,
                "timestamp": round(t_sec, 3),
                "detections": len(dets),
                "new_survivor_records": num_new,
                "matched_observations": num_matched,
                "total_ambiguous_sightings": total_ambiguous,
                "rescue_tasks_created": tasks_after - tasks_before,
                "total_tasks": tasks_after,
                "busy_drones": drones_after,
            }
            per_frame_records.append(record)

        # Verification of baseline association stability across consecutive frames:
        # 1. First frame (25) initializes tasks (17 new tasks, allocating 6 available drones)
        self.assertEqual(per_frame_records[0]["frame_idx"], 25)
        self.assertGreaterEqual(per_frame_records[0]["new_survivor_records"], 15)
        self.assertEqual(per_frame_records[0]["busy_drones"], 6)

        # 2. Subsequent consecutive frames (frames 27..34) match the vast majority of detections
        for rec in per_frame_records[2:10]:  # frames 27..34
            match_rate = rec["matched_observations"] / rec["detections"]
            self.assertGreaterEqual(
                match_rate,
                0.85,
                f"Frame {rec['frame_idx']} match rate {match_rate:.1%} must be >= 85%",
            )
            self.assertLessEqual(rec["rescue_tasks_created"], 1)

        # 3. Ambiguity increases due to dense pedestrian clusters (< 3m physical separation)
        final_ambiguous = per_frame_records[-1]["total_ambiguous_sightings"]
        self.assertGreater(
            final_ambiguous,
            20,
            "Dense pedestrian crowd in vid1.mp4 must accumulate ambiguous sightings under 3m separation gating",
        )

    # -------------------------------------------------------------------------
    # Test 3: Sampled frame replay (1 Hz, frames 0..150) association measurements
    # -------------------------------------------------------------------------
    def test_03_sampled_frame_association_measurements(self):
        """Replays 7 sampled frames at 1.0s intervals (frames 0, 25, 50, 75, 100, 125, 150)
        and measures survivor association under low-frequency snapshot conditions.
        """
        delivery_drones = [
            Drone(drone_id=f"D{i}", role=DroneRole.DELIVERY, status=DroneStatus.READY, battery_fraction=0.90)
            for i in range(1, 10)
        ]
        associator = SurvivorAssociator(
            match_distance_threshold_m=10.0,
            max_association_uncertainty_m=20.0,
            min_distinct_separation_m=3.0,
            intra_frame_iou_threshold=0.3,
        )
        manager = SwarmManager(drones=delivery_drones, associator=associator)
        manager.mission_manager.transition_to(MissionState.INITIALIZING)
        manager.mission_manager.transition_to(MissionState.SEARCHING)

        per_frame_records = []
        for f_idx in self.sampled_indices:
            t_sec = f_idx / self.fps
            frame_id = f"frame_{f_idx:04d}"
            dets, pairs = self._extract_and_localize_frame(f_idx)

            tasks_before = len(manager.get_all_tasks())
            batch_results = manager.process_detections_batch(
                detections_with_geo=pairs,
                detected_by_drone_id="S1",
                frame_id=frame_id,
                timestamp=t_sec,
                minimum_confidence=0.25,
            )
            tasks_after = len(manager.get_all_tasks())

            num_new = sum(1 for task, is_new in batch_results if is_new)
            num_matched = sum(1 for task, is_new in batch_results if not is_new)
            total_ambiguous = sum(r.ambiguous_sightings for r in manager.survivor_records.values())

            per_frame_records.append({
                "frame_idx": f_idx,
                "timestamp": round(t_sec, 3),
                "detections": len(dets),
                "new_survivor_records": num_new,
                "matched_observations": num_matched,
                "total_ambiguous_sightings": total_ambiguous,
                "rescue_tasks_created": tasks_after - tasks_before,
                "total_tasks": tasks_after,
            })

        # At 1 Hz sampling, pedestrians walk across the field of view over 6 seconds.
        # Cumulative tasks grow from ~15 in frame 0 to > 25 in frame 150 because moving
        # pedestrians displace beyond the 10m static spatial match radius.
        initial_tasks = per_frame_records[0]["total_tasks"]
        final_tasks = per_frame_records[-1]["total_tasks"]
        self.assertGreater(
            final_tasks,
            initial_tasks,
            "1 Hz sampled frames over 6s must exhibit task growth due to pedestrian movement > 10m",
        )


if __name__ == "__main__":
    unittest.main()
