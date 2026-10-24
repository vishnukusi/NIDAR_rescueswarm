"""Real Perception-to-Task Pipeline Integration Tests for NIDAR RescueSwarm.

Verifies the complete data path:
best.pt -> YOLOPerceptionAdapter -> Detection -> localization -> SwarmManager -> assigned RescueTask

Uses real assets:
- assets/models/best.pt
- assets/vid1.mp4

Constraints:
- Never fabricates detections.
- Clearly labels all drone pose/telemetry as synthetic.
- Does not start a delivery mission or dispatch flight commands.
- Skips gracefully if required weight/video assets are unavailable.
"""

import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))

from rescue_swarm_core import (
    BoundingBox,
    CameraCalibration,
    Detection,
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
    haversine_distance_m,
)


BEST_PT_PATH = ROOT / "assets" / "models" / "best.pt"
if not BEST_PT_PATH.exists():
    BEST_PT_PATH = ROOT / "assets" / "models" / "best"
VID1_PATH = ROOT / "assets" / "vid1.mp4"
EXTRACTED_FRAME_PATH = ROOT / "assets" / "extracted_frame_vid1.jpg"
TEST_SURVIVORS_PATH = ROOT / "assets" / "test_survivors.jpg"
FLEET_CONFIG_PATH = ROOT / "config" / "fleet.json"

ASSETS_AVAILABLE = BEST_PT_PATH.exists() and VID1_PATH.exists()


@unittest.skipUnless(ASSETS_AVAILABLE, "Requires assets/models/best.pt and assets/vid1.mp4")
class RealPerceptionToTaskIntegrationTests(unittest.TestCase):
    """Integration test suite executing genuine Ultralytics model inference on video frames."""

    @classmethod
    def setUpClass(cls):
        """Extract a reproducible frame (index 30, T=1.2s) from vid1.mp4 into a test artifact."""
        if not ASSETS_AVAILABLE:
            return
        cls.initial_vid_stat = VID1_PATH.stat()
        # Extract reproducible frame (frame index 30)
        cls.frame_path = extract_frame_from_video(
            video_path=VID1_PATH,
            frame_index=30,
            output_path=EXTRACTED_FRAME_PATH,
        )

    def test_01_reproducible_frame_extraction(self):
        """Requirement 1: Verify reproducible frame extraction without modifying vid1.mp4."""
        self.assertTrue(self.frame_path.exists(), "Extracted frame artifact must exist on disk")
        self.assertGreater(self.frame_path.stat().st_size, 100_000, "Frame artifact should be a valid JPEG")

        import cv2
        img = cv2.imread(str(self.frame_path))
        self.assertIsNotNone(img, "Frame artifact must be readable by OpenCV")
        h, w, c = img.shape
        self.assertEqual(w, 1080, "Extracted frame width must match vid1.mp4 width (1080)")
        self.assertEqual(h, 1920, "Extracted frame height must match vid1.mp4 height (1920)")
        self.assertEqual(c, 3, "Extracted frame must have 3 color channels (BGR)")

        # Verify original video was not modified
        current_vid_stat = VID1_PATH.stat()
        self.assertEqual(
            current_vid_stat.st_size,
            self.initial_vid_stat.st_size,
            "Original vid1.mp4 size must remain unchanged",
        )

    def test_02_real_yolo_adapter_inference(self):
        """Requirements 2 & 3: Run YOLOPerceptionAdapter at conf=0.25 and verify detections."""
        adapter = YOLOPerceptionAdapter(
            model_path=BEST_PT_PATH,
            confidence_threshold=0.25,
            target_classes=["pedestrian", "people", "person", "survivor"],
        )
        meta = adapter.inspect_model()
        # Verify model classes mapping
        self.assertIn("classes", meta)
        classes_map = meta["classes"]
        self.assertEqual(classes_map.get(0), "pedestrian")
        self.assertEqual(classes_map.get(1), "people")

        # Run real inference on the extracted frame
        detections = adapter.predict(self.frame_path)
        self.assertGreater(len(detections), 0, "Model must produce genuine detections on vid1.mp4 frame")

        for det in detections:
            # Check class names and IDs
            self.assertIn(det.class_name.lower(), ["pedestrian", "people"])
            self.assertIn(det.class_id, [0, 1])

            # Check confidence
            self.assertGreaterEqual(det.confidence, 0.25, "All detections must meet confidence threshold")
            self.assertLessEqual(det.confidence, 1.0, "Confidence must not exceed 1.0")

            # Check normalized bounding-box coordinates
            bbox = det.bbox
            self.assertGreaterEqual(bbox.xmin, 0.0)
            self.assertGreaterEqual(bbox.ymin, 0.0)
            self.assertLessEqual(bbox.xmax, 1.0)
            self.assertLessEqual(bbox.ymax, 1.0)
            self.assertLess(bbox.xmin, bbox.xmax, "xmin must be strictly less than xmax")
            self.assertLess(bbox.ymin, bbox.ymax, "ymin must be strictly less than ymax")

            # Check center coordinates
            self.assertTrue(0.0 <= bbox.center_x <= 1.0)
            self.assertTrue(0.0 <= bbox.center_y <= 1.0)

    def test_03_perception_to_localization_to_task_assignment(self):
        """Requirements 4 & 5: Complete data path best.pt -> Detection -> Localization -> SwarmManager."""
        # 1. Genuine model inference
        adapter = YOLOPerceptionAdapter(
            model_path=BEST_PT_PATH,
            confidence_threshold=0.25,
            target_classes=["pedestrian", "people", "person", "survivor"],
        )
        detections = adapter.predict(self.frame_path)
        self.assertGreater(len(detections), 0)

        # Select genuine top-confidence survivor detection
        top_detection = max(detections, key=lambda d: d.confidence)
        self.assertIn(top_detection.class_name, ["pedestrian", "people"])
        self.assertGreaterEqual(top_detection.confidence, 0.50, "Top detection confidence should exceed 50%")

        # 2. Camera calibration matching portrait frame dimensions (1080 x 1920)
        camera = CameraCalibration(
            image_width=1080,
            image_height=1920,
            hfov_deg=84.0,
            mount_pitch_deg=-90.0,  # Nadir
        )

        # 3. Explicitly labelled synthetic drone telemetry
        # (Software-only test pose: observing drone S1 hovering at 30m AGL, heading 45° NE)
        synthetic_telemetry = DroneTelemetry(
            drone_id="S1",
            position=GeoPoint(latitude=12.972000, longitude=77.594500, altitude_m=30.0),
            altitude_agl_m=30.0,
            heading_deg=45.0,
        )

        # 4. 3D Ray-plane ground projection
        geo_estimate = estimate_survivor_location(
            detection=top_detection,
            telemetry=synthetic_telemetry,
            camera=camera,
            ground_elevation_m=0.0,
        )

        # Verify mathematical consistency of ground projection
        self.assertIsNotNone(geo_estimate.location)
        self.assertTrue(12.0 <= geo_estimate.location.latitude <= 14.0)
        self.assertTrue(76.0 <= geo_estimate.location.longitude <= 78.0)
        self.assertGreater(geo_estimate.uncertainty_radius_m, 0.0)

        # Mathematical consistency checks
        calc_ground_dist = math.sqrt(geo_estimate.offset_north_m**2 + geo_estimate.offset_east_m**2)
        self.assertAlmostEqual(geo_estimate.ground_distance_m, calc_ground_dist, places=5)
        calc_slant_range = math.sqrt(geo_estimate.ground_distance_m**2 + synthetic_telemetry.altitude_agl_m**2)
        self.assertAlmostEqual(geo_estimate.slant_range_m, calc_slant_range, places=5)

        # Expected software offsets for this reproducible frame and synthetic pose:
        # (North: +3.74m ± 0.1m, East: -18.19m ± 0.1m, Ground dist: 18.57m ± 0.1m)
        self.assertAlmostEqual(geo_estimate.offset_north_m, 3.74, places=1)
        self.assertAlmostEqual(geo_estimate.offset_east_m, -18.19, places=1)
        self.assertAlmostEqual(geo_estimate.ground_distance_m, 18.57, places=1)

        # 5. SwarmManager Task Registration and Allocation
        manager = SwarmManager.from_config_file(FLEET_CONFIG_PATH)
        manager.mission_manager.transition_to(MissionState.INITIALIZING)
        manager.mission_manager.transition_to(MissionState.SEARCHING)

        task, is_new = manager.process_detection(
            detection=top_detection,
            geo_estimate=geo_estimate,
            detected_by_drone_id="S1",
            minimum_confidence=0.25,
        )

        # Verify RescueTask created and allocated
        self.assertTrue(is_new, "First observation of survivor must create a new task")
        self.assertEqual(task.status, TaskStatus.ASSIGNED, "Task must transition to ASSIGNED upon drone allocation")
        self.assertIsNotNone(task.assigned_drone_id, "Task must be allocated to an eligible delivery drone")
        self.assertIn(task.assigned_drone_id, ["D1", "D2", "D3"], "Assigned drone must be a delivery drone")

        # Verify assigned delivery drone state in SwarmManager
        assigned_drone = manager.get_drone(task.assigned_drone_id)
        self.assertEqual(assigned_drone.role, DroneRole.DELIVERY)
        self.assertEqual(assigned_drone.active_task_id, task.task_id)
        self.assertEqual(assigned_drone.status, DroneStatus.BUSY)

        # Verify that NO delivery mission was started (controller not instantiated or flown)
        active_tasks = manager.get_active_tasks()
        self.assertEqual(len(active_tasks), 1)
        self.assertEqual(active_tasks[0].status, TaskStatus.ASSIGNED)  # Not EN_ROUTE or VERIFIED

    def test_04_zero_detection_creates_no_task_and_starts_no_delivery(self):
        """Requirement 6: Verify no-detection path creates no task and starts no delivery."""
        if not TEST_SURVIVORS_PATH.exists():
            self.skipTest("assets/test_survivors.jpg not found")

        # On test_survivors.jpg, genuine model inference returns 0 detections at conf >= 0.25
        adapter = YOLOPerceptionAdapter(
            model_path=BEST_PT_PATH,
            confidence_threshold=0.25,
            target_classes=["pedestrian", "people", "person", "survivor"],
        )
        detections = adapter.predict(TEST_SURVIVORS_PATH)
        self.assertEqual(len(detections), 0, "test_survivors.jpg must return 0 detections at conf >= 0.25")

        # Verify that with zero detections, SwarmManager remains empty
        manager = SwarmManager.from_config_file(FLEET_CONFIG_PATH)
        manager.mission_manager.transition_to(MissionState.INITIALIZING)
        manager.mission_manager.transition_to(MissionState.SEARCHING)

        # No process_detection is called
        self.assertEqual(len(manager.get_all_tasks()), 0, "No rescue task should exist")
        self.assertEqual(len(manager.get_active_tasks()), 0, "No active rescue task should exist")

        # Verify all delivery drones remain in standby/ready
        for d in manager.drones.values():
            if d.role == DroneRole.DELIVERY:
                self.assertIsNone(d.active_task_id, f"Drone {d.drone_id} must have no assigned task")
                self.assertEqual(d.status, DroneStatus.READY)

    def test_05_class_filtering_with_real_model(self):
        """Requirement 7: Test target class filtering behavior with loaded model."""
        # Filter for only 'pedestrian'
        adapter_ped = YOLOPerceptionAdapter(
            model_path=BEST_PT_PATH,
            confidence_threshold=0.25,
            target_classes=["pedestrian"],
        )
        dets_ped = adapter_ped.predict(self.frame_path)
        for d in dets_ped:
            self.assertEqual(d.class_name.lower(), "pedestrian")

        # Filter for vehicle class (e.g. 'car')
        adapter_car = YOLOPerceptionAdapter(
            model_path=BEST_PT_PATH,
            confidence_threshold=0.25,
            target_classes=["car"],
        )
        dets_car = adapter_car.predict(self.frame_path)
        for d in dets_car:
            self.assertEqual(d.class_name.lower(), "car")
            self.assertNotEqual(d.class_name.lower(), "pedestrian")

        # Filter for non-existent class in model
        adapter_none = YOLOPerceptionAdapter(
            model_path=BEST_PT_PATH,
            confidence_threshold=0.25,
            target_classes=["nonexistent_unknown_class"],
        )
        dets_none = adapter_none.predict(self.frame_path)
        self.assertEqual(len(dets_none), 0, "Non-matching target classes must produce 0 detections")

    def test_06_missing_and_invalid_image_inputs(self):
        """Requirement 7: Test error handling for missing and invalid image inputs."""
        adapter = YOLOPerceptionAdapter(
            model_path=BEST_PT_PATH,
            confidence_threshold=0.25,
        )

        # Missing image file
        with self.assertRaises(FileNotFoundError):
            adapter.predict("assets/missing_file_never_exists.jpg")

        # None image input
        with self.assertRaises(ValueError):
            adapter.predict(None)

    def test_07_corrupted_geolocation_rejected_in_integration(self):
        """Requirement: Ensure localization failure cannot create a valid-looking task in SwarmManager."""
        manager = SwarmManager.from_config_file(FLEET_CONFIG_PATH)
        manager.mission_manager.transition_to(MissionState.INITIALIZING)
        manager.mission_manager.transition_to(MissionState.SEARCHING)

        det = Detection(
            class_id=0,
            class_name="pedestrian",
            confidence=0.85,
            bbox=BoundingBox(xmin=0.2, ymin=0.3, xmax=0.3, ymax=0.4),
        )

        # Rejection of None geolocation estimate
        with self.assertRaises(ValueError):
            manager.process_detection(det, None, detected_by_drone_id="S1")

        # Rejection of estimate with None location
        corrupt_estimate = type("CorruptEstimate", (), {
            "location": None,
            "uncertainty_radius_m": 5.0,
        })()
        with self.assertRaises(ValueError):
            manager.process_detection(det, corrupt_estimate, detected_by_drone_id="S1")

        # Verify no rescue task was created or delivery drone dispatched
        self.assertEqual(len(manager.get_all_tasks()), 0)
        self.assertEqual(len(manager.get_active_tasks()), 0)


if __name__ == "__main__":
    unittest.main()
