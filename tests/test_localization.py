import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.localization import (
    CameraCalibration,
    DroneTelemetry,
    LocalizationError,
    estimate_survivor_location,
    haversine_distance_m,
)
from rescue_swarm_core.models import GeoPoint
from rescue_swarm_core.perception import BoundingBox, Detection


class LocalizationTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraCalibration(
            image_width=1920,
            image_height=1080,
            hfov_deg=84.0,
            mount_pitch_deg=-90.0,  # Nadir
        )
        self.drone_pos = GeoPoint(latitude=12.9716, longitude=77.5946, altitude_m=50.0)

    def test_effective_vfov_calculation(self):
        # 16:9 aspect ratio should yield ~53.8 deg VFOV for 84 deg HFOV
        vfov = self.camera.effective_vfov_deg
        self.assertTrue(50.0 < vfov < 60.0)

    def test_direct_nadir_center_detection(self):
        telemetry = DroneTelemetry(
            drone_id="S1",
            position=self.drone_pos,
            altitude_agl_m=30.0,
            heading_deg=0.0,
        )
        detection = Detection(
            class_id=0,
            class_name="person",
            confidence=0.92,
            bbox=BoundingBox(xmin=0.48, ymin=0.48, xmax=0.52, ymax=0.52),
        )

        estimate = estimate_survivor_location(detection, telemetry, self.camera)
        self.assertAlmostEqual(estimate.offset_north_m, 0.0, places=1)
        self.assertAlmostEqual(estimate.offset_east_m, 0.0, places=1)
        self.assertAlmostEqual(estimate.location.latitude, self.drone_pos.latitude, places=5)
        self.assertAlmostEqual(estimate.location.longitude, self.drone_pos.longitude, places=5)
        self.assertAlmostEqual(estimate.ground_distance_m, 0.0, places=1)

    def test_off_center_detection_displaces_target_location(self):
        telemetry = DroneTelemetry(
            drone_id="S1",
            position=self.drone_pos,
            altitude_agl_m=40.0,
            heading_deg=0.0,  # North
        )
        # Target located on right side of image (East when heading North)
        detection = Detection(
            class_id=0,
            class_name="person",
            confidence=0.88,
            bbox=BoundingBox(xmin=0.75, ymin=0.48, xmax=0.85, ymax=0.52),
        )

        estimate = estimate_survivor_location(detection, telemetry, self.camera)
        # Should be strictly East of the drone
        self.assertGreater(estimate.offset_east_m, 5.0)
        self.assertAlmostEqual(estimate.offset_north_m, 0.0, places=1)
        self.assertGreater(estimate.location.longitude, self.drone_pos.longitude)
        self.assertAlmostEqual(estimate.location.latitude, self.drone_pos.latitude, places=5)
        # Target location must NOT equal drone position
        self.assertNotEqual(estimate.location.longitude, self.drone_pos.longitude)

    def test_drone_heading_rotates_target_bearing(self):
        # Heading 90 deg (facing East): image right is now South
        telemetry = DroneTelemetry(
            drone_id="S2",
            position=self.drone_pos,
            altitude_agl_m=40.0,
            heading_deg=90.0,
        )
        detection = Detection(
            class_id=0,
            class_name="person",
            confidence=0.85,
            bbox=BoundingBox(xmin=0.75, ymin=0.48, xmax=0.85, ymax=0.52),
        )

        estimate = estimate_survivor_location(detection, telemetry, self.camera)
        # Target should be South of the drone (negative North offset)
        self.assertLess(estimate.offset_north_m, -5.0)
        self.assertAlmostEqual(estimate.offset_east_m, 0.0, places=1)
        self.assertLess(estimate.location.latitude, self.drone_pos.latitude)

    def test_ray_above_horizon_raises_localization_error(self):
        # Camera tilted forward horizontal (0 deg pitch) while drone pitched up 15 deg
        camera_forward = CameraCalibration(mount_pitch_deg=0.0)
        telemetry_up = DroneTelemetry(
            drone_id="S3",
            position=self.drone_pos,
            altitude_agl_m=25.0,
            heading_deg=0.0,
            pitch_deg=15.0,  # Pitched up
        )
        detection = Detection(
            class_id=0,
            class_name="person",
            confidence=0.80,
            bbox=BoundingBox(xmin=0.45, ymin=0.1, xmax=0.55, ymax=0.2),  # Looking near top of frame
        )
        with self.assertRaises(LocalizationError):
            estimate_survivor_location(detection, telemetry_up, camera_forward)

    def test_haversine_distance(self):
        # Points ~111km apart (1 degree latitude)
        p1 = GeoPoint(0.0, 0.0)
        p2 = GeoPoint(1.0, 0.0)
        dist = haversine_distance_m(p1, p2)
        self.assertTrue(110000.0 < dist < 112000.0)

    def test_camera_mount_yaw_and_roll_offsets(self):
        # Camera mounted with 90 deg yaw offset (pointing right relative to vehicle heading)
        camera_yaw_90 = CameraCalibration(mount_pitch_deg=-90.0, mount_yaw_deg=90.0)
        telemetry = DroneTelemetry(
            drone_id="S1",
            position=self.drone_pos,
            altitude_agl_m=30.0,
            heading_deg=0.0,  # North
        )
        # Bbox off-center: target on top of frame (in optical camera -Y)
        detection = Detection(
            class_id=0,
            class_name="person",
            confidence=0.90,
            bbox=BoundingBox(xmin=0.48, ymin=0.20, xmax=0.52, ymax=0.25),
        )
        est = estimate_survivor_location(detection, telemetry, camera_yaw_90)
        # Because camera is yawed 90 deg clockwise (facing East), camera up/forward points East
        self.assertNotEqual(est.offset_east_m, 0.0)
        self.assertTrue(math.isfinite(est.location.latitude))
        self.assertTrue(math.isfinite(est.location.longitude))

    def test_deterministic_center_detection_zero_horizontal_offset(self):
        """Image center on level nadir camera must yield exactly 0 ground offset."""
        telemetry = DroneTelemetry(
            drone_id="S1",
            position=self.drone_pos,
            altitude_agl_m=35.0,
            heading_deg=0.0,
            pitch_deg=0.0,
            roll_deg=0.0,
        )
        detection = Detection(
            class_id=0,
            class_name="person",
            confidence=0.90,
            bbox=BoundingBox(xmin=0.49, ymin=0.49, xmax=0.51, ymax=0.51),
        )
        est = estimate_survivor_location(detection, telemetry, self.camera)
        self.assertAlmostEqual(est.offset_north_m, 0.0, places=5)
        self.assertAlmostEqual(est.offset_east_m, 0.0, places=5)
        self.assertAlmostEqual(est.ground_distance_m, 0.0, places=5)
        self.assertAlmostEqual(est.slant_range_m, 35.0, places=5)
        self.assertAlmostEqual(est.location.latitude, self.drone_pos.latitude, places=7)
        self.assertAlmostEqual(est.location.longitude, self.drone_pos.longitude, places=7)

    def test_deterministic_symmetric_lateral_offsets(self):
        """Symmetric left/right detections must produce symmetric, opposite East offsets."""
        square_cam = CameraCalibration(
            image_width=1000,
            image_height=1000,
            hfov_deg=90.0,
            mount_pitch_deg=-90.0,
        )
        telemetry = DroneTelemetry(
            drone_id="S1",
            position=self.drone_pos,
            altitude_agl_m=30.0,
            heading_deg=0.0,
        )
        # Right detection: cx = 0.75 -> tan(45°) = 1.0, offset = +0.5 * 30m = +15m East
        det_right = Detection(
            class_id=0,
            class_name="person",
            confidence=0.85,
            bbox=BoundingBox(xmin=0.70, ymin=0.45, xmax=0.80, ymax=0.55),
        )
        # Left detection: cx = 0.25 -> offset = -0.5 * 30m = -15m East
        det_left = Detection(
            class_id=0,
            class_name="person",
            confidence=0.85,
            bbox=BoundingBox(xmin=0.20, ymin=0.45, xmax=0.30, ymax=0.55),
        )
        est_right = estimate_survivor_location(det_right, telemetry, square_cam)
        est_left = estimate_survivor_location(det_left, telemetry, square_cam)

        self.assertAlmostEqual(est_right.offset_east_m, 15.0, places=4)
        self.assertAlmostEqual(est_left.offset_east_m, -15.0, places=4)
        self.assertAlmostEqual(est_right.offset_east_m, -est_left.offset_east_m, places=6)
        self.assertAlmostEqual(est_right.offset_north_m, 0.0, places=6)
        self.assertAlmostEqual(est_left.offset_north_m, 0.0, places=6)
        self.assertAlmostEqual(est_right.ground_distance_m, 15.0, places=4)
        self.assertAlmostEqual(est_left.ground_distance_m, 15.0, places=4)
        expected_slant = math.sqrt(15.0**2 + 30.0**2)
        self.assertAlmostEqual(est_right.slant_range_m, expected_slant, places=4)
        self.assertAlmostEqual(est_left.slant_range_m, expected_slant, places=4)

    def test_deterministic_symmetric_longitudinal_offsets(self):
        """Symmetric upper/lower detections must produce symmetric North/South offsets."""
        square_cam = CameraCalibration(
            image_width=1000,
            image_height=1000,
            hfov_deg=90.0,
            mount_pitch_deg=-90.0,
        )
        telemetry = DroneTelemetry(
            drone_id="S1",
            position=self.drone_pos,
            altitude_agl_m=30.0,
            heading_deg=0.0,
        )
        # Upper detection (top of frame): cy = 0.25 -> points forward (+North)
        det_upper = Detection(
            class_id=0,
            class_name="person",
            confidence=0.85,
            bbox=BoundingBox(xmin=0.45, ymin=0.20, xmax=0.55, ymax=0.30),
        )
        # Lower detection (bottom of frame): cy = 0.75 -> points aft (-North / South)
        det_lower = Detection(
            class_id=0,
            class_name="person",
            confidence=0.85,
            bbox=BoundingBox(xmin=0.45, ymin=0.70, xmax=0.55, ymax=0.80),
        )
        est_upper = estimate_survivor_location(det_upper, telemetry, square_cam)
        est_lower = estimate_survivor_location(det_lower, telemetry, square_cam)

        self.assertAlmostEqual(est_upper.offset_north_m, 15.0, places=4)
        self.assertAlmostEqual(est_lower.offset_north_m, -15.0, places=4)
        self.assertAlmostEqual(est_upper.offset_north_m, -est_lower.offset_north_m, places=6)
        self.assertAlmostEqual(est_upper.offset_east_m, 0.0, places=6)
        self.assertAlmostEqual(est_lower.offset_east_m, 0.0, places=6)
        self.assertAlmostEqual(est_upper.ground_distance_m, 15.0, places=4)
        self.assertAlmostEqual(est_lower.ground_distance_m, 15.0, places=4)

    def test_deterministic_heading_rotation_invariance(self):
        """Heading rotation rotates North/East offsets while preserving ground distance."""
        square_cam = CameraCalibration(
            image_width=1000,
            image_height=1000,
            hfov_deg=90.0,
            mount_pitch_deg=-90.0,
        )
        # Target positioned 15m forward of drone (cy=0.25)
        det = Detection(
            class_id=0,
            class_name="person",
            confidence=0.85,
            bbox=BoundingBox(xmin=0.45, ymin=0.20, xmax=0.55, ymax=0.30),
        )
        headings_expected = [
            (0.0, 15.0, 0.0),       # North
            (90.0, 0.0, 15.0),      # East
            (180.0, -15.0, 0.0),    # South
            (270.0, 0.0, -15.0),    # West
            (45.0, 15.0 / math.sqrt(2), 15.0 / math.sqrt(2)),       # North-East
            (135.0, -15.0 / math.sqrt(2), 15.0 / math.sqrt(2)),    # South-East
        ]
        for heading, exp_north, exp_east in headings_expected:
            telem = DroneTelemetry(
                drone_id="S1",
                position=self.drone_pos,
                altitude_agl_m=30.0,
                heading_deg=heading,
            )
            est = estimate_survivor_location(det, telem, square_cam)
            self.assertAlmostEqual(est.offset_north_m, exp_north, places=4)
            self.assertAlmostEqual(est.offset_east_m, exp_east, places=4)
            self.assertAlmostEqual(est.ground_distance_m, 15.0, places=4)
            self.assertAlmostEqual(est.slant_range_m, math.sqrt(15.0**2 + 30.0**2), places=4)

    def test_deterministic_altitude_scaling_proportionality(self):
        """Ground offset scales strictly proportionally with AGL altitude for same ray."""
        square_cam = CameraCalibration(
            image_width=1000,
            image_height=1000,
            hfov_deg=90.0,
            mount_pitch_deg=-90.0,
        )
        det = Detection(
            class_id=0,
            class_name="person",
            confidence=0.85,
            bbox=BoundingBox(xmin=0.70, ymin=0.45, xmax=0.80, ymax=0.55),
        )
        altitudes = [10.0, 20.0, 40.0, 80.0]
        estimates = []
        for alt in altitudes:
            telem = DroneTelemetry(
                drone_id="S1",
                position=self.drone_pos,
                altitude_agl_m=alt,
                heading_deg=0.0,
            )
            est = estimate_survivor_location(det, telem, square_cam)
            estimates.append(est)
            # Analytically: offset_east = 0.5 * alt
            self.assertAlmostEqual(est.offset_east_m, 0.5 * alt, places=4)
            self.assertAlmostEqual(est.offset_north_m, 0.0, places=4)
            self.assertAlmostEqual(est.ground_distance_m, 0.5 * alt, places=4)

        # Check linear scaling ratios
        self.assertAlmostEqual(estimates[1].offset_east_m / estimates[0].offset_east_m, 2.0, places=6)
        self.assertAlmostEqual(estimates[2].offset_east_m / estimates[1].offset_east_m, 2.0, places=6)
        self.assertAlmostEqual(estimates[3].offset_east_m / estimates[2].offset_east_m, 2.0, places=6)

    def test_independent_analytical_geometry_tilted_camera(self):
        """Known tilted camera (mount_pitch=-45°) ray intersects at independently derived point."""
        # Camera tilted 45° down from horizontal
        tilted_cam = CameraCalibration(
            image_width=1920,
            image_height=1080,
            hfov_deg=84.0,
            mount_pitch_deg=-45.0,
        )
        telem = DroneTelemetry(
            drone_id="S1",
            position=self.drone_pos,
            altitude_agl_m=30.0,
            heading_deg=0.0,
        )
        # Optical center ray: points exactly 45° forward and down
        det_center = Detection(
            class_id=0,
            class_name="person",
            confidence=0.90,
            bbox=BoundingBox(xmin=0.49, ymin=0.49, xmax=0.51, ymax=0.51),
        )
        est = estimate_survivor_location(det_center, telem, tilted_cam)
        # Independently derived:
        # tan(45°) = 1.0 => ground_distance = 30.0 / tan(45°) = 30.0m North
        # East = 0.0m
        # Slant range = 30.0 / sin(45°) = 30 * sqrt(2) ≈ 42.4264m
        self.assertAlmostEqual(est.offset_north_m, 30.0, places=4)
        self.assertAlmostEqual(est.offset_east_m, 0.0, places=4)
        self.assertAlmostEqual(est.ground_distance_m, 30.0, places=4)
        self.assertAlmostEqual(est.slant_range_m, 30.0 * math.sqrt(2.0), places=4)

    def test_slant_range_and_ground_distance_pythagorean_consistency(self):
        """Slant range must satisfy slant_range^2 == ground_dist^2 + altitude^2 for any pose."""
        telem = DroneTelemetry(
            drone_id="S1",
            position=self.drone_pos,
            altitude_agl_m=42.5,
            heading_deg=67.0,
            pitch_deg=-5.0,
            roll_deg=3.0,
        )
        det = Detection(
            class_id=0,
            class_name="person",
            confidence=0.88,
            bbox=BoundingBox(xmin=0.25, ymin=0.35, xmax=0.35, ymax=0.45),
        )
        est = estimate_survivor_location(det, telem, self.camera)
        expected_hypot = math.sqrt(est.ground_distance_m**2 + telem.altitude_agl_m**2)
        self.assertAlmostEqual(est.slant_range_m, expected_hypot, places=4)

    def test_haversine_vs_local_flat_earth_consistency(self):
        """Haversine distance between drone and estimated target matches ground_distance within 0.1%."""
        telem = DroneTelemetry(
            drone_id="S1",
            position=self.drone_pos,
            altitude_agl_m=30.0,
            heading_deg=45.0,
        )
        det = Detection(
            class_id=0,
            class_name="person",
            confidence=0.85,
            bbox=BoundingBox(xmin=0.70, ymin=0.60, xmax=0.80, ymax=0.70),
        )
        est = estimate_survivor_location(det, telem, self.camera)
        calc_dist = haversine_distance_m(telem.position, est.location)
        self.assertAlmostEqual(calc_dist, est.ground_distance_m, delta=est.ground_distance_m * 0.001)

    def test_defensive_fail_fast_on_invalid_telemetry_and_camera(self):
        """Defensive validation catches invalid dimensions, FOVs, altitudes, and NaNs."""
        # Non-positive altitude
        with self.assertRaises(ValueError):
            DroneTelemetry(drone_id="S1", position=self.drone_pos, altitude_agl_m=0.0, heading_deg=0.0)
        with self.assertRaises(ValueError):
            DroneTelemetry(drone_id="S1", position=self.drone_pos, altitude_agl_m=-10.0, heading_deg=0.0)

        # NaN / Inf altitude
        with self.assertRaises(ValueError):
            DroneTelemetry(drone_id="S1", position=self.drone_pos, altitude_agl_m=float("nan"), heading_deg=0.0)
        with self.assertRaises(ValueError):
            DroneTelemetry(drone_id="S1", position=self.drone_pos, altitude_agl_m=float("inf"), heading_deg=0.0)

        # NaN / Inf angles
        with self.assertRaises(ValueError):
            DroneTelemetry(drone_id="S1", position=self.drone_pos, altitude_agl_m=30.0, heading_deg=float("nan"))

        # Camera invalid dimensions
        with self.assertRaises(ValueError):
            CameraCalibration(image_width=0, image_height=1080)
        with self.assertRaises(ValueError):
            CameraCalibration(image_width=1920, image_height=-500)

        # Camera invalid FOV
        with self.assertRaises(ValueError):
            CameraCalibration(hfov_deg=0.0)
        with self.assertRaises(ValueError):
            CameraCalibration(hfov_deg=180.0)
        with self.assertRaises(ValueError):
            CameraCalibration(vfov_deg=0.0)
        with self.assertRaises(ValueError):
            CameraCalibration(vfov_deg=185.0)

        # Non-finite mount angles
        with self.assertRaises(ValueError):
            CameraCalibration(mount_pitch_deg=float("nan"))

    def test_swarm_manager_safeguards_against_invalid_geolocation(self):
        """SwarmManager.process_detection rejects invalid, None, or NaN geolocation estimates."""
        from rescue_swarm_core.swarm_manager import SwarmManager
        from rescue_swarm_core.models import MissionState

        fleet_cfg = ROOT / "config" / "fleet.json"
        manager = SwarmManager.from_config_file(fleet_cfg)
        manager.mission_manager.transition_to(MissionState.INITIALIZING)
        manager.mission_manager.transition_to(MissionState.SEARCHING)

        det = Detection(
            class_id=0,
            class_name="person",
            confidence=0.90,
            bbox=BoundingBox(xmin=0.45, ymin=0.45, xmax=0.55, ymax=0.55),
        )

        # None geo_estimate
        with self.assertRaises(ValueError):
            manager.process_detection(det, None, detected_by_drone_id="S1")

        # Corrupt geo_estimate with None location
        corrupt_estimate = type("CorruptEstimate", (), {
            "location": None,
            "uncertainty_radius_m": 5.0,
        })()
        with self.assertRaises(ValueError):
            manager.process_detection(det, corrupt_estimate, detected_by_drone_id="S1")

        # Verify no task was created
        self.assertEqual(len(manager.get_all_tasks()), 0)
        self.assertEqual(len(manager.get_active_tasks()), 0)


if __name__ == "__main__":
    unittest.main()

