import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from rescue_swarm_core.coverage_planner import (
    CoveragePlan,
    SearchArea,
    compute_ground_swath_width,
    plan_coverage,
)
from rescue_swarm_core.localization import CameraCalibration
from rescue_swarm_core.models import GeoPoint


class CoveragePlannerTests(unittest.TestCase):
    def setUp(self):
        # 200m x 200m bounding search area near Bangalore
        self.area = SearchArea(
            min_lat=12.9710,
            min_lon=77.5940,
            max_lat=12.9730,
            max_lon=77.5960,
        )
        self.camera = CameraCalibration(
            image_width=1920,
            image_height=1080,
            hfov_deg=84.0,
            mount_pitch_deg=-90.0,
        )

    def test_search_area_validation(self):
        with self.assertRaises(ValueError):
            # Inverted latitudes
            SearchArea(min_lat=13.0, min_lon=77.0, max_lat=12.0, max_lon=77.5)

        with self.assertRaises(ValueError):
            # Inverted longitudes
            SearchArea(min_lat=12.0, min_lon=78.0, max_lat=12.5, max_lon=77.0)

    def test_search_area_properties(self):
        center = self.area.center
        self.assertAlmostEqual(center.latitude, 12.9720, places=4)
        self.assertAlmostEqual(center.longitude, 77.5950, places=4)
        self.assertGreater(self.area.height_m, 200.0)
        self.assertGreater(self.area.width_m, 200.0)
        self.assertGreater(self.area.area_m2, 40000.0)

        inside = GeoPoint(12.9720, 77.5950)
        outside = GeoPoint(12.9800, 77.5950)
        self.assertTrue(self.area.contains(inside))
        self.assertFalse(self.area.contains(outside))

    def test_ground_swath_width(self):
        swath_30m = compute_ground_swath_width(self.camera, altitude_agl_m=30.0)
        # For HFOV = 84 deg: tan(42 deg) ~ 0.9004 -> swath = 2 * 30 * 0.9004 = 54.02m
        self.assertAlmostEqual(swath_30m, 54.02, delta=0.5)

        with self.assertRaises(ValueError):
            compute_ground_swath_width(self.camera, altitude_agl_m=0.0)

    def test_multi_drone_coverage_partition(self):
        drones = ["S1", "S2", "S3"]
        plan = plan_coverage(
            search_area=self.area,
            search_drone_ids=drones,
            camera=self.camera,
            altitude_agl_m=30.0,
            overlap_fraction=0.20,
            cruise_speed_mps=8.0,
        )

        self.assertEqual(len(plan.sectors), 3)
        self.assertIn("S1", plan.sectors)
        self.assertIn("S2", plan.sectors)
        self.assertIn("S3", plan.sectors)

        # Ensure sectors are non-overlapping and contiguous along longitude
        s1_area = plan.sectors["S1"].area
        s2_area = plan.sectors["S2"].area
        s3_area = plan.sectors["S3"].area

        self.assertAlmostEqual(s1_area.max_lon, s2_area.min_lon, places=6)
        self.assertAlmostEqual(s2_area.max_lon, s3_area.min_lon, places=6)
        self.assertAlmostEqual(s1_area.min_lon, self.area.min_lon, places=6)
        self.assertAlmostEqual(s3_area.max_lon, self.area.max_lon, places=6)

        # Check waypoints generated for all sectors
        for drone_id in drones:
            sector = plan.sectors[drone_id]
            self.assertGreater(len(sector.waypoints), 0)
            self.assertGreater(sector.total_distance_m, 0.0)
            self.assertGreater(sector.estimated_duration_s, 0.0)
            # All waypoints should lie inside the search area
            for wp in sector.waypoints:
                self.assertTrue(self.area.contains(wp))
                self.assertEqual(wp.altitude_m, 30.0)


if __name__ == "__main__":
    unittest.main()
